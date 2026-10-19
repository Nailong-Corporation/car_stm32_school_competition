#include "line_follow.h"
#include "control_config.h"
#include "motion_config.h"
#include <math.h>
#include <string.h>

static bool locked(LineStopReason reason)
{
    return reason == LINE_STOP_FAULT || reason == LINE_STOP_ENCODER;
}

void line_follow_init(LineFollow *f, OdometryCalibration calibration,
                       const RouteStep *steps, size_t count,
                       RouteStopCallback callback, void *context)
{
    memset(f, 0, sizeof(*f));
    f->state = LINE_IDLE;
    odometry_init(&f->odometry, calibration);
    route_init(&f->route, steps, count, callback, context);
    speed_control_init(&f->speed);
    pid_init(&f->pid, (PidConfig){POSITION_KP_CM_S, POSITION_KI_CM_S, POSITION_KD_CM_S,
                                  PID_INTEGRAL_LIMIT, PID_DERIVATIVE_TAU_S});
}

void line_follow_stop(LineFollow *f, LineStopReason reason)
{
    f->state = LINE_STOPPED;
    if (!locked(f->stop_reason)) f->stop_reason = reason;
    f->command = (MotorCommand){0};
    f->left_target_cm_s = f->right_target_cm_s = f->base = f->correction = 0.0f;
    f->have_reliable = f->abnormal = false;
    f->valid_count = 0;
    pid_reset(&f->pid);
    speed_control_reset(&f->speed);
    if (reason != LINE_STOP_ROUTE_COMPLETE) f->route.stop_pending = false;
}

bool line_follow_have_recent_position(const LineFollow *f, uint32_t now_ms)
{
    return f->have_reliable && (uint32_t)(now_ms - f->last_reliable_ms) < INTERFERENCE_TIMEOUT_MS;
}

static void targets(LineFollow *f, float base, float correction, float dt)
{
    f->base = fminf(base, f->base + TARGET_RISE_CM_S2 * dt);
    float limit = fminf(f->base, WHEEL_TARGET_MAX_CM_S - f->base);
    f->correction = fmaxf(-limit, fminf(limit, correction));
    f->left_target_cm_s = f->base + f->correction;
    f->right_target_cm_s = f->base - f->correction;
}

void line_follow_update(LineFollow *f, GrayObservation obs, EncoderCounts counts,
                        uint32_t now_ms, float dt, bool start_request)
{
    if (!isfinite(dt) || dt <= 0.0f || dt > (float)CONTROL_MAX_INTERVAL_MS / 1000.0f ||
        (obs.kind != GRAY_VALID && obs.kind != GRAY_NONE && obs.kind != GRAY_AMBIGUOUS) ||
        (obs.kind == GRAY_VALID && (!isfinite(obs.position) || fabsf(obs.position) > 7.0f))) {
        line_follow_stop(f, LINE_STOP_FAULT);
        return;
    }
    if (!f->odometry.ready) {
        if (start_request) line_follow_stop(f, LINE_STOP_CALIBRATION);
        return;
    }
    if (!odometry_update(&f->odometry, counts, dt)) {
        line_follow_stop(f, LINE_STOP_ENCODER);
        return;
    }
    bool narrow = route_is_narrow_line(obs);
    if (narrow) {
        if (f->valid_count < LINE_CONFIRM_SAMPLES) ++f->valid_count;
    } else f->valid_count = 0;
    bool stable = f->valid_count >= LINE_CONFIRM_SAMPLES;
    if (f->state == LINE_IDLE || f->state == LINE_STOPPED) {
        if (!start_request || !stable || locked(f->stop_reason)) return;
        if (!f->route.valid) { line_follow_stop(f, LINE_STOP_ROUTE); return; }
        route_start(&f->route, f->odometry.distance_cm, now_ms);
        f->state = LINE_TRACKING;
        f->stop_reason = LINE_STOP_NONE;
        f->last_reliable_ms = now_ms;
        f->have_reliable = true;
        f->base = 0.0f;
        pid_reset(&f->pid);
        speed_control_reset(&f->speed);
    }
    /* Deadline precedes recovery/junction confirmation; isolated noise cannot extend it. */
    if (f->abnormal && ((uint32_t)(now_ms - f->last_reliable_ms) >= INTERFERENCE_TIMEOUT_MS ||
        fabsf(f->odometry.distance_cm - f->abnormal_origin_cm) >= INTERFERENCE_MAX_DISTANCE_CM)) {
        line_follow_stop(f, LINE_STOP_LOST);
        return;
    }
    RouteState previous_route = f->route.state;
    route_update(&f->route, obs, f->odometry.distance_cm, f->odometry.heading_rad, now_ms);
    if (f->route.state == ROUTE_FAILED) { line_follow_stop(f, LINE_STOP_ROUTE); return; }
    if (f->route.state == ROUTE_FINISHED) { line_follow_stop(f, LINE_STOP_ROUTE_COMPLETE); return; }
    if (previous_route == ROUTE_EXECUTING && f->route.state == ROUTE_CLEARING) {
        /* A completed, confirmed maneuver establishes a fresh line history. */
        pid_reset(&f->pid);
        speed_control_reset(&f->speed);
        f->abnormal = false;
        f->valid_count = 0;
        f->last_reliable_ms = now_ms;
        stable = false;
    }
    RouteStep step = f->route.steps[f->route.index];
    if (f->route.state == ROUTE_EXECUTING) {
        if (previous_route != ROUTE_EXECUTING) {
            pid_reset(&f->pid);
            speed_control_reset(&f->speed);
        }
        f->abnormal = false;
        f->state = LINE_JUNCTION;
        float correction = step.action == LEFT ? -JUNCTION_TURN_DIFF_CM_S :
                           step.action == RIGHT ? JUNCTION_TURN_DIFF_CM_S :
                           HEADING_KP_CM_S_PER_RAD * (f->route.entry_heading - f->odometry.heading_rad);
        targets(f, fminf((float)step.speed, JUNCTION_TURN_SPEED_CM_S), correction, dt);
    } else if (narrow && (stable || !f->abnormal) && f->route.state != ROUTE_CANDIDATE) {
        if (f->abnormal) {
            pid_reset(&f->pid);
            speed_control_reset(&f->speed);
            f->abnormal = false;
        }
        f->state = LINE_TRACKING;
        if (stable) {
            f->have_reliable = true;
            f->last_reliable_ms = now_ms;
            f->reliable_position = obs.position;
        }
        float base = (float)step.speed * (1.0f - POSITION_CURVE_SLOWDOWN * fabsf(obs.position) / 7.0f);
        float next = fminf(base, f->base + TARGET_RISE_CM_S2 * dt);
        float correction;
        if (!pid_update(&f->pid, obs.position, dt,
                        fminf(next, WHEEL_TARGET_MAX_CM_S - next), &correction)) {
            line_follow_stop(f, LINE_STOP_FAULT);
            return;
        }
        targets(f, base, correction, dt);
    } else {
        /* Unconfirmed junctions, wide/multiple regions and no-line all hold heading.
         * Do not integrate graphics into the normal white-line position PID. */
        if (!f->abnormal) {
            f->abnormal = true;
            f->hold_heading_rad = f->odometry.heading_rad;
            f->abnormal_origin_cm = f->odometry.distance_cm;
        }
        f->state = LINE_GAP_HOLD;
        targets(f, fminf((float)step.speed, HOLD_SPEED_CM_S),
                HEADING_KP_CM_S_PER_RAD * (f->hold_heading_rad - f->odometry.heading_rad), dt);
    }
    if (!speed_control_update(&f->speed, f->left_target_cm_s, f->right_target_cm_s,
                              f->odometry.left_speed_cm_s, f->odometry.right_speed_cm_s,
                              dt, &f->command)) line_follow_stop(f, LINE_STOP_ENCODER);
}
