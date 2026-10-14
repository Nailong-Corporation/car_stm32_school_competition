#include "line_follow.h"
#include "control_config.h"
#include <math.h>
#include <string.h>

void line_follow_init(LineFollow *follow)
{
    memset(follow, 0, sizeof(*follow));
    follow->state = LINE_IDLE;
    PidConfig config = {PID_KP, PID_KI, PID_KD, PID_INTEGRAL_LIMIT, PID_DERIVATIVE_TAU_S};
    pid_init(&follow->pid, config);
}

void line_follow_stop(LineFollow *follow, LineStopReason reason)
{
    follow->state = LINE_STOPPED;
    /* A later user stop must not clear the latched timing/calculation fault. */
    follow->stop_reason = follow->stop_reason == LINE_STOP_FAULT ? LINE_STOP_FAULT : reason;
    follow->command = (MotorCommand){0};
    follow->base = follow->correction = follow->held_correction = 0.0f;
    follow->have_reliable = follow->abnormal = follow->ambiguity_seen = false;
    follow->direction = 0;
    follow->valid_count = 0;
    pid_reset(&follow->pid);
}

bool line_follow_have_recent_position(const LineFollow *follow, uint32_t now_ms)
{
    return follow->have_reliable &&
           (uint32_t)(now_ms - follow->last_reliable_ms) <= LINE_GAP_MS + LINE_SEARCH_MS;
}

static void output(LineFollow *follow, float target_base, float correction, float dt)
{
    /* Reductions and stop are immediate; only rising PWM is rate limited. */
    follow->base = fminf(target_base, follow->base + LINE_PWM_RISE_PER_SECOND * dt);
    follow->command = motor_mix(follow->base, correction);
    follow->correction = 0.5f * (follow->command.left - follow->command.right);
}

void line_follow_update(LineFollow *follow, GrayObservation obs,
                        uint32_t now_ms, float dt, bool start_request)
{
    if (!isfinite(dt) || dt <= 0.0f || dt > (float)CONTROL_MAX_INTERVAL_MS / 1000.0f ||
        (obs.kind != GRAY_VALID && obs.kind != GRAY_NONE && obs.kind != GRAY_AMBIGUOUS) ||
        (obs.kind == GRAY_VALID && (!isfinite(obs.position) || fabsf(obs.position) > 7.0f))) {
        line_follow_stop(follow, LINE_STOP_FAULT);
        return;
    }
    if (obs.kind == GRAY_VALID) {
        if (follow->valid_count < LINE_CONFIRM_SAMPLES) ++follow->valid_count;
    } else follow->valid_count = 0;
    bool stable = follow->valid_count >= LINE_CONFIRM_SAMPLES;
    if (follow->state == LINE_IDLE || follow->state == LINE_STOPPED) {
        follow->command = (MotorCommand){0};
        if (!start_request || !stable || follow->stop_reason == LINE_STOP_FAULT) return;
        follow->state = LINE_TRACKING;
        follow->stop_reason = LINE_STOP_NONE;
        follow->base = 0.0f;
        follow->have_reliable = false;
        pid_reset(&follow->pid);
    }
    /* Check deadlines BEFORE accepting recovery: noise cannot extend the budget. */
    uint32_t elapsed = (uint32_t)(now_ms - follow->last_reliable_ms);
    if (follow->abnormal &&
        (elapsed >= LINE_GAP_MS + LINE_SEARCH_MS ||
         (follow->ambiguity_seen && elapsed >= LINE_AMBIGUITY_MS))) {
        line_follow_stop(follow, follow->ambiguity_seen ? LINE_STOP_AMBIGUOUS : LINE_STOP_LOST);
        return;
    }
    if (obs.kind == GRAY_VALID && (!follow->abnormal || stable)) {
        if (follow->abnormal) {
            pid_reset(&follow->pid);
            follow->abnormal = follow->ambiguity_seen = false;
        }
        follow->state = LINE_TRACKING;
        if (stable) {
            follow->have_reliable = true;
            follow->reliable_position = obs.position;
            follow->last_reliable_ms = now_ms;
            if (fabsf(obs.position) >= LINE_DIRECTION_DEADBAND) {
                follow->direction = obs.position > 0.0f ? 1 : -1;
                follow->direction_ms = now_ms;
            }
        }
        float target_base = LINE_BASE_PWM -
            (LINE_BASE_PWM - LINE_TURN_BASE_PWM) * fabsf(obs.position) / 7.0f;
        float next_base = fminf(target_base, follow->base + LINE_PWM_RISE_PER_SECOND * dt);
        float correction = 0.0f;
        float limit = fminf(next_base, MOTOR_PWM_LIMIT - next_base);
        if (!pid_update(&follow->pid, obs.position, dt, limit, &correction)) {
            line_follow_stop(follow, LINE_STOP_FAULT);
            return;
        }
        output(follow, target_base, correction, dt);
        return;
    }
    if (!follow->have_reliable) {
        line_follow_stop(follow, LINE_STOP_NO_DIRECTION);
        return;
    }
    if (!follow->abnormal) {
        follow->abnormal = true;
        follow->held_correction = follow->correction;
    }
    if (obs.kind == GRAY_AMBIGUOUS) follow->ambiguity_seen = true;
    elapsed = (uint32_t)(now_ms - follow->last_reliable_ms);
    if (follow->ambiguity_seen) {
        if (elapsed >= LINE_AMBIGUITY_MS) {
            line_follow_stop(follow, LINE_STOP_AMBIGUOUS);
            return;
        }
        follow->state = LINE_GAP_HOLD; /* ambiguity is a sub-mode, never normal PID */
        float decay = 1.0f - (float)elapsed / (float)LINE_AMBIGUITY_MS;
        output(follow, LINE_GAP_BASE_PWM, follow->held_correction * decay, dt);
    } else if (elapsed < LINE_GAP_MS) {
        follow->state = LINE_GAP_HOLD;
        float decay = 1.0f - (float)elapsed / (float)LINE_GAP_MS;
        output(follow, LINE_GAP_BASE_PWM, follow->held_correction * decay, dt);
    } else {
        if (elapsed >= LINE_GAP_MS + LINE_SEARCH_MS) {
            line_follow_stop(follow, LINE_STOP_LOST);
            return;
        }
        if (follow->direction == 0 ||
            (uint32_t)(now_ms - follow->direction_ms) > LINE_DIRECTION_MAX_AGE_MS) {
            line_follow_stop(follow, LINE_STOP_NO_DIRECTION);
            return;
        }
        follow->state = LINE_SEARCH;
        output(follow, LINE_SEARCH_BASE_PWM,
               (float)follow->direction * LINE_SEARCH_CORRECTION, dt);
    }
}
