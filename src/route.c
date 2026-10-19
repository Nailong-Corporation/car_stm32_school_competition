#include "route.h"
#include "motion_config.h"
#include <math.h>
#include <string.h>

bool route_is_narrow_line(GrayObservation obs)
{
    return obs.kind == GRAY_VALID && obs.region_count == 1U &&
        obs.white_count > 0U && obs.white_count <= SENSOR_MAX_REGION_WIDTH;
}

void route_init(RoutePlanner *p, const RouteStep *steps, size_t count,
                 RouteStopCallback callback, void *context)
{
    memset(p, 0, sizeof(*p));
    p->steps = steps;
    p->count = count;
    p->stop_callback = callback;
    p->callback_context = context;
    p->valid = steps != NULL && count > 0U;
    for (size_t i = 0; p->valid && i < count; ++i) {
        RouteStep s = steps[i];
        p->valid = isfinite(s.distance) && isfinite(s.tolerance) &&
            s.distance > 0.0f && s.tolerance >= 0.0f && s.tolerance < s.distance &&
            (unsigned)s.action <= (unsigned)STOP && s.speed > 0 &&
            (float)s.speed <= WHEEL_TARGET_MAX_CM_S &&
            (s.action != STOP || i == count - 1U);
    }
    if (p->valid && steps[count - 1U].action != STOP) p->valid = false;
    p->state = p->valid ? ROUTE_FOLLOW : ROUTE_FAILED;
}

void route_start(RoutePlanner *p, float distance_cm, uint32_t now_ms)
{
    p->index = 0;
    p->origin_cm = distance_cm;
    p->segment_ms = now_ms;
    p->candidate_count = p->exit_count = 0;
    p->stop_pending = p->callback_dispatched = false;
    p->state = p->valid && isfinite(distance_cm) ? ROUTE_FOLLOW : ROUTE_FAILED;
}

static bool junction_matches(GrayObservation obs, Action action)
{
    /* Center plus a branch. A narrow isolated edge is never a junction. */
    bool center = (obs.line_mask & 0x18U) != 0U;
    bool left = (obs.line_mask & 0x03U) != 0U;
    bool right = (obs.line_mask & 0xc0U) != 0U;
    if (!center || obs.white_count < 4U) return false;
    return action == LEFT ? left : action == RIGHT ? right : (left || right);
}

void route_update(RoutePlanner *p, GrayObservation obs, float distance_cm,
                   float heading_rad, uint32_t now_ms)
{
    if (p->state == ROUTE_FINISHED || p->state == ROUTE_FAILED) return;
    if (!p->valid || p->index >= p->count || !isfinite(distance_cm) || !isfinite(heading_rad)) {
        p->state = ROUTE_FAILED;
        return;
    }
    RouteStep s = p->steps[p->index];
    float progress = distance_cm - p->origin_cm;
    if ((p->state == ROUTE_FOLLOW || p->state == ROUTE_CANDIDATE) &&
        (progress > s.distance + s.tolerance ||
         (uint32_t)(now_ms - p->segment_ms) >= ROUTE_SEGMENT_TIMEOUT_MS)) {
        p->state = ROUTE_FAILED; /* missing junction, including missing STOP marker */
        return;
    }
    if (p->state == ROUTE_FOLLOW || p->state == ROUTE_CANDIDATE) {
        bool in_window = progress >= s.distance - s.tolerance && progress <= s.distance + s.tolerance;
        if (!in_window || !junction_matches(obs, s.action)) {
            p->candidate_count = 0;
            p->state = ROUTE_FOLLOW;
            return;
        }
        if (p->state == ROUTE_FOLLOW) {
            p->state = ROUTE_CANDIDATE;
            p->phase_ms = now_ms;
            p->entry_cm = distance_cm;
            p->candidate_count = 0;
        }
        if ((uint32_t)(now_ms - p->phase_ms) >= JUNCTION_CONFIRM_TIMEOUT_MS) {
            p->state = ROUTE_FAILED;
            return;
        }
        if (p->candidate_count < JUNCTION_CONFIRM_SAMPLES) ++p->candidate_count;
        if (p->candidate_count < JUNCTION_CONFIRM_SAMPLES ||
            distance_cm - p->entry_cm < JUNCTION_CONFIRM_DISTANCE_CM) return;
        if (s.action == STOP) {
            p->state = ROUTE_FINISHED;
            p->stop_pending = true;
            return;
        }
        p->state = ROUTE_EXECUTING;
        p->phase_ms = now_ms;
        p->entry_cm = distance_cm;
        p->entry_heading = heading_rad;
        p->exit_count = 0;
        return;
    }
    if ((uint32_t)(now_ms - p->phase_ms) >= JUNCTION_ACTION_TIMEOUT_MS) {
        p->state = ROUTE_FAILED;
        return;
    }
    if (p->state == ROUTE_EXECUTING) {
        float turn = (heading_rad - p->entry_heading) * (s.action == LEFT ? -1.0f : 1.0f);
        if (s.action != STRAIGHT && (turn > JUNCTION_TURN_MAX_RAD || turn < -JUNCTION_WRONG_WAY_RAD)) {
            p->state = ROUTE_FAILED;
            return;
        }
        bool departed = s.action == STRAIGHT ? distance_cm - p->entry_cm >= JUNCTION_CLEAR_DISTANCE_CM :
            turn >= JUNCTION_TURN_MIN_RAD;
        bool centered = route_is_narrow_line(obs) && fabsf(obs.position) <= JUNCTION_CENTER_LIMIT;
        p->exit_count = departed && centered ? (uint8_t)(p->exit_count + 1U) : 0U;
        if (p->exit_count >= LINE_CONFIRM_SAMPLES) {
            p->state = ROUTE_CLEARING;
            p->clear_cm = distance_cm;
            p->exit_count = 0;
        }
    } else if (p->state == ROUTE_CLEARING) {
        if (!route_is_narrow_line(obs)) p->exit_count = 0;
        else if (p->exit_count < LINE_CONFIRM_SAMPLES) ++p->exit_count;
        if (p->exit_count < LINE_CONFIRM_SAMPLES ||
            distance_cm - p->clear_cm < JUNCTION_CLEAR_DISTANCE_CM) return;
        /* Debounce the physical junction before consuming another table entry. */
        ++p->index;
        if (p->index >= p->count) { p->state = ROUTE_FAILED; return; }
        p->origin_cm = distance_cm;
        p->segment_ms = now_ms;
        p->state = ROUTE_FOLLOW;
        p->candidate_count = p->exit_count = 0;
    }
}

void route_dispatch_stop_callback(RoutePlanner *p)
{
    if (!p->stop_pending || p->callback_dispatched) return;
    p->stop_pending = false;
    p->callback_dispatched = true; /* latch BEFORE invoking user code */
    if (p->stop_callback != NULL) p->stop_callback(p->index, p->callback_context);
}
