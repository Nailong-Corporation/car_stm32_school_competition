#include "route.h"
#include "motion_config.h"
#include <math.h>
#include <string.h>

static void fail(RoutePlanner *p, RouteFailure reason)
{
    p->failure = reason;
    p->state = ROUTE_FAILED;
}

/* 只建立路段里程和诊断计时起点；整段行驶不设时间限制。 */
static void begin_segment(RoutePlanner *p, float distance_cm, uint32_t now_ms)
{
    p->origin_cm = distance_cm;
    p->segment_ms = now_ms;
}

/**
 * @brief 判断观测是否为有效的单一窄白线区域，供正常巡线及路口退出确认使用。
 * 多区域即使底层已选出目标，也不会在此被视作可靠窄白线。
 */
bool route_is_narrow_line(GrayObservation obs)
{
    return obs.kind == GRAY_VALID && obs.region_count == 1U &&
        obs.white_count > 0U && obs.white_count <= SENSOR_MAX_REGION_WIDTH;
}

/**
 * @brief 绑定只读路径表和停止回调，并校验距离、容差、速度及动作序列。
 * 最后一项必须为 STOP，表中间不得出现 STOP；不合法时置 ROUTE_FAILED。
 */
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
    p->failure = p->valid ? ROUTE_FAILURE_NONE : ROUTE_FAILURE_INVALID;
}

/**
 * @brief 从当前厘米里程重新开始路径，重置索引、计时与一次性回调标志。
 * now_ms 为启动毫秒时刻；不支持从上次停止的半段路程自动续跑。
 */
void route_start(RoutePlanner *p, float distance_cm, uint32_t now_ms)
{
    p->index = 0;
    p->candidate_count = p->exit_count = 0;
    p->stop_pending = p->callback_dispatched = false;
    p->state = p->valid && isfinite(distance_cm) ? ROUTE_FOLLOW : ROUTE_FAILED;
    p->failure = p->state == ROUTE_FAILED ? ROUTE_FAILURE_INVALID : ROUTE_FAILURE_NONE;
    if (p->state != ROUTE_FAILED) begin_segment(p, distance_cm, now_ms);
}

/**
 * @brief 检查中心白线与目标方向分支是否构成候选路口形状。
 * LEFT/RIGHT 分别要求左/右边缘分支，STRAIGHT/STOP 接受任一侧；不代表已确认路口。
 */
static bool junction_matches(GrayObservation obs, Action action)
{
    /* Center plus a branch. A narrow isolated edge is never a junction. */
    bool center = (obs.line_mask & 0x18U) != 0U;
    bool left = (obs.line_mask & 0x03U) != 0U;
    bool right = (obs.line_mask & 0xc0U) != 0U;
    if (!center || obs.white_count < 4U) return false;
    return action == LEFT ? left : action == RIGHT ? right : (left || right);
}

/**
 * @brief 依据白线形状、厘米里程、航向和毫秒时基推进路口有限状态机。
 * 候选需距离窗口、连续帧和前进距离联合确认；动作退出后还需离口消抖。
 * STOP 确认只产生待分派事件，超时或异常置失败；本函数不直接操作电机或调用回调。
 */
void route_update(RoutePlanner *p, GrayObservation obs, float distance_cm,
                   float heading_rad, uint32_t now_ms)
{
    if (p->state == ROUTE_FINISHED || p->state == ROUTE_FAILED) return;
    if (!p->valid || p->index >= p->count || !isfinite(distance_cm) || !isfinite(heading_rad)) {
        fail(p, ROUTE_FAILURE_INVALID);
        return;
    }
    RouteStep s = p->steps[p->index];
    float progress = distance_cm - p->origin_cm;
    if (p->state == ROUTE_FOLLOW || p->state == ROUTE_CANDIDATE) {
        if (progress > s.distance + s.tolerance) {
            fail(p, ROUTE_FAILURE_WINDOW);
            return;
        }
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
            fail(p, ROUTE_FAILURE_CONFIRM_TIMEOUT);
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
        fail(p, ROUTE_FAILURE_ACTION_TIMEOUT);
        return;
    }
    if (p->state == ROUTE_EXECUTING) {
        float turn = (heading_rad - p->entry_heading) * (s.action == LEFT ? -1.0f : 1.0f);
        if (s.action != STRAIGHT && (turn > JUNCTION_TURN_MAX_RAD || turn < -JUNCTION_WRONG_WAY_RAD)) {
            fail(p, turn < -JUNCTION_WRONG_WAY_RAD ? ROUTE_FAILURE_WRONG_WAY : ROUTE_FAILURE_TURN_LIMIT);
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
        if (p->index >= p->count) { fail(p, ROUTE_FAILURE_INVALID); return; }
        begin_segment(p, distance_cm, now_ms);
        p->state = ROUTE_FOLLOW;
        p->candidate_count = p->exit_count = 0;
    }
}

/**
 * @brief 消费路径 STOP 的待分派事件，并至多调用一次用户回调。
 * 调用者必须先下发电机停车；先锁存分派标志再回调，避免重复调用或重入。
 */
void route_dispatch_stop_callback(RoutePlanner *p)
{
    if (!p->stop_pending || p->callback_dispatched) return;
    p->stop_pending = false;
    p->callback_dispatched = true; /* latch BEFORE invoking user code */
    if (p->stop_callback != NULL) p->stop_callback(p->index, p->callback_context);
}
