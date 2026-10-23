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
    p->have_approach_center = false;
    p->approach_center_count = 0;
    p->narrow_count = 0;
    p->turn_forced = false;
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
    p->repeat_count = ROUTE_REPEAT_COUNT;
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
    p->completed_laps = 0;
    p->turn_capturing = p->lap_boundary = false;
    p->turn_forced = false;
    p->narrow_count = 0;
    p->candidate_count = p->exit_count = 0;
    p->stop_pending = p->callback_dispatched = false;
    p->state = p->valid && isfinite(distance_cm) ? ROUTE_FOLLOW : ROUTE_FAILED;
    p->failure = p->state == ROUTE_FAILED ? ROUTE_FAILURE_INVALID : ROUTE_FAILURE_NONE;
    if (p->state != ROUTE_FAILED) begin_segment(p, distance_cm, now_ms);
}

/* 循环边界复用直行与离口确认，避免同一标记被再次消费或发生瞬时重启。 */
Action route_current_action(const RoutePlanner *p)
{
    return p->lap_boundary ? STRAIGHT : p->steps[p->index].action;
}

/**
 * @brief 检查中心白线与目标方向分支是否构成候选路口形状。
 * LEFT/RIGHT 分别要求左/右边缘分支，STRAIGHT/STOP 接受任一侧；不代表已确认路口。
 */
static bool junction_matches(const RoutePlanner *p, GrayObservation obs, Action action,
                              float distance_cm, uint32_t now_ms)
{
    bool center = (obs.line_mask & 0x18U) != 0U;
    bool left = (obs.line_mask & 0x03U) != 0U;
    bool right = (obs.line_mask & 0xc0U) != 0U;
    bool branch = action == LEFT ? left : action == RIGHT ? right : (left || right);
    if (!branch) return false;
    if (center && obs.white_count >= 4U) return true;
    /* L 弯不一定同时覆盖四路；只对指定左右转允许窄分支的时序证据。
     * STOP/STRAIGHT 维持原来的同时中心+分支要求，防止提前消费终点标记。 */
    if (action != LEFT && action != RIGHT) return false;
    if (!route_is_narrow_line(obs) || obs.white_count < 2U) return false;
    if (center && obs.white_count >= 3U) return true;
    return p->have_approach_center &&
        (uint32_t)(now_ms - p->approach_center_ms) < JUNCTION_CENTER_MEMORY_MS &&
        distance_cm >= p->approach_center_cm &&
        distance_cm - p->approach_center_cm <= JUNCTION_CENTER_MEMORY_CM;
}

static unsigned bit_count(uint8_t mask)
{
    unsigned count = 0;
    for (; mask != 0U; mask = (uint8_t)(mask & (uint8_t)(mask - 1U))) ++count;
    return count;
}

/* 仅对相邻滤波帧判断骤变；稳定窄线是入口证据，本帧无需在线或有指定侧分支。 */
static bool row_changed_abruptly(const RoutePlanner *p, GrayObservation obs, uint32_t now_ms)
{
    if (p->narrow_count < LINE_CONFIRM_SAMPLES ||
        (uint32_t)(now_ms - p->previous_line_ms) > CONTROL_MAX_INTERVAL_MS) return false;
    if (obs.line_mask == 0U) return true;
    if (obs.white_count < 2U) return false; /* 单个边缘噪点不作为整排骤变。 */
    unsigned changed = bit_count((uint8_t)(obs.line_mask ^ p->previous_line_mask));
    return changed >= JUNCTION_ABRUPT_CHANGED_BITS ||
        obs.white_count >= bit_count(p->previous_line_mask) + JUNCTION_ABRUPT_WIDTH_GROWTH ||
        (changed >= 2U && !route_is_narrow_line(obs)) ||
        (route_is_narrow_line(obs) &&
         fabsf(obs.position - p->previous_line_position) >= JUNCTION_ABRUPT_POSITION_JUMP);
}

static void remember_line(RoutePlanner *p, GrayObservation obs, uint32_t now_ms)
{
    if ((uint32_t)(now_ms - p->previous_line_ms) > CONTROL_MAX_INTERVAL_MS)
        p->narrow_count = 0;
    if (route_is_narrow_line(obs)) {
        if (p->narrow_count < LINE_CONFIRM_SAMPLES) ++p->narrow_count;
    } else p->narrow_count = 0;
    p->previous_line_mask = obs.line_mask;
    p->previous_line_position = obs.position;
    p->previous_line_ms = now_ms;
}

static void begin_action(RoutePlanner *p, float distance_cm, float heading_rad,
                         uint32_t now_ms, bool forced)
{
    p->state = ROUTE_EXECUTING;
    p->phase_ms = now_ms;
    p->entry_cm = distance_cm;
    p->entry_heading = heading_rad;
    p->exit_candidate_pending = false;
    p->outgoing_cm = 0.0f;
    p->projection_previous_cm = distance_cm;
    p->projection_previous_heading = heading_rad;
    p->turn_capturing = false;
    p->turn_forced = forced;
    p->exit_count = 0;
}

/**
 * @brief 依据白线形状、厘米里程、航向和毫秒时基推进路口有限状态机。
 * 窗口内 LEFT/RIGHT 的整排骤变优先按表执行，其余走连续帧/前进距离候选确认。
 * 动作退出后还需离口消抖。
 * 最终轮 STOP 确认产生待分派事件，中间轮按直行离口后回到首项。
 * 超时或异常置失败；本函数不直接操作电机或调用回调。
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
        bool forced = in_window && (s.action == LEFT || s.action == RIGHT) &&
            row_changed_abruptly(p, obs, now_ms);
        remember_line(p, obs, now_ms);
        if (forced) {
            /* 不再等中心/指定分支、候选帧数或普通 PID 的在线判断。方向只取路径表。 */
            p->candidate_count = 0;
            begin_action(p, distance_cm, heading_rad, now_ms, true);
            return;
        }
        bool centered_line = in_window && route_is_narrow_line(obs) &&
            fabsf(obs.position) <= JUNCTION_CENTER_LIMIT;
        if (centered_line) {
            if (p->approach_center_count < LINE_CONFIRM_SAMPLES) ++p->approach_center_count;
            if (p->approach_center_count >= LINE_CONFIRM_SAMPLES) {
                p->have_approach_center = true;
                p->approach_center_ms = now_ms;
                p->approach_center_cm = distance_cm;
            }
        } else p->approach_center_count = 0;
        if (!in_window || !junction_matches(p, obs, s.action, distance_cm, now_ms)) {
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
            if (p->completed_laps < UINT32_MAX) ++p->completed_laps;
            if (p->repeat_count != 0U && p->completed_laps >= p->repeat_count) {
                p->state = ROUTE_FINISHED;
                p->stop_pending = true;
                return;
            }
            p->lap_boundary = true;
        }
        begin_action(p, distance_cm, heading_rad, now_ms, false);
        return;
    }
    uint32_t phase_timeout = p->state == ROUTE_CLEARING ? JUNCTION_CLEAR_TIMEOUT_MS :
                             JUNCTION_ACTION_TIMEOUT_MS;
    if ((uint32_t)(now_ms - p->phase_ms) >= phase_timeout) {
        fail(p, ROUTE_FAILURE_ACTION_TIMEOUT);
        return;
    }
    Action action = route_current_action(p);
    /* 将车轴中心增量投影至入口方向 ±90° 的出口轴；圆弧不等于下一直线长度。
     * 动作触发点作为路口参考点，灰度排到轮轴的偏移/打滑仍需实测校准。
     * 中点航向 + sinc 对单次恒曲率增量精确积分，也兼容直行和零半径自转。 */
    float dh = heading_rad - p->projection_previous_heading;
    float half = dh * 0.5f;
    float sinc = fabsf(half) < 0.00001f ? 1.0f : sinf(half) / half;
    float exit_heading = p->entry_heading +
        (action == LEFT ? -JUNCTION_TURN_TARGET_RAD :
         action == RIGHT ? JUNCTION_TURN_TARGET_RAD : 0.0f);
    p->outgoing_cm += (distance_cm - p->projection_previous_cm) * sinc *
        cosf((heading_rad + p->projection_previous_heading) * 0.5f - exit_heading);
    p->projection_previous_cm = distance_cm;
    p->projection_previous_heading = heading_rad;
    if (p->state == ROUTE_EXECUTING) {
        float turn = (heading_rad - p->entry_heading) * (action == LEFT ? -1.0f : 1.0f);
        if (action != STRAIGHT && (turn > JUNCTION_TURN_MAX_RAD || turn < -JUNCTION_WRONG_WAY_RAD)) {
            fail(p, turn < -JUNCTION_WRONG_WAY_RAD ? ROUTE_FAILURE_WRONG_WAY : ROUTE_FAILURE_TURN_LIMIT);
            return;
        }
        /* 侧边窄线先交给位置反馈捕获；全目标/多区域不能进入捕获 PID。
         * 最终退出仍要求足够转角和连续中心线，不能用提前遇到的图形直接退出。 */
        p->turn_capturing = action != STRAIGHT && turn >= JUNCTION_CAPTURE_MIN_RAD &&
            route_is_narrow_line(obs);
        bool departed = action == STRAIGHT ? distance_cm - p->entry_cm >= JUNCTION_CLEAR_DISTANCE_CM :
            turn >= JUNCTION_TURN_MIN_RAD;
        bool centered = route_is_narrow_line(obs) && fabsf(obs.position) <= JUNCTION_CENTER_LIMIT;
        /* 捕获期间允许窄出口线横移；仅最终退出要求居中。
         * 若计时也要求居中，位置 2/3 来回变化会反复清零，直至动作超时。 */
        bool near_exit = action != STRAIGHT && route_is_narrow_line(obs) &&
            turn >= JUNCTION_TURN_MIN_RAD - JUNCTION_EXIT_ANGLE_MARGIN_RAD;
        if (!near_exit) p->exit_candidate_pending = false;
        else if (!p->exit_candidate_pending) {
            p->exit_candidate_pending = true;
            p->exit_candidate_ms = now_ms;
            p->exit_candidate_cm = distance_cm;
        }
        /* 实测转角差约 0.003 rad 时，1.6 cm/s 目标轮差未形成足够实际轮差。
         * 仅在接近角度门槛且连续沿中心出口线前进后，接受有界估计误差。 */
        if (near_exit && (uint32_t)(now_ms - p->exit_candidate_ms) >= JUNCTION_EXIT_CONFIRM_MS &&
            distance_cm - p->exit_candidate_cm >= JUNCTION_EXIT_CONFIRM_CM) departed = true;
        p->exit_count = departed && centered ? (uint8_t)(p->exit_count + 1U) : 0U;
        if (p->exit_count >= LINE_CONFIRM_SAMPLES) {
            p->state = ROUTE_CLEARING;
            p->phase_ms = now_ms; /* 已确认出口后，离口使用独立且有界的时间预算。 */
            p->turn_capturing = false;
            p->clear_cm = distance_cm;
            p->exit_count = 0;
        }
    } else if (p->state == ROUTE_CLEARING) {
        if (!route_is_narrow_line(obs)) p->exit_count = 0;
        else if (p->exit_count < LINE_CONFIRM_SAMPLES) ++p->exit_count;
        if (p->exit_count < LINE_CONFIRM_SAMPLES ||
            distance_cm - p->clear_cm < JUNCTION_CLEAR_DISTANCE_CM) return;
        /* Debounce the physical junction before consuming another table entry. */
        if (p->lap_boundary) {
            p->index = 0;
            p->lap_boundary = false;
        } else ++p->index;
        p->turn_capturing = false;
        if (p->index >= p->count) { fail(p, ROUTE_FAILURE_INVALID); return; }
        begin_segment(p, distance_cm - p->outgoing_cm, now_ms);
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
