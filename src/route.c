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
    p->recovery_window_active = false;
    p->segment_ms = now_ms;
    p->have_approach_center = false;
    p->approach_center_count = 0;
    p->narrow_count = 0;
    p->turn_forced = false;
    p->corner_arc_complete = false;
    p->handoff_early = false;
    p->capture_line_seen = p->line_exit_confirmed = false;
    p->clear_center_pending = false;
    p->segment_start_progress_cm = 0.0f;
    p->turn_exit_branch = false;
    p->exit_line_mask = 0;
    p->online_samples = 0;
    p->online_ms = 0;
    p->turn_ended_on_line = false;
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

bool route_waiting_after_straight(const RoutePlanner *p, float distance_cm)
{
    if (!p->valid || p->index == 0U || p->index >= p->count ||
        p->steps[p->index - 1U].action != STRAIGHT) return false;
    Action action = p->steps[p->index].action;
    return (action == LEFT || action == RIGHT || action == CUSTOM_TURN) &&
        distance_cm - p->origin_cm < p->steps[p->index].distance;
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
    p->repeat_action = ROUTE_REPEAT_ACTION;
    p->stop_callback = callback;
    p->callback_context = context;
    p->valid = steps != NULL && count > 0U;
    for (size_t i = 0; p->valid && i < count; ++i) {
        RouteStep s = steps[i];
        p->valid = isfinite(s.distance) && isfinite(s.tolerance) &&
            s.distance > 0.0f && s.tolerance >= 0.0f && s.tolerance < s.distance &&
            (unsigned)s.action <= (unsigned)CUSTOM_TURN && s.speed > 0 &&
            (s.action != CUSTOM_TURN || (isfinite(s.angle_deg) &&
             s.angle_deg != 0.0f && fabsf(s.angle_deg) <= 360.0f)) &&
            (float)s.speed <= WHEEL_TARGET_MAX_CM_S &&
            (s.action != STOP || i == count - 1U) &&
            (!s.corner_arc_enabled || (isfinite(CORNER_RADIUS_CM) &&
             CORNER_RADIUS_CM > WHEEL_TRACK_CM * 0.5f &&
             isfinite(GRAY_FORWARD_OFFSET_CM) && GRAY_FORWARD_OFFSET_CM >= CORNER_RADIUS_CM));
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
    p->segment_heading_valid = false;
    p->turn_capturing = p->lap_boundary = false;
    p->turn_forced = false;
    p->corner_arc_complete = false;
    p->narrow_count = 0;
    p->candidate_count = p->exit_count = 0;
    p->stop_pending = p->callback_dispatched = false;
    p->state = p->valid && isfinite(distance_cm) ? ROUTE_FOLLOW : ROUTE_FAILED;
    p->failure = p->state == ROUTE_FAILED ? ROUTE_FAILURE_INVALID : ROUTE_FAILURE_NONE;
    if (p->state != ROUTE_FAILED) begin_segment(p, distance_cm, now_ms);
}

/* 最终轮停车，其余轮从入口形状判断开始就使用闭合动作。 */
static bool another_lap(const RoutePlanner *p)
{
    return p->repeat_count == 0U || p->completed_laps < p->repeat_count - 1U;
}
Action route_current_action(const RoutePlanner *p)
{
    Action action = p->steps[p->index].action;
    if (action == CUSTOM_TURN) return p->steps[p->index].angle_deg < 0.0f ? LEFT : RIGHT;
    return action == STOP && (p->lap_boundary || another_lap(p)) ? p->repeat_action : action;
}

/* 普通左右转、闭合左右转及显式 ±90° 共用切角；其他自定义角度保持原策略。 */
bool route_uses_corner_arc(const RoutePlanner *p)
{
    return p->steps[p->index].corner_arc_enabled &&
        fabsf(fabsf(route_turn_target_rad(p)) - JUNCTION_TURN_TARGET_RAD) < 0.0001f;
}

bool route_prepares_straight(const RoutePlanner *p)
{
    if (!p->valid || p->index >= p->count || !route_uses_corner_arc(p)) return false;
    size_t next_index = p->lap_boundary ? 0U : p->index + 1U;
    return next_index < p->count && p->steps[next_index].action == STRAIGHT;
}

/* 编码器航向本身右转为正，不对 ±180°/360° 取模，以保留实际转动方向与圈数。 */
float route_turn_target_rad(const RoutePlanner *p)
{
    if (p->steps[p->index].action == CUSTOM_TURN)
        return p->steps[p->index].angle_deg * (JUNCTION_TURN_TARGET_RAD / 90.0f);
    Action action = route_current_action(p);
    return action == LEFT ? -JUNCTION_TURN_TARGET_RAD :
           action == RIGHT ? JUNCTION_TURN_TARGET_RAD : 0.0f;
}

float route_turn_threshold_rad(const RoutePlanner *p, float threshold)
{
    if (p->steps[p->index].action != CUSTOM_TURN) return threshold;
    float target = fabsf(route_turn_target_rad(p));
    /* 自定义角度正常退出须到达目标；慢扫/捕获/过转沿用直角参数的角度余量。
     * 小角度按比例压缩，避免门槛为负。 */
    if (threshold == JUNCTION_TURN_MIN_RAD) return target;
    if (threshold > JUNCTION_TURN_TARGET_RAD)
        return target + threshold - JUNCTION_TURN_TARGET_RAD;
    return fmaxf(target + threshold - JUNCTION_TURN_TARGET_RAD,
                 target * threshold / JUNCTION_TURN_TARGET_RAD);
}

uint32_t route_action_timeout_ms(const RoutePlanner *p)
{
    if (route_current_action(p) == STRAIGHT) return JUNCTION_STRAIGHT_TIMEOUT_MS;
    float scale = fmaxf(1.0f, fabsf(route_turn_target_rad(p)) / JUNCTION_TURN_TARGET_RAD);
    return (uint32_t)ceilf((float)JUNCTION_ACTION_TIMEOUT_MS * scale);
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
    /* 剩余细缝被过滤不是“真实全排丢线”，不能触发路径表强制转向。
     * raw/prefilter 原本就为零的真实间隙仍按原规则处理。 */
    if (obs.line_mask==0U && obs.seam_mask!=0U) return false;
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
    /* 短段可能在上一弯尚未对齐时到达；保持路径出口坐标轴，不累加残余偏航。 */
    p->entry_heading = p->segment_heading_valid ? p->segment_heading_rad : heading_rad;
    p->action_start_heading = heading_rad;
    p->exit_candidate_pending = false;
    p->outgoing_cm = 0.0f;
    if (p->steps[p->index].action == STRAIGHT) {
        /* STRAIGHT 是按表通过的里程点，图形内部不要求存在中心线。
         * 入口窗口可提前/滞后触发，下一项仍从本项标称点起算。 */
        p->entry_cm = p->origin_cm + p->steps[p->index].distance;
        p->outgoing_cm = distance_cm - p->entry_cm;
    }
    p->projection_previous_cm = distance_cm;
    p->projection_previous_heading = heading_rad;
    p->turn_capturing = false;
    p->turn_forced = forced;
    p->capture_line_seen = p->line_exit_confirmed = false;
    p->capture_line_ms = 0;
    p->clear_center_pending = false;
    p->clear_sample_ms = now_ms;
    p->corner_arc_complete = false;
    p->exit_count = 0;
    p->online_samples = 0;
    p->online_ms = now_ms;
    p->turn_ended_on_line = false;
}

/* 普通确认与骤变入口共用计轮逻辑，末项不能漏计或重复计轮。 */
static void accept_step(RoutePlanner *p, float distance_cm, float heading_rad,
                        uint32_t now_ms, bool forced)
{
    if (p->steps[p->index].action == STOP) {
        bool repeat = another_lap(p);
        if (p->completed_laps < UINT32_MAX) ++p->completed_laps;
        if (!repeat) {
            p->state = ROUTE_FINISHED;
            p->stop_pending = true;
            return;
        }
        p->lap_boundary = true;
    }
    begin_action(p, distance_cm, heading_rad, now_ms, forced);
}

/* 仅已确认转弯的出口处理可识别此结构；普通巡线/候选仍保留多区域歧义。
 * 中心连续区域必须同时覆盖 S3/S4，附加区域只能位于转弯反侧最外两路。
 * 这可能是尚在探头中的入口残支，不声称对图形或路口作语义识别。 */
static bool branch_exit_line(const RoutePlanner *p, GrayObservation obs, float heading,
                             GrayObservation *exit_obs)
{
    if ((obs.kind != GRAY_AMBIGUOUS && obs.kind != GRAY_VALID) || obs.region_count != 2U ||
        (obs.line_mask & 0x18U) != 0x18U) return false;
    float target = route_turn_target_rad(p);
    float turn = (heading-p->entry_heading) * (target < 0 ? -1.0f : 1.0f);
    float angle = fabsf(target);
    if (angle < JUNCTION_CAPTURE_MIN_RAD ||
        turn < angle-fminf(JUNCTION_EXIT_ANGLE_MARGIN_RAD,angle*.1f) ||
        turn > angle+JUNCTION_BRANCH_EXIT_OVERRUN_RAD) return false;
    size_t next = p->lap_boundary ? 0U : p->index+1U;
    if (next >= p->count ||
        p->outgoing_cm < p->steps[next].distance-JUNCTION_APPROACH_DISTANCE_CM ||
        p->outgoing_cm > p->steps[next].distance+p->steps[next].tolerance) return false;
    unsigned left=3, right=4;
    while (left>0 && (obs.line_mask & (1U<<(left-1U)))) --left;
    while (right<7 && (obs.line_mask & (1U<<(right+1U)))) ++right;
    uint8_t center=(uint8_t)(((1U<<(right-left+1U))-1U)<<left);
    uint8_t extra=(uint8_t)(obs.line_mask & (uint8_t)~center);
    uint8_t edge=target<0 ? 0xc0U : 0x03U;
    if (extra==0U || (extra & (uint8_t)~edge)!=0U) return false;
    GrayObservation selected=gray_classify(center,false,0);
    if (!route_is_narrow_line(selected) || fabsf(selected.position)>JUNCTION_CENTER_LIMIT) return false;
    *exit_obs=selected;
    return true;
}

static void project_outgoing(RoutePlanner *p, float distance_cm, float heading_rad)
{
    /* 将车轴中心增量投影至路径目标航向的出口轴；圆弧不等于下一直线长度。
     * 入口灰度中心近似路口参考点；额外积分前置探头因旋转产生的位移。
     * 不修改原始轮程/轴中点弧长，也不把探头弧长当作路口间距离。
     * 中点航向 + sinc 对单次恒曲率增量精确积分，也兼容直行和零半径自转。 */
    float dh = heading_rad - p->projection_previous_heading;
    float half = dh * 0.5f;
    float sinc = fabsf(half) < 0.00001f ? 1.0f : sinf(half) / half;
    float exit_heading = p->entry_heading + route_turn_target_rad(p);
    p->outgoing_cm += (distance_cm - p->projection_previous_cm) * sinc *
        cosf((heading_rad + p->projection_previous_heading) * 0.5f - exit_heading);
    p->outgoing_cm += GRAY_FORWARD_OFFSET_CM *
        (cosf(heading_rad - exit_heading) -
         cosf(p->projection_previous_heading - exit_heading));
    p->projection_previous_cm = distance_cm;
    p->projection_previous_heading = heading_rad;
}

bool route_resume(RoutePlanner *p, RouteState failed_phase, float distance_cm,
                  float heading_rad, uint32_t now_ms)
{
    if (!p->valid || p->index >= p->count || p->state != ROUTE_FAILED ||
        !isfinite(distance_cm) || !isfinite(heading_rad)) return false;
    bool follow = (failed_phase == ROUTE_FOLLOW || failed_phase == ROUTE_CANDIDATE) &&
        (p->failure == ROUTE_FAILURE_WINDOW || p->failure == ROUTE_FAILURE_CONFIRM_TIMEOUT);
    bool clearing = (failed_phase == ROUTE_EXECUTING || failed_phase == ROUTE_CLEARING) &&
        p->failure == ROUTE_FAILURE_ACTION_TIMEOUT;
    if (!follow && !clearing) return false;
    /* 动作超时后的自动恢复也不能直接跳过未完成的普通90°转向。 */
    if (clearing && failed_phase == ROUTE_EXECUTING && !route_uses_corner_arc(p) &&
        p->steps[p->index].action != CUSTOM_TURN) {
        Action action=route_current_action(p);
        if (action==LEFT || action==RIGHT) {
            float turn=(heading_rad-p->entry_heading)*(action==LEFT?-1.0f:1.0f);
            if (turn < fabsf(route_turn_target_rad(p)) || turn > JUNCTION_TURN_MAX_RAD)
                return false;
        }
    }
    if (follow) {
        p->state = ROUTE_FOLLOW;
        p->recovery_window_active = true;
        p->recovery_window_origin_cm = distance_cm;
        p->have_approach_center = false;
        p->approach_center_count = p->narrow_count = p->candidate_count = 0;
        p->previous_line_ms = now_ms;
    } else {
        project_outgoing(p, distance_cm, heading_rad);
        p->state = ROUTE_CLEARING;
        p->clear_cm = distance_cm;
        p->turn_ended_on_line = true;
    }
    p->phase_ms = now_ms;
    p->failure = ROUTE_FAILURE_NONE;
    p->exit_count = p->online_samples = 0;
    p->exit_candidate_pending = p->turn_capturing = p->turn_exit_branch = false;
    p->clear_center_pending = false;
    p->clear_sample_ms = now_ms;
    p->stop_pending = false;
    return true;
}

/**
 * @brief 依据白线形状、厘米里程、航向和毫秒时基推进路口有限状态机。
 * 显式 STRAIGHT 按表内里程通过；LEFT/RIGHT 骤变优先，其余转弯/STOP 走候选确认。
 * 动作退出后还需离口消抖。
 * 最终轮 STOP 确认产生待分派事件，中间轮按闭合动作及离口后回到首项。
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
    if (s.action == STRAIGHT && !p->segment_heading_valid) {
        p->segment_heading_rad = heading_rad;
        p->segment_heading_valid = true;
    }
    float progress = distance_cm - p->origin_cm;
    float window_progress = p->recovery_window_active ? distance_cm-p->recovery_window_origin_cm : progress;
    float window_low = p->recovery_window_active ? 0.0f : s.distance-s.tolerance;
    float window_high = p->recovery_window_active ?
        fmaxf(2.0f*s.tolerance, GRAY_FORWARD_OFFSET_CM) : s.distance+s.tolerance;
    if (p->state == ROUTE_FOLLOW || p->state == ROUTE_CANDIDATE) {
        if (window_progress > window_high) {
            fail(p, ROUTE_FAILURE_WINDOW);
            return;
        }
    }
    if (p->state == ROUTE_FOLLOW || p->state == ROUTE_CANDIDATE) {
        bool in_window = window_progress >= window_low && window_progress <= window_high;
        /* 末项的大容差不能使终点提前降速/转向；最早从标称位置前减速区开始确认。 */
        if (!p->recovery_window_active && s.action == STOP &&
            progress < s.distance - JUNCTION_APPROACH_DISTANCE_CM)
            in_window = false;
        Action action = route_current_action(p);
        /* STRAIGHT 后的侧线/出口横线不能借用下一转弯的提前容差。
         * 必须走满本段标称距离，再允许强制入口或普通形状确认；自动恢复也不绕过。 */
        if (route_waiting_after_straight(p, distance_cm))
            in_window = false;
        if (in_window && s.action == STRAIGHT) {
            accept_step(p, distance_cm, heading_rad, now_ms, false);
            return;
        }
        bool forced = in_window && (action == LEFT || action == RIGHT) &&
            row_changed_abruptly(p, obs, now_ms);
        remember_line(p, obs, now_ms);
        if (forced) {
            /* 不再等中心/指定分支、候选帧数或普通 PID 的在线判断。方向只取路径表。 */
            p->candidate_count = 0;
            accept_step(p, distance_cm, heading_rad, now_ms, forced);
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
        if (!in_window || !junction_matches(p, obs, action, distance_cm, now_ms)) {
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
        accept_step(p, distance_cm, heading_rad, now_ms, false);
        return;
    }
    uint32_t phase_timeout = p->state == ROUTE_CLEARING ? JUNCTION_CLEAR_TIMEOUT_MS :
                             route_action_timeout_ms(p);
    bool timed_out = (uint32_t)(now_ms - p->phase_ms) >= phase_timeout;
    if (p->state == ROUTE_CLEARING && timed_out) {
        fail(p, ROUTE_FAILURE_ACTION_TIMEOUT);
        return;
    }
    Action action = route_current_action(p);
    project_outgoing(p, distance_cm, heading_rad);
    float exit_heading = p->entry_heading + route_turn_target_rad(p);
    GrayObservation exit_obs=obs;
    bool previous_branch=p->turn_exit_branch;
    p->turn_exit_branch=branch_exit_line(p,obs,heading_rad,&exit_obs);
    p->exit_line_mask=route_is_narrow_line(exit_obs) ? exit_obs.line_mask : 0;
    if (p->turn_exit_branch && !previous_branch) {
        p->exit_candidate_pending=false;
        p->exit_count=0; /* 不能借用此前普通窄线的计时或确认帧。 */
    }
    if (p->state == ROUTE_EXECUTING) {
        float turn = (heading_rad - p->entry_heading) * (action == LEFT ? -1.0f : 1.0f);
        /* 反向保护从实际动作起点计算；不能将继承的残余偏航误当作本动作倒转。 */
        float actual_turn = (heading_rad - p->action_start_heading) * (action == LEFT ? -1.0f : 1.0f);
        if (action != STRAIGHT && (turn > route_turn_threshold_rad(p, JUNCTION_TURN_MAX_RAD) ||
                                  actual_turn < -JUNCTION_WRONG_WAY_RAD)) {
            fail(p, actual_turn < -JUNCTION_WRONG_WAY_RAD ? ROUTE_FAILURE_WRONG_WAY : ROUTE_FAILURE_TURN_LIMIT);
            return;
        }
        if (route_prepares_straight(p)) {
            /* 接直行也必须实际检测到出口中心线，不能只凭航向对齐离口。 */
            if (timed_out) { fail(p, ROUTE_FAILURE_ACTION_TIMEOUT); return; }
            bool aligned = fabsf(heading_rad - exit_heading) <= STRAIGHT_ENTRY_ALIGN_RAD;
            bool exit_seen=route_is_narrow_line(obs) && fabsf(obs.position)<=JUNCTION_CENTER_LIMIT;
            if (!aligned || !exit_seen || (uint32_t)(now_ms - p->online_ms) > CONTROL_MAX_INTERVAL_MS)
                p->exit_count = 0;
            if (aligned && exit_seen && p->exit_count < LINE_CONFIRM_SAMPLES) ++p->exit_count;
            p->online_ms = now_ms;
            p->turn_capturing = p->turn_exit_branch = p->exit_candidate_pending = false;
            if (p->exit_count >= LINE_CONFIRM_SAMPLES) {
                p->state = ROUTE_CLEARING;
                p->phase_ms = now_ms;
                p->clear_cm = distance_cm;
                p->exit_count = 0;
            }
            return;
        }
        /* 满 1 s 后以连续完整单线结束转弯；超时在线也转入离口。
         * 不借用附带分支选区，不要求旧转角下限；错向/过转检查仍优先。
         * 相邻采样间隔过长或无线/歧义会清空连续计数。 */
        bool ordinary_turn = !route_uses_corner_arc(p) && s.action != CUSTOM_TURN &&
                             action != STRAIGHT;
        if(ordinary_turn && p->capture_line_seen &&
           distance_cm-p->capture_start_cm>JUNCTION_CLEAR_MAX_CM) {
            fail(p,ROUTE_FAILURE_EXIT_RECAPTURE);
            return;
        }
        /* 普通非圆弧直角必须实际达到估计目标角；在线/超时不能绕过角度门禁。
         * 角度不足的样本也不能提前积累时间退出的连续帧。 */
        bool target_angle_reached = !ordinary_turn || turn >= fabsf(route_turn_target_rad(p));
        bool online = route_is_narrow_line(obs) && target_angle_reached &&
            (!ordinary_turn || fabsf(obs.position) <= JUNCTION_CENTER_LIMIT);
        if (!online || (uint32_t)(now_ms - p->online_ms) > CONTROL_MAX_INTERVAL_MS)
            p->online_samples = 0;
        if (online && p->online_samples < LINE_CONFIRM_SAMPLES) ++p->online_samples;
        p->online_ms = now_ms;
        bool line_exit = action != STRAIGHT && online &&
            ((timed_out && (!ordinary_turn || p->online_samples >= LINE_CONFIRM_SAMPLES)) ||
             ((uint32_t)(now_ms - p->phase_ms) >= JUNCTION_LINE_EXIT_AFTER_MS &&
                           p->online_samples >= LINE_CONFIRM_SAMPLES));
        if (line_exit) {
            if (ordinary_turn && !p->turn_exit_branch) {
                p->line_exit_confirmed=true;
                p->line_exit_heading_rad=heading_rad;
            }
            p->state = ROUTE_CLEARING;
            p->phase_ms = now_ms;
            p->clear_cm = distance_cm;
            p->exit_count = 0;
            p->exit_candidate_pending = false;
            p->turn_capturing = p->turn_exit_branch = false;
            p->turn_ended_on_line = true;
            return;
        }
        if (timed_out) { fail(p, ROUTE_FAILURE_ACTION_TIMEOUT); return; }
        if (s.action == STRAIGHT) {
            /* 不等中心线：通过标称点后进入按距离去重，避免一直直行吞掉后续转弯。 */
            if (p->outgoing_cm >= 0.0f) {
                p->state = ROUTE_CLEARING;
                p->phase_ms = now_ms;
                p->clear_cm = distance_cm - p->outgoing_cm;
                p->exit_count = 0;
            }
            return;
        }
        /* 侧边窄线先交给位置反馈捕获；全目标/普通多区域不能进入位置 PID；有界附带分支只用航向控制。
         * 最终退出仍要求足够转角和连续中心线，不能用提前遇到的图形直接退出。 */
        p->turn_capturing = action != STRAIGHT && turn >= route_turn_threshold_rad(p, JUNCTION_CAPTURE_MIN_RAD) &&
            route_is_narrow_line(exit_obs);
        if (p->turn_capturing && !p->turn_exit_branch) {
            if(!p->capture_line_seen) p->capture_start_cm=distance_cm;
            p->capture_line_seen=true;
            p->capture_line_ms=now_ms;
        }
        bool departed = action == STRAIGHT ? distance_cm - p->entry_cm >= JUNCTION_CLEAR_DISTANCE_CM :
            turn >= route_turn_threshold_rad(p, JUNCTION_TURN_MIN_RAD);
        bool centered = route_is_narrow_line(exit_obs) && fabsf(exit_obs.position) <= JUNCTION_CENTER_LIMIT;
        /* 捕获期间允许窄出口线横移；仅最终退出要求居中。
         * 若计时也要求居中，位置 2/3 来回变化会反复清零，直至动作超时。 */
        /* 成熟出口证据建立后，位置 PID 回线可能使角度略微回落。
         * 仅保持成熟证据时使用小回差，避免清零后重新强制同向转向；
         * 未积累证据、无线/歧义和超出回差时仍走原始角度门槛。
         * 附带分支模式不放宽，保持其独立形状与角度约束。 */
        bool exit_evidence_ready = p->exit_candidate_pending &&
            (uint32_t)(now_ms - p->exit_candidate_ms) >= JUNCTION_EXIT_CONFIRM_MS &&
            distance_cm - p->exit_candidate_cm >= JUNCTION_EXIT_CONFIRM_CM;
        float exit_margin = fminf(JUNCTION_EXIT_ANGLE_MARGIN_RAD,
                                  fabsf(route_turn_target_rad(p)) * 0.1f);
        if (exit_evidence_ready && !p->turn_exit_branch)
            exit_margin += fminf(JUNCTION_EXIT_HYSTERESIS_RAD,
                                fabsf(route_turn_target_rad(p)) * 0.05f);
        float capture_threshold = ordinary_turn && !p->turn_exit_branch ?
            route_turn_threshold_rad(p,JUNCTION_CAPTURE_MIN_RAD) :
            route_turn_threshold_rad(p,JUNCTION_TURN_MIN_RAD)-exit_margin;
        bool near_exit = action != STRAIGHT && route_is_narrow_line(exit_obs) &&
            turn >= capture_threshold;
        if (!near_exit) p->exit_candidate_pending = false;
        else if (!p->exit_candidate_pending) {
            p->exit_candidate_pending = true;
            p->exit_candidate_ms = now_ms;
            p->exit_candidate_cm = distance_cm;
        }
        /* 实测转角差约 0.003 rad 时，1.6 cm/s 目标轮差未形成足够实际轮差。
         * 仅在接近角度门槛且连续沿窄出口线前进后，接受有界估计误差。 */
        if (near_exit && (uint32_t)(now_ms - p->exit_candidate_ms) >= JUNCTION_EXIT_CONFIRM_MS &&
            distance_cm - p->exit_candidate_cm >= JUNCTION_EXIT_CONFIRM_CM) departed = true;
        if (p->turn_exit_branch) {
            departed = p->exit_candidate_pending &&
                (uint32_t)(now_ms-p->exit_candidate_ms)>=JUNCTION_BRANCH_EXIT_CONFIRM_MS &&
                distance_cm-p->exit_candidate_cm>=JUNCTION_BRANCH_EXIT_CONFIRM_CM;
        }
        bool line_priority_ready=ordinary_turn && !p->turn_exit_branch && near_exit &&
            (uint32_t)(now_ms-p->exit_candidate_ms)>=JUNCTION_EXIT_CONFIRM_MS &&
            distance_cm-p->exit_candidate_cm>=JUNCTION_EXIT_CONFIRM_CM;
        departed = departed && (target_angle_reached || line_priority_ready);
        p->exit_count = departed && centered ? (uint8_t)(p->exit_count + 1U) : 0U;
        if (p->exit_count >= LINE_CONFIRM_SAMPLES) {
            p->state = ROUTE_CLEARING;
            p->phase_ms = now_ms; /* 已确认出口后，离口使用独立且有界的时间预算。 */
            p->turn_capturing = false;
            p->clear_cm = distance_cm;
            p->exit_count = 0;
            if (ordinary_turn && !p->turn_exit_branch) {
                p->line_exit_confirmed=true;
                p->line_exit_heading_rad=heading_rad;
            }
        }
    } else if (p->state == ROUTE_CLEARING) {
        bool ordinary_turn = !route_uses_corner_arc(p) && s.action != CUSTOM_TURN &&
                             action != STRAIGHT;
        float directed_turn=(heading_rad-p->entry_heading)*(action==LEFT?-1.0f:1.0f);
        bool heading_ready=!ordinary_turn || (p->line_exit_confirmed ?
            directed_turn>=route_turn_threshold_rad(p,JUNCTION_CAPTURE_MIN_RAD) :
            directed_turn>=fabsf(route_turn_target_rad(p)));
        if (!route_is_narrow_line(exit_obs) ||
            (ordinary_turn && (!heading_ready || fabsf(exit_obs.position)>JUNCTION_CENTER_LIMIT)) ||
            (action == STRAIGHT && fabsf(exit_obs.position) > JUNCTION_CENTER_LIMIT)) p->exit_count = 0;
        else if (p->exit_count < LINE_CONFIRM_SAMPLES) ++p->exit_count;
        size_t next_index = p->lap_boundary ? 0U : p->index + 1U;
        if (next_index >= p->count) { fail(p, ROUTE_FAILURE_INVALID); return; }
        float clearance = distance_cm - p->clear_cm;
        if(action!=STRAIGHT && p->outgoing_cm>p->steps[next_index].distance+p->steps[next_index].tolerance) {
            fail(p,ROUTE_FAILURE_WINDOW);
            return; /* 不把已越过的下一路口交接成合法新路段。 */
        }
        bool stable_clear=true;
        if(ordinary_turn && !p->turn_exit_branch) {
            if(clearance>JUNCTION_CLEAR_MAX_CM) { fail(p,ROUTE_FAILURE_EXIT_RECAPTURE); return; }
            bool center=heading_ready && route_is_narrow_line(exit_obs) &&
                fabsf(exit_obs.position)<=JUNCTION_CENTER_LIMIT;
            if(!center || (uint32_t)(now_ms-p->clear_sample_ms)>CONTROL_MAX_INTERVAL_MS)
                p->clear_center_pending=false;
            if(center && !p->clear_center_pending) {
                p->clear_center_pending=true;
                p->clear_center_ms=now_ms;
                p->clear_center_cm=distance_cm;
            }
            p->clear_sample_ms=now_ms;
            stable_clear=p->clear_center_pending &&
                (uint32_t)(now_ms-p->clear_center_ms)>=JUNCTION_CLEAR_CENTER_MS &&
                distance_cm-p->clear_center_cm>=JUNCTION_CLEAR_CENTER_CM;
        }
        bool normal_clear = p->exit_count >= LINE_CONFIRM_SAMPLES &&
                            clearance >= JUNCTION_CLEAR_DISTANCE_CM && stable_clear;
        if (s.action == STRAIGHT) {
            /* 下一窗口比常规 3 cm 去重更近时，在该窗口起点交接，不能压过下一路口。 */
            float straight_clear = fminf(JUNCTION_CLEAR_DISTANCE_CM,
                p->steps[next_index].distance - p->steps[next_index].tolerance);
            normal_clear = p->outgoing_cm >= straight_clear;
        }
        if (route_prepares_straight(p))
            normal_clear = clearance >= JUNCTION_CLEAR_DISTANCE_CM;
        /* 下一项临近时不继续固定 3 cm 离口，避免切段后窗口已耗尽。
         * 这里不放大/平移窗口、不清零继承里程；直行动作仍使用完整离口距离。
         * 转角与出口确认已在 EXECUTING 完成，再要求 0.5 cm 和新的连续窄线证据，允许出口线横移。 */
        bool early_clear = fabsf(route_turn_target_rad(p)) >= JUNCTION_CAPTURE_MIN_RAD &&
            p->outgoing_cm >= p->steps[next_index].distance - JUNCTION_APPROACH_DISTANCE_CM &&
            p->outgoing_cm <= p->steps[next_index].distance + p->steps[next_index].tolerance &&
            clearance >= JUNCTION_SHORT_CLEAR_CM &&
            (route_prepares_straight(p) || p->exit_count >= LINE_CONFIRM_SAMPLES) && stable_clear;
        if (!normal_clear && !early_clear) return;
        /* Debounce the physical junction before consuming another table entry. */
        if (p->lap_boundary) {
            p->index = 0;
            p->lap_boundary = false;
        } else ++p->index;
        p->turn_capturing = false;
        if (p->index >= p->count) { fail(p, ROUTE_FAILURE_INVALID); return; }
        bool short_segment = p->steps[p->index].distance <
            CORNER_RADIUS_CM + GRAY_FORWARD_OFFSET_CM + JUNCTION_SHORT_CLEAR_CM;
        begin_segment(p, distance_cm - p->outgoing_cm, now_ms);
        p->segment_heading_valid = short_segment;
        p->segment_heading_rad = exit_heading;
        p->segment_start_progress_cm = p->outgoing_cm;
        p->handoff_early = early_clear && !normal_clear;
        if (p->handoff_early) {
            /* 已完成出口中心线确认及连续窄线/距离去重，保留入口证据供紧邻下一弯使用。
             * 时间戳取当前帧，后续仍受 CONTROL_MAX_INTERVAL_MS 的相邻帧门禁。 */
            remember_line(p, exit_obs, now_ms);
            p->narrow_count = LINE_CONFIRM_SAMPLES;
            if (fabsf(exit_obs.position) <= JUNCTION_CENTER_LIMIT) {
                p->approach_center_count = LINE_CONFIRM_SAMPLES;
                p->have_approach_center = true;
                p->approach_center_cm = distance_cm;
                p->approach_center_ms = now_ms;
            }
        }
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
