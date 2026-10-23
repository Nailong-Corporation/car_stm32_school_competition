#include "line_follow.h"
#include "control_config.h"
#include "motion_config.h"
#include <math.h>
#include <string.h>

/**
 * @brief 判断停止原因是否属于必须复位才能解除的任务或编码器故障。
 */
static bool locked(LineStopReason reason)
{
    return reason == LINE_STOP_FAULT || reason == LINE_STOP_ENCODER;
}

/* 只记录请求结果，不排队、不放宽连续白线与故障门禁。 */
static bool start_allowed(LineFollow *f, bool stable)
{
    if (locked(f->stop_reason)) {
        f->start_result = LINE_START_REJECTED_FAULT;
        return false;
    }
    if (!stable) {
        f->start_result = LINE_START_REJECTED_LINE;
        return false;
    }
    return true;
}

/**
 * @brief 初始化巡线状态及当前编译模式的控制器，上电保持 IDLE。
 * 编码器模式绑定标定和双环；仅开启预设路径时绑定路径表和停止回调。
 */
void line_follow_init(LineFollow *f, OdometryCalibration calibration,
                       const RouteStep *steps, size_t count,
                       RouteStopCallback callback, void *context)
{
    memset(f, 0, sizeof(*f));
    f->state = LINE_IDLE;
#if PRESET_ROUTE_ENABLED
    route_init(&f->route, steps, count, callback, context);
#else
    (void)steps;
    (void)count;
    (void)callback;
    (void)context;
    f->route.state = ROUTE_DISABLED;
#endif
#if ENCODER_CONTROL_ENABLED
    odometry_init(&f->odometry, calibration);
    speed_control_init(&f->speed);
    pid_init(&f->pid, (PidConfig){POSITION_KP_CM_S, POSITION_KI_CM_S, POSITION_KD_CM_S,
                                  PID_INTEGRAL_LIMIT, PID_DERIVATIVE_TAU_S});
#else
    (void)calibration;
    pid_init(&f->pid, (PidConfig){OPEN_LOOP_POSITION_KP, OPEN_LOOP_POSITION_KI,
                                  OPEN_LOOP_POSITION_KD, PID_INTEGRAL_LIMIT, PID_DERIVATIVE_TAU_S});
#endif
}

/**
 * @brief 清空控制输出与 PID 历史，进入 STOPPED，并保留已锁定的故障原因。
 * 本函数仅更新软件状态，实际电机停车由调用者下发；非路径完成会取消待分派回调。
 */
void line_follow_stop(LineFollow *f, LineStopReason reason)
{
    if (!f->stop_snapshot.valid) {
        f->stop_snapshot = (LineStopSnapshot){
            .valid = true, .reason = reason, .state_before_stop = f->state,
            .route_state = f->route.state, .route_state_before_update = f->route_state_before_update,
            .route_failure = f->route.failure, .speed_fault = f->speed.fault,
            .speed_fault_wheel = f->speed.fault_wheel, .observation = f->observation,
            .counts = f->sample_counts,
            .sample_ms = f->sample_ms,
            .reliable_age_ms = (uint32_t)(f->sample_ms - f->last_reliable_ms),
            .route_phase_age_ms = (uint32_t)(f->sample_ms - f->route.phase_ms),
            .route_index = (uint32_t)f->route.index, .dt_s = f->sample_dt_s,
            .base = f->base, .left_target_cm_s = f->left_target_cm_s,
            .right_target_cm_s = f->right_target_cm_s,
            .left_speed_cm_s = f->odometry.left_speed_cm_s,
            .right_speed_cm_s = f->odometry.right_speed_cm_s,
            .distance_cm = f->odometry.distance_cm,
            .route_progress_cm = f->odometry.distance_cm - f->route.origin_cm,
            .clear_progress_cm = f->odometry.distance_cm - f->route.clear_cm,
            .turn_rad = f->odometry.heading_rad - f->route.entry_heading,
            .exit_candidate_pending = f->route.exit_candidate_pending,
            .exit_candidate_age_ms = f->route.exit_candidate_pending ?
                (uint32_t)(f->sample_ms - f->route.exit_candidate_ms) : 0U,
            .exit_candidate_progress_cm = f->route.exit_candidate_pending ?
                f->odometry.distance_cm - f->route.exit_candidate_cm : 0.0f,
            .exit_confirm_samples = f->route.exit_count
        };
    }
    f->state = LINE_STOPPED;
    if (!locked(f->stop_reason)) f->stop_reason = reason;
    f->command = (MotorCommand){0};
    f->left_target_cm_s = f->right_target_cm_s = f->base = f->correction = 0.0f;
    f->held_pwm_correction = 0.0f;
    f->have_reliable = f->abnormal = false;
    f->valid_count = 0;
    pid_reset(&f->pid);
#if ENCODER_CONTROL_ENABLED
    speed_control_reset(&f->speed);
#endif
    if (reason != LINE_STOP_ROUTE_COMPLETE) f->route.stop_pending = false;
}

/**
 * @brief 判断最近稳定白线位置是否仍处于有效时间窗口内。
 * now_ms 使用毫秒时基；返回值供传感器连续性选择参考，不会刷新可靠观测时间。
 */
bool line_follow_have_recent_position(const LineFollow *f, uint32_t now_ms)
{
    return f->have_reliable && (uint32_t)(now_ms - f->last_reliable_ms) < INTERFERENCE_TIMEOUT_MS;
}

/* 根据实际 dt 规划下一周期基础值；启动包络不阻塞采样，降速及停止不等待。 */
static float next_base(const LineFollow *f, float base, float rise, float dt)
{
    float fraction = fminf(1.0f, (f->startup_elapsed_s + dt) / STARTUP_RAMP_TIME_S);
    return fminf(base * fraction, f->base + rise * dt);
}

#if ENCODER_CONTROL_ENABLED
/**
 * @brief 在编码器模式混合左右目标速度，base/correction 单位均为 cm/s。
 * 按实际 dt 限制基础速度上升，并限制差值以保证两轮目标非负且不超上限。
 */
static void targets(LineFollow *f, float base, float correction, float dt)
{
    f->base = next_base(f, base, TARGET_RISE_CM_S2, dt);
    f->startup_elapsed_s = fminf(STARTUP_RAMP_TIME_S, f->startup_elapsed_s + dt);
    float limit = fminf(f->base, WHEEL_TARGET_MAX_CM_S - f->base);
    f->correction = fmaxf(-limit, fminf(limit, correction));
    f->left_target_cm_s = f->base + f->correction;
    f->right_target_cm_s = f->base - f->correction;
}
#else
/**
 * @brief 在普通 PWM 模式限制基础指令上升速率并混合左右轮输出。
 * base/correction 为 PWM 指令值；保存限幅后的实际半差，供干扰保持使用。
 */
static void pwm_output(LineFollow *f, float base, float correction, float dt)
{
    f->base = next_base(f, base, OPEN_LOOP_PWM_RISE_PER_S, dt);
    f->startup_elapsed_s = fminf(STARTUP_RAMP_TIME_S, f->startup_elapsed_s + dt);
    /* 满速基准保留转向余量：外轮维持基准，内轮降低两倍半差。
     * 交给 motor_mix 的均值随半差降低，避免基准 255 时差速被钳为零。 */
    float limit = f->state == LINE_TRACKING ?
        fminf(0.5f * f->base, OPEN_LOOP_CORRECTION_MAX_PWM) :
        fminf(f->base, MOTOR_PWM_LIMIT - f->base);
    correction = fmaxf(-limit, fminf(limit, correction));
    if (f->state == LINE_TRACKING)
        f->command = motor_mix(f->base - fabsf(correction), correction);
    else
        f->command = motor_mix(f->base, correction);
    f->correction = 0.5f * (f->command.left - f->command.right);
}

/**
 * @brief 执行无编码器的巡线状态机，由白线位置 PID 直接生成左右 PWM。
 * 启动和重获需连续单一区域目标线（最多 7 路）；干扰时冻结 PID、衰减历史差值，超时停止。
 * dt 单位为秒，now_ms 单位为毫秒；不计算轮速、厘米里程或真实航向。
 */
static void open_loop_update(LineFollow *f, GrayObservation obs, uint32_t now_ms,
                              float dt, bool start_request)
{
    bool narrow = route_is_narrow_line(obs);
    if (!narrow) f->valid_count = 0;
    else if (f->valid_count < LINE_CONFIRM_SAMPLES) ++f->valid_count;
    bool stable = f->valid_count >= LINE_CONFIRM_SAMPLES;
    if (f->state == LINE_IDLE || f->state == LINE_STOPPED) {
        if (!start_request || !start_allowed(f, stable)) return;
        f->start_result = LINE_START_ACCEPTED;
        f->state = LINE_TRACKING;
        f->stop_reason = LINE_STOP_NONE;
        f->stop_snapshot = (LineStopSnapshot){0};
        f->base = 0.0f;
        f->startup_elapsed_s = 0.0f;
        pid_reset(&f->pid);
    }
    if (LOST_LINE_STOP_ENABLED && f->abnormal &&
        (uint32_t)(now_ms - f->last_reliable_ms) >= INTERFERENCE_TIMEOUT_MS) {
        line_follow_stop(f, LINE_STOP_LOST);
        return;
    }
    if (narrow && (!f->abnormal || stable)) {
        if (f->abnormal) pid_reset(&f->pid);
        f->abnormal = false;
        f->state = LINE_TRACKING;
        if (stable) {
            f->have_reliable = true;
            f->reliable_position = obs.position;
            f->last_reliable_ms = now_ms;
        }
        float base = OPEN_LOOP_BASE_PWM;
        float next = next_base(f, base, OPEN_LOOP_PWM_RISE_PER_S, dt);
        float correction;
        if (!pid_update(&f->pid, obs.position, dt,
                        fminf(0.5f * next, OPEN_LOOP_CORRECTION_MAX_PWM), &correction)) {
            line_follow_stop(f, LINE_STOP_FAULT);
            return;
        }
        pwm_output(f, base, correction, dt);
    } else {
        if (!f->abnormal) {
            f->abnormal = true;
            f->held_pwm_correction = f->correction;
        }
        f->state = LINE_GAP_HOLD;
        float decay = fmaxf(0.0f, 1.0f -
            (float)(uint32_t)(now_ms - f->last_reliable_ms) / (float)INTERFERENCE_TIMEOUT_MS);
        /* No encoder: bounded historical PWM difference only, not heading feedback. */
        pwm_output(f, fminf(OPEN_LOOP_HOLD_PWM, f->base), f->held_pwm_correction * decay, dt);
    }
}
#endif

/**
 * @brief 执行一个实际 dt 控制周期，更新巡线状态及最终 PWM 命令。
 * 编码器模式用 counts 更新里程和速度内环，仅开启路径时推进路口；PWM 模式忽略 counts。
 * 非法观测或时序进入故障停车；启动请求为一次性，超时判断先于恢复确认。
 */
void line_follow_update(LineFollow *f, GrayObservation obs, EncoderCounts counts,
                        uint32_t now_ms, float dt, bool start_request)
{
    f->sample_ms = now_ms;
    f->sample_dt_s = dt;
    f->observation = obs;
    f->sample_counts = counts;
    f->route_state_before_update = f->route.state;
    if (start_request) f->start_result = LINE_START_ALREADY_RUNNING;
    if (!isfinite(dt) || dt <= 0.0f || dt > (float)CONTROL_MAX_INTERVAL_MS / 1000.0f ||
        (obs.kind != GRAY_VALID && obs.kind != GRAY_NONE && obs.kind != GRAY_AMBIGUOUS) ||
        (obs.kind == GRAY_VALID && (!isfinite(obs.position) || fabsf(obs.position) > 7.0f))) {
        line_follow_stop(f, LINE_STOP_FAULT);
        if (start_request) f->start_result = LINE_START_REJECTED_FAULT;
        return;
    }
#if !ENCODER_CONTROL_ENABLED
    (void)counts;
    open_loop_update(f, obs, now_ms, dt, start_request);
#else
    if (!f->odometry.ready) {
        if (start_request) {
            line_follow_stop(f, LINE_STOP_CALIBRATION);
            f->start_result = LINE_START_REJECTED_CALIBRATION;
        }
        return;
    }
    if (!odometry_update(&f->odometry, counts, dt)) {
        line_follow_stop(f, LINE_STOP_ENCODER);
        if (start_request) f->start_result = LINE_START_REJECTED_FAULT;
        return;
    }
    bool narrow = route_is_narrow_line(obs);
    if (narrow) {
        if (f->valid_count < LINE_CONFIRM_SAMPLES) ++f->valid_count;
    } else f->valid_count = 0;
    bool stable = f->valid_count >= LINE_CONFIRM_SAMPLES;
    if (f->state == LINE_IDLE || f->state == LINE_STOPPED) {
        if (!start_request || !start_allowed(f, stable)) return;
#if PRESET_ROUTE_ENABLED
        if (!f->route.valid) {
            line_follow_stop(f, LINE_STOP_ROUTE);
            f->start_result = LINE_START_REJECTED_ROUTE;
            return;
        }
        route_start(&f->route, f->odometry.distance_cm, now_ms);
#endif
        f->start_result = LINE_START_ACCEPTED;
        f->state = LINE_TRACKING;
        f->stop_reason = LINE_STOP_NONE;
        f->stop_snapshot = (LineStopSnapshot){0};
        f->last_reliable_ms = now_ms;
        f->have_reliable = true;
        f->base = 0.0f;
        f->startup_elapsed_s = 0.0f;
        pid_reset(&f->pid);
        speed_control_reset(&f->speed);
    }
    /* Deadline precedes recovery/junction confirmation; isolated noise cannot extend it. */
    if (LOST_LINE_STOP_ENABLED && f->abnormal && ((uint32_t)(now_ms - f->last_reliable_ms) >= INTERFERENCE_TIMEOUT_MS ||
        fabsf(f->odometry.distance_cm - f->abnormal_origin_cm) >= INTERFERENCE_MAX_DISTANCE_CM)) {
        line_follow_stop(f, LINE_STOP_LOST);
        return;
    }
#if PRESET_ROUTE_ENABLED
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
    RouteStep speed_step = step;
    float progress = f->odometry.distance_cm - f->route.origin_cm;
    if (f->route.state == ROUTE_CLEARING) {
        /* 转弯与中心出口线已确认，立即按下一段提速，不再受旧路口窗口的低速锁定。
         * 路径索引/里程起点仍等 3 cm 离口去重完成后再推进；循环边界预看第 0 项。
         * 下一段按路口参考点的出口方向位移计程，预看实际进度以约束短段。 */
        size_t next_index = f->route.lap_boundary ? 0U : f->route.index + 1U;
        if (next_index >= f->route.count) { line_follow_stop(f, LINE_STOP_ROUTE); return; }
        speed_step = f->route.steps[next_index];
        progress = f->route.outgoing_cm;
    }
    float base_speed = (float)speed_step.speed;
    /* 按表内路口位置前 5 cm 规划减速；容差窗口只决定是否允许确认路口。
     * 窗口内提前确认的路口仍立即进入候选低速或转弯动作。
     * v² 随剩余距离线性变化，对理想跟随目标的直线行驶相当于恒定减速度。
     * 这里只改变速度，不放宽 route_update 的距离/形状确认门禁。
     */
    float slow_speed = fminf(base_speed, JUNCTION_APPROACH_SPEED_CM_S);
    float remaining = speed_step.distance - progress;
    if (f->route.state == ROUTE_CANDIDATE || remaining <= 0.0f) {
        base_speed = slow_speed;
    } else if (remaining < JUNCTION_APPROACH_DISTANCE_CM) {
        float fraction = remaining / JUNCTION_APPROACH_DISTANCE_CM;
        base_speed = sqrtf(slow_speed * slow_speed +
                           (base_speed * base_speed - slow_speed * slow_speed) * fraction);
    }
#else
    float base_speed = LINE_FOLLOW_SPEED_CM_S;
#endif
#if PRESET_ROUTE_ENABLED
    if (f->route.state == ROUTE_EXECUTING) {
        if (previous_route != ROUTE_EXECUTING) {
            pid_reset(&f->pid);
            speed_control_reset(&f->speed);
        }
        f->abnormal = false;
        f->state = LINE_JUNCTION;
        float turn_base = fminf((float)step.speed, JUNCTION_TURN_SPEED_CM_S);
        float correction;
        Action action = route_current_action(&f->route);
        if (action == LEFT || action == RIGHT) {
            float direction = action == LEFT ? -1.0f : 1.0f;
            float turn = direction * (f->odometry.heading_rad - f->route.entry_heading);
            if (turn >= JUNCTION_TURN_SLOW_RAD)
                turn_base = fminf(turn_base, JUNCTION_TURN_SLOW_CM_S);
            if (f->route.turn_capturing) {
                /* 捕获侧边出口线，向 ±90° 航向和目标线共同收敛。
                 * 先前固定支点转到中心才接管，可能把侧边线扫出探头范围。 */
                turn_base = fminf((float)step.speed, JUNCTION_CAPTURE_SPEED_CM_S);
                float line_correction;
                if (!pid_update(&f->pid, obs.position, dt, turn_base, &line_correction)) {
                    line_follow_stop(f, LINE_STOP_FAULT);
                    return;
                }
                correction = line_correction + JUNCTION_CAPTURE_HEADING_KP *
                    (f->route.entry_heading + direction * JUNCTION_TURN_TARGET_RAD -
                     f->odometry.heading_rad);
                /* 实车快照：中心附近出口线使两项抵消，停在 1.3465 < 1.35 rad。
                 * 最小退出角之前保持期望方向的小转向；达到门槛后仍按出口线收敛。 */
                if (turn < JUNCTION_TURN_MIN_RAD) {
                    float min_turn = fminf(turn_base, JUNCTION_CAPTURE_MIN_TURN_CM_S);
                    correction = direction * fmaxf(direction * correction, min_turn);
                }
            } else {
                pid_reset(&f->pid);
                /* 无可靠出口线时继续有界非反转扫线，找线阶段降低外轮目标。 */
                correction = direction * turn_base;
            }
        } else {
            correction = HEADING_KP_CM_S_PER_RAD *
                         (f->route.entry_heading - f->odometry.heading_rad);
        }
        targets(f, turn_base, correction, dt);
    } else
#endif
    if (narrow && (stable || !f->abnormal)
#if PRESET_ROUTE_ENABLED
        && f->route.state != ROUTE_CANDIDATE
#endif
    ) {
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
        float base = base_speed * (1.0f - POSITION_CURVE_SLOWDOWN * fabsf(obs.position) / 7.0f);
        float next = next_base(f, base, TARGET_RISE_CM_S2, dt);
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
        targets(f, fminf(base_speed, HOLD_SPEED_CM_S),
                HEADING_KP_CM_S_PER_RAD * (f->hold_heading_rad - f->odometry.heading_rad), dt);
    }
    if (!speed_control_update(&f->speed, f->left_target_cm_s, f->right_target_cm_s,
                              f->odometry.left_speed_cm_s, f->odometry.right_speed_cm_s,
                              dt, &f->command)) line_follow_stop(f, LINE_STOP_ENCODER);
#endif
}
