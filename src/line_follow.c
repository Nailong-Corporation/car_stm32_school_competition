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
    return reason == LINE_STOP_FAULT || reason == LINE_STOP_ENCODER ||
           reason == LINE_STOP_IMU || reason == LINE_STOP_TILT;
}

void line_follow_acknowledge_tilt(LineFollow *f)
{
    if (f->state == LINE_STOPPED && f->stop_reason == LINE_STOP_TILT)
        f->stop_reason = LINE_STOP_NONE;
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
            .exit_recovery_active=f->exit_recovery_active,
            .exit_lateral_error_cm=f->exit_lateral_error_cm,
            .turn_exit_branch = f->route.turn_exit_branch,
            .exit_line_mask = f->route.exit_line_mask,
            .handoff_early = f->route.handoff_early,
            .segment_start_progress_cm = f->route.segment_start_progress_cm,
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
            .actual_turn_rad = f->odometry.heading_rad - f->route.action_start_heading,
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
    f->exit_accelerating = false;
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
/* 确认出口后快速提速，跨切段延续直到达到规划目标；START 保留原包络。 */
static float target_rise(const LineFollow *f)
{
#if PRESET_ROUTE_ENABLED && TURN_EXIT_ACCEL_ENABLED
    if (f->exit_accelerating || (f->route.state == ROUTE_CLEARING && route_current_action(&f->route) != STRAIGHT))
        return TURN_EXIT_ACCEL_CM_S2;
#else
    (void)f;
#endif
    return TARGET_RISE_CM_S2;
}

/**
 * @brief 在编码器模式混合左右目标速度，base/correction 单位均为 cm/s。
 * 按实际 dt 限制基础速度上升，并限制差值以保证两轮目标非负且不超上限。
 */
static void targets(LineFollow *f, float base, float correction, float dt)
{
    f->base = next_base(f, base, target_rise(f), dt);
    if (f->base >= base) f->exit_accelerating = false;
    f->startup_elapsed_s = fminf(STARTUP_RAMP_TIME_S, f->startup_elapsed_s + dt);
    float limit = fminf(f->base, WHEEL_TARGET_MAX_CM_S - f->base);
    f->correction = fmaxf(-limit, fminf(limit, correction));
    f->left_target_cm_s = f->base + f->correction;
    f->right_target_cm_s = f->base - f->correction;
}

/* 正常巡线与非圆弧转弯保持外轮基准，内轮减速，满速仍有PID修正余量。 */
static float next_outer(const LineFollow *f, float outer, float dt)
{
    float previous=fmaxf(f->left_target_cm_s,f->right_target_cm_s);
    float fraction=fminf(1.0f,(f->startup_elapsed_s+dt)/STARTUP_RAMP_TIME_S);
    return fminf(WHEEL_TARGET_MAX_CM_S,fminf(outer*fraction,previous+target_rise(f)*dt));
}
static void outer_targets(LineFollow *f, float outer, float correction, float dt)
{
    float requested_outer = outer;
    outer=next_outer(f,outer,dt);
    if (outer >= requested_outer) f->exit_accelerating = false;
    correction=fmaxf(-outer*.5f,fminf(outer*.5f,correction));
    float mean=outer-fabsf(correction);
    f->left_target_cm_s=mean+correction;
    f->right_target_cm_s=mean-correction;
    /* 外轮目标120而实测仅90时，内轮96也会饱和，两轮实际等速导致PID无效。
     * 保留外轮目标，以实测外轮×请求内外轮比限制内轮，零修正不触发。 */
    if(fabsf(correction)>=STEERING_FEEDBACK_MIN_HALF_CM_S && outer>0) {
        float measured=correction>0?f->odometry.left_speed_cm_s:f->odometry.right_speed_cm_s;
        float inner=correction>0?f->right_target_cm_s:f->left_target_cm_s;
        if(measured>=STEERING_FEEDBACK_MIN_CM_S && measured<outer) {
            inner=fminf(inner,measured*inner/outer);
            if(correction>0) f->right_target_cm_s=inner;
            else f->left_target_cm_s=inner;
        }
    }
    f->base=.5f*(f->left_target_cm_s+f->right_target_cm_s);
    f->correction=.5f*(f->left_target_cm_s-f->right_target_cm_s);
    f->startup_elapsed_s=fminf(STARTUP_RAMP_TIME_S,f->startup_elapsed_s+dt);
}

#if PRESET_ROUTE_ENABLED
/* 在已见过的中心线点上建立规划出口轴。失线后反馈横向偏移，不能只保持
 * 当时尚未沿出口摆正的车头航向；无真实中心线点时不伪造回线几何。 */
static float exit_recovery_correction(LineFollow *f, float outer)
{
    float dx=f->odometry.sensor_x_cm-f->exit_anchor_x_cm;
    float dy=f->odometry.sensor_y_cm-f->exit_anchor_y_cm;
    f->exit_lateral_error_cm=-dx*sinf(f->exit_axis_rad)+dy*cosf(f->exit_axis_rad);
    float bias=-atanf(f->exit_lateral_error_cm/JUNCTION_RECAPTURE_LOOKAHEAD_CM);
    bias=fmaxf(-JUNCTION_RECAPTURE_BIAS_RAD,fminf(JUNCTION_RECAPTURE_BIAS_RAD,bias));
    float error=f->exit_axis_rad+bias-f->odometry.heading_rad;
    float correction=f->odometry.calibration.track_cm*outer*sinf(error)/
        (ODOMETRY_HEADING_SCALE*JUNCTION_RECAPTURE_LOOKAHEAD_CM);
    float limit=outer*JUNCTION_RECAPTURE_MAX_HALF_RATIO;
    f->exit_recovery_active=true;
    return fmaxf(-limit,fminf(limit,correction));
}

/* STRAIGHT 按表速通过；满轮速时只钳外轮，仍允许内轮降速纠偏，避免航向半差被钳成零。 */
static void straight_targets(LineFollow *f, float base, float correction, float dt)
{
    targets(f, base, 0.0f, dt);
    f->left_target_cm_s = fmaxf(0.0f, fminf(WHEEL_TARGET_MAX_CM_S, f->base + correction));
    f->right_target_cm_s = fmaxf(0.0f, fminf(WHEEL_TARGET_MAX_CM_S, f->base - correction));
    f->correction = 0.5f * (f->left_target_cm_s - f->right_target_cm_s);
    f->base = 0.5f * (f->left_target_cm_s + f->right_target_cm_s);
}
#endif
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
    bool keep_base = f->state == LINE_TRACKING ||
        (f->state == LINE_GAP_HOLD && f->observation.kind != GRAY_NONE);
    float limit = keep_base ?
        fminf(0.5f * f->base, OPEN_LOOP_CORRECTION_MAX_PWM) :
        fminf(f->base, MOTOR_PWM_LIMIT - f->base);
    correction = fmaxf(-limit, fminf(limit, correction));
    if (keep_base)
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
        /* 歧义与恢复确认期沿用正常基准；完全无线仍按原上限降速。 */
        float base = obs.kind == GRAY_NONE ? fminf(OPEN_LOOP_HOLD_PWM, f->base) :
                                             OPEN_LOOP_BASE_PWM;
        pwm_output(f, base, f->held_pwm_correction * decay, dt);
    }
}
#endif

/**
 * @brief 执行一个实际 dt 控制周期，更新巡线状态及最终 PWM 命令。
 * 编码器模式用 counts 更新里程和速度内环，仅开启路径时推进路口；PWM 模式忽略 counts。
 * 非法观测或时序进入故障停车；启动请求为一次性，超时判断先于恢复确认。
 */
void line_follow_update_heading(LineFollow *f, GrayObservation obs, EncoderCounts counts,
                        uint32_t now_ms, float dt, bool start_request,
                        bool imu_valid, float imu_heading_rad)
{
#if !MPU6050_ENABLED
    (void)imu_valid;
    (void)imu_heading_rad;
#endif
#if ENCODER_CONTROL_ENABLED && PRESET_ROUTE_ENABLED
    bool was_waiting_after_straight = f->route.state == ROUTE_FOLLOW &&
        route_waiting_after_straight(&f->route, f->odometry.distance_cm);
#endif
#if ENCODER_CONTROL_ENABLED && PRESET_ROUTE_ENABLED && ROUTE_AUTO_RESUME_ENABLED
    uint32_t previous_sample_ms = f->sample_ms;
#endif
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
#if MPU6050_ENABLED
    if (!imu_valid || !isfinite(imu_heading_rad)) {
        line_follow_stop(f, LINE_STOP_IMU);
        if (start_request) f->start_result = LINE_START_REJECTED_FAULT;
        return;
    }
#endif
    if (!odometry_update_with_heading(&f->odometry, counts, dt,
#if MPU6050_ENABLED
                                      true, imu_heading_rad
#else
                                      false, 0.0f
#endif
                                      )) {
        line_follow_stop(f, LINE_STOP_ENCODER);
        if (start_request) f->start_result = LINE_START_REJECTED_FAULT;
        return;
    }
    bool narrow = route_is_narrow_line(obs);
#if PRESET_ROUTE_ENABLED && ROUTE_AUTO_RESUME_ENABLED
    if (f->state == LINE_STOPPED && f->stop_reason == LINE_STOP_ROUTE &&
        (uint32_t)(now_ms-previous_sample_ms) > CONTROL_MAX_INTERVAL_MS) f->valid_count = 0;
#endif
    if (narrow) {
        if (f->valid_count < LINE_CONFIRM_SAMPLES) ++f->valid_count;
    } else f->valid_count = 0;
    bool stable = f->valid_count >= LINE_CONFIRM_SAMPLES;
    if (f->state == LINE_IDLE || f->state == LINE_STOPPED) {
        bool resumed = false;
#if PRESET_ROUTE_ENABLED && ROUTE_AUTO_RESUME_ENABLED
        if (!start_request && stable && f->state == LINE_STOPPED &&
            f->stop_reason == LINE_STOP_ROUTE && f->stop_snapshot.valid) {
            resumed = route_resume(&f->route, f->stop_snapshot.route_state_before_update,
                                   f->odometry.distance_cm, f->odometry.heading_rad, now_ms);
            if (resumed) {
                f->auto_resume_snapshot = f->stop_snapshot;
                if (f->auto_resume_count < UINT32_MAX) ++f->auto_resume_count;
                f->auto_resume_last_ms = now_ms;
                f->start_result = LINE_START_AUTO_RESUMED;
                f->route_state_before_update = f->route.state;
            }
        }
#endif
        if (!resumed) {
            if (!start_request || !start_allowed(f, stable)) return;
#if PRESET_ROUTE_ENABLED
            if (!f->route.valid) {
                line_follow_stop(f, LINE_STOP_ROUTE);
                f->start_result = LINE_START_REJECTED_ROUTE;
                return;
            }
            route_start(&f->route, f->odometry.distance_cm, now_ms);
#endif
            f->auto_resume_count = f->auto_resume_last_ms = 0;
            f->auto_resume_snapshot = (LineStopSnapshot){0};
            f->straight_entry_aligned = f->straight_entry_handoff = (StraightEntrySnapshot){0};
            memset(f->turn_exit,0,sizeof(f->turn_exit));
            memset(&f->turn_trace,0,sizeof(f->turn_trace));
            memset(&f->entry_trace,0,sizeof(f->entry_trace));
            f->exit_anchor_valid=f->exit_recovery_active=false;
            f->exit_lateral_error_cm=0;
            f->start_result = LINE_START_ACCEPTED;
        }
        f->state = LINE_TRACKING;
        f->stop_reason = LINE_STOP_NONE;
        f->stop_snapshot = (LineStopSnapshot){0};
        f->last_reliable_ms = now_ms;
        f->have_reliable = true;
        f->abnormal = false;
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
    size_t previous_index = f->route.index;
    bool previous_prepares_straight = route_prepares_straight(&f->route);
    float previous_turn=(f->odometry.heading_rad-f->route.entry_heading)*
        (route_current_action(&f->route)==LEFT?-1.0f:1.0f);
    if(previous_route==ROUTE_CLEARING && previous_index<2 && f->turn_exit[previous_index].valid)
        f->turn_exit[previous_index].minimum_turn_rad=fminf(
            f->turn_exit[previous_index].minimum_turn_rad,previous_turn);
    uint8_t narrow_before=f->route.narrow_count;
    float entry_progress=f->odometry.distance_cm-f->route.origin_cm;
    route_update(&f->route, obs, f->odometry.distance_cm, f->odometry.heading_rad, now_ms);
    /* 正常完成离口后开始记录下一路口；失败后的自动恢复不经过此交接，
     * 因此仍保留首次失败证据。否则第一个弯会永久占住整轮入口记录。 */
    if (previous_route==ROUTE_CLEARING && f->route.state==ROUTE_FOLLOW)
        memset(&f->entry_trace,0,sizeof(f->entry_trace));
    if (!f->entry_trace.frozen &&
        (previous_route==ROUTE_FOLLOW || previous_route==ROUTE_CANDIDATE)) {
        EntryTrace *trace=&f->entry_trace;
        RouteStep entry_step=f->route.steps[previous_index];
        float window_progress=f->route.recovery_window_active ?
            f->odometry.distance_cm-f->route.recovery_window_origin_cm : entry_progress;
        float low=f->route.recovery_window_active ? 0 : entry_step.distance-entry_step.tolerance;
        float high=f->route.recovery_window_active ?
            fmaxf(2*entry_step.tolerance,GRAY_FORWARD_OFFSET_CM) : entry_step.distance+entry_step.tolerance;
        EntryTraceSample sample={
            .ms=now_ms, .progress_cm=entry_progress,
            .left_speed=f->odometry.left_speed_cm_s, .right_speed=f->odometry.right_speed_cm_s,
            .prior_base=f->base, .prior_left_target=f->left_target_cm_s,
            .prior_right_target=f->right_target_cm_s,
            .prior_left_pwm=f->command.left, .prior_right_pwm=f->command.right,
            .mask=obs.line_mask, .prefilter_mask=obs.prefilter_mask,
            .seam_mask=obs.seam_mask, .kind=(uint8_t)obs.kind,
            .narrow_before=narrow_before, .state_before=(uint8_t)previous_route,
            .state_after=(uint8_t)f->route.state,
            .index=(uint8_t)(previous_index>UINT8_MAX ? UINT8_MAX : previous_index),
            .window_band=route_step_is_auto(entry_step) ? 3U :
                window_progress<low ? 0U : window_progress>high ? 2U : 1U
        };
        trace->samples[trace->next]=sample;
        trace->next=(uint8_t)((trace->next+1U)%ENTRY_TRACE_CAPACITY);
        if (trace->count<ENTRY_TRACE_CAPACITY) ++trace->count;
        /* 长时间同样的无线帧不能淹没早先的分支/丢线/窗口开放事件。 */
        EntryTraceSample *last=&trace->events[(trace->event_next+ENTRY_EVENT_CAPACITY-1U)%ENTRY_EVENT_CAPACITY];
        if (trace->event_count==0 || sample.mask!=last->mask ||
            sample.prefilter_mask!=last->prefilter_mask || sample.seam_mask!=last->seam_mask ||
            sample.kind!=last->kind || sample.state_before!=last->state_before ||
            sample.state_after!=last->state_after || sample.index!=last->index ||
            sample.window_band!=last->window_band) {
            trace->events[trace->event_next]=sample;
            trace->event_next=(uint8_t)((trace->event_next+1U)%ENTRY_EVENT_CAPACITY);
            if (trace->event_count<ENTRY_EVENT_CAPACITY) ++trace->event_count;
        }
    }
    if (f->route.state==ROUTE_FAILED ||
        (previous_route!=ROUTE_EXECUTING && f->route.state==ROUTE_EXECUTING))
        f->entry_trace.frozen=true;
    if(previous_route!=ROUTE_EXECUTING && f->route.state==ROUTE_EXECUTING) {
        f->exit_anchor_valid=false;
        f->exit_lateral_error_cm=0;
    }
    if(!f->exit_anchor_valid && (f->route.state==ROUTE_EXECUTING || f->route.state==ROUTE_CLEARING) &&
       !route_uses_corner_arc(&f->route) && f->route.steps[f->route.index].action!=CUSTOM_TURN &&
       f->route.capture_line_seen && narrow && fabsf(obs.position)<=JUNCTION_CENTER_LIMIT) {
        f->exit_anchor_valid=true;
        f->exit_anchor_x_cm=f->odometry.sensor_x_cm;
        f->exit_anchor_y_cm=f->odometry.sensor_y_cm;
        f->exit_axis_rad=f->route.entry_heading+route_turn_target_rad(&f->route);
    }
    if(previous_route!=ROUTE_EXECUTING && f->route.state==ROUTE_EXECUTING && f->route.index<2)
        f->turn_exit[f->route.index]=(TurnExitDiagnostics){0};
    if(previous_index<2 && previous_route==ROUTE_EXECUTING && f->route.state==ROUTE_CLEARING) {
        TurnExitDiagnostics *record=&f->turn_exit[previous_index];
        record->valid=true;
        record->clearing_ms=now_ms;
        record->clearing_turn_rad=record->minimum_turn_rad=previous_turn;
        record->clearing_mask=obs.line_mask;
    }
    if(previous_index<2 && previous_route==ROUTE_CLEARING && f->route.index!=previous_index &&
       f->turn_exit[previous_index].valid) {
        f->turn_exit[previous_index].handed_off=true;
        f->turn_exit[previous_index].handoff_ms=now_ms;
        f->turn_exit[previous_index].handoff_turn_rad=previous_turn;
        f->turn_exit[previous_index].handoff_mask=obs.line_mask;
    }
    if (f->route.state == ROUTE_FAILED) { line_follow_stop(f, LINE_STOP_ROUTE); return; }
    if (f->route.state == ROUTE_FINISHED) { line_follow_stop(f, LINE_STOP_ROUTE_COMPLETE); return; }
    f->exit_recovery_active=false;
    if (was_waiting_after_straight &&
        !route_waiting_after_straight(&f->route, f->odometry.distance_cm)) {
        /* 有意保持航向的无中心线行驶结束，普通丢线预算从此刻起算。
         * 不伪造稳定线证据，也不借用保护期间的 PID/连续样本历史。 */
        f->last_reliable_ms = now_ms;
        f->have_reliable = false;
        f->valid_count = 0;
        stable = false;
        f->abnormal = false;
        pid_reset(&f->pid);
    }
    if (previous_route == ROUTE_EXECUTING && f->route.state == ROUTE_CLEARING) {
        f->exit_accelerating = TURN_EXIT_ACCEL_ENABLED && route_current_action(&f->route) != STRAIGHT;
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
    if (TURN_EXIT_ACCEL_ENABLED && f->route.state == ROUTE_CLEARING && step.action != STRAIGHT) {
        /* 转弯与中心出口线已确认，立即按下一段提速，不再受旧路口窗口的低速锁定。
         * 路径索引/里程起点在离口去重后推进；紧邻下一路口可用 0.5 cm + 连续窄线早交接。
         * 下一段按路口参考点的出口方向位移计程，预看实际进度以约束短段。 */
        size_t next_index = f->route.lap_boundary ? 0U : f->route.index + 1U;
        if (next_index >= f->route.count) { line_follow_stop(f, LINE_STOP_ROUTE); return; }
        speed_step = f->route.steps[next_index];
        progress = f->route.outgoing_cm;
    }
    float base_speed = (float)speed_step.speed;
    /* 在最早允许提交路口动作的位置前完成目标减速。
     * v² <= slow² + 2*a*remaining，至少保留5 cm减速区；高速段自动提前。
     * STOP最早确认点还受标称位置前5 cm门禁限制；STRAIGHT/AUTO不做距离减速。
     * 规划减速度不是已标定的机械制动能力，入口仍由route_update独立确认。
     */
    if (!route_step_is_auto(speed_step) && speed_step.action != STRAIGHT && f->route.recovery_window_active &&
        (f->route.state == ROUTE_FOLLOW || f->route.state == ROUTE_CANDIDATE))
        base_speed = fminf(base_speed, JUNCTION_APPROACH_SPEED_CM_S);
    bool approach_slow = !route_step_is_auto(speed_step) && TURN_APPROACH_SLOWDOWN_ENABLED &&
        (f->route.state == ROUTE_FOLLOW || f->route.state == ROUTE_CANDIDATE ||
         (TURN_EXIT_ACCEL_ENABLED && f->route.state == ROUTE_CLEARING && step.action != STRAIGHT));
    float slow_speed = fminf(base_speed, JUNCTION_APPROACH_SPEED_CM_S);
    float entry_lead = speed_step.action == STOP ?
        fminf(speed_step.tolerance, JUNCTION_APPROACH_DISTANCE_CM) : speed_step.tolerance;
    float remaining = speed_step.distance - entry_lead - progress;
    float braking_distance = fmaxf(JUNCTION_APPROACH_DISTANCE_CM,
        (base_speed*base_speed-slow_speed*slow_speed)/(2.0f*JUNCTION_APPROACH_DECEL_CM_S2));
    if (approach_slow && speed_step.action != STRAIGHT && (f->route.state == ROUTE_CANDIDATE || remaining <= 0.0f)) {
        base_speed = slow_speed;
    } else if (approach_slow && speed_step.action != STRAIGHT && remaining < braking_distance) {
        float fraction = remaining / braking_distance;
        base_speed = sqrtf(slow_speed * slow_speed +
                           (base_speed * base_speed - slow_speed * slow_speed) * fraction);
    }
#else
    float base_speed = LINE_FOLLOW_SPEED_CM_S;
#endif
#if PRESET_ROUTE_ENABLED
    if (f->route.state == ROUTE_EXECUTING) {
        f->exit_accelerating = false;
        if (previous_route != ROUTE_EXECUTING) {
            pid_reset(&f->pid);
            speed_control_reset(&f->speed);
        }
        f->abnormal = false;
        f->state = LINE_JUNCTION;
        float turn_outer = (TURN_APPROACH_SLOWDOWN_ENABLED || route_step_is_auto(step)) &&
            route_current_action(&f->route) != STRAIGHT ?
            fminf((float)step.speed, JUNCTION_APPROACH_SPEED_CM_S) : (float)step.speed;
        float turn_base = fminf(turn_outer, JUNCTION_TURN_SPEED_CM_S);
        float correction;
        Action action = route_current_action(&f->route);
        bool have_turn_line=route_is_narrow_line(obs) || f->route.turn_exit_branch;
        if (action == STRAIGHT) turn_base = turn_outer;
        if (action == LEFT || action == RIGHT) {
            float direction = action == LEFT ? -1.0f : 1.0f;
            float turn = direction * (f->odometry.heading_rad - f->route.entry_heading);
            bool corner_arc = route_uses_corner_arc(&f->route);
            if (!corner_arc) turn_base = .5f * turn_outer;
            float lead = GRAY_FORWARD_OFFSET_CM - CORNER_RADIUS_CM;
            float arc_distance = f->odometry.distance_cm - f->route.entry_cm - lead;
            if (corner_arc && turn >= JUNCTION_TURN_TARGET_RAD)
                f->route.corner_arc_complete = true;
            bool arc_planning = corner_arc &&
                (!f->route.corner_arc_complete ||
                 (route_prepares_straight(&f->route) && have_turn_line));
            if (arc_planning) {
                /* 从触发点按 s/R 规划航向（当前 R=L，切点前距离为零）。
                 * 不把转弯中途的灰度位置送入 PID；出口判定仍完全由 route_update 决定。 */
                pid_reset(&f->pid);
                float track = f->odometry.calibration.track_cm;
                if (CORNER_RADIUS_CM <= track * 0.5f) {
                    line_follow_stop(f, LINE_STOP_CALIBRATION);
                    return;
                }
                turn_base = fminf(turn_base, WHEEL_TARGET_MAX_CM_S /
                                 (1.0f + track / (2.0f * CORNER_RADIUS_CM)));
                float planned_turn = fmaxf(0.0f, fminf(JUNCTION_TURN_TARGET_RAD,
                                               arc_distance / CORNER_RADIUS_CM));
                float planned_base = next_base(f, turn_base, target_rise(f), dt);
                float feedforward = arc_distance >= 0.0f && planned_turn < JUNCTION_TURN_TARGET_RAD ?
                    planned_base * track / (2.0f * CORNER_RADIUS_CM) : 0.0f;
                correction = direction * (feedforward + CORNER_HEADING_KP * (planned_turn - turn));
            } else {
                /* 没有可靠出口线就继续本项指定方向，不因达到目标角改成直行或回正。
                 * 普通转弯保持外轮规划值；圆弧完成后保持正向曲率，内轮仍前进。
                 * 路径层的过转/反向/超时保护仍先于这里执行。 */
                if (!have_turn_line) {
                    pid_reset(&f->pid);
                    if (!corner_arc && f->route.capture_line_seen &&
                        (uint32_t)(now_ms-f->route.capture_line_ms)<=JUNCTION_CAPTURE_LOSS_GRACE_MS) {
                        correction=0; /* 见线后短暂抖动不立即恢复120/0扫转。 */
                    } else if (!corner_arc && f->exit_anchor_valid) {
                        correction=exit_recovery_correction(f,turn_outer);
                    } else if (corner_arc) {
                        float ratio=f->odometry.calibration.track_cm/(2.0f*CORNER_RADIUS_CM);
                        turn_base=fminf(turn_base,WHEEL_TARGET_MAX_CM_S/(1.0f+ratio));
                        correction=direction*turn_base*ratio;
                    } else correction=direction*turn_base;
                } else if (f->route.turn_capturing) {
                    if (!corner_arc && !f->route.turn_exit_branch) {
                        /* 见到有效出口线立即停止扫转。只用有界位置反馈，
                         * 不叠加目标航向、强制补角或速度倍增，先让内轮跟上。 */
                        turn_base=.5f*turn_outer;
                        if (!pid_update(&f->pid,obs.position,dt,
                            fminf(JUNCTION_RECAPTURE_MAX_HALF_RATIO*next_outer(f,turn_outer,dt),JUNCTION_CAPTURE_MAX_HALF_DIFF_CM_S),&correction)) {
                            line_follow_stop(f,LINE_STOP_FAULT);
                            return;
                        }
                    } else {
                        /* 捕获侧边出口线，向路径目标航向和目标线共同收敛。
                         * 先前固定支点转到中心才接管，可能把侧边线扫出探头范围。 */
                        turn_base = corner_arc ? fminf(turn_outer, JUNCTION_CAPTURE_SPEED_CM_S) :
                                                .5f * turn_outer;
                        float line_correction=0;
                        if (f->route.turn_exit_branch) pid_reset(&f->pid);
                        else if (!pid_update(&f->pid, obs.position, dt, turn_base, &line_correction)) {
                            line_follow_stop(f, LINE_STOP_FAULT);
                            return;
                        }
                        correction = line_correction + JUNCTION_CAPTURE_HEADING_KP *
                            (f->route.entry_heading + route_turn_target_rad(&f->route) -
                             f->odometry.heading_rad);
                        /* 普通直角已经超出出口航向时，侧线 PID 不能抵消回正并继续同向转。
                         * 仍等待路径层的真实出口确认，不以航向完成直接切段。 */
                        if (!corner_arc && step.action != CUSTOM_TURN &&
                            turn >= fabsf(route_turn_target_rad(&f->route))) {
                            float recovery = JUNCTION_CAPTURE_HEADING_KP *
                                (turn - fabsf(route_turn_target_rad(&f->route)));
                            correction = direction * fminf(direction * correction, -recovery);
                        }
                        /* 最小同向半差防止未到出口角时位置/航向修正抵消。
                         * 接近出口且连续窄线/前进证据已满足有界退出条件后，允许反向回线：
                         * 实车 +120° 在 119.16°、position=-3 时被旧下限压住，直至动作超时。
                         * 证据由本帧 route_update 更新；无线/歧义/不足角度会清空，重新启用下限。
                         * 中心线确认仍由路径层负责，多区域附带分支不使用此放行。 */
                        bool exit_angle_confirmed = !f->route.turn_exit_branch &&
                            f->route.exit_candidate_pending &&
                            (uint32_t)(now_ms - f->route.exit_candidate_ms) >= JUNCTION_EXIT_CONFIRM_MS &&
                            f->odometry.distance_cm - f->route.exit_candidate_cm >= JUNCTION_EXIT_CONFIRM_CM;
                        bool unfinished_right_angle = !corner_arc && step.action != CUSTOM_TURN &&
                            turn < fabsf(route_turn_target_rad(&f->route));
                        if (unfinished_right_angle ||
                            (turn < route_turn_threshold_rad(&f->route, JUNCTION_TURN_MIN_RAD) &&
                             !exit_angle_confirmed)) {
                            float min_turn = fminf(turn_base, JUNCTION_CAPTURE_MIN_TURN_CM_S);
                            correction = direction * fmaxf(direction * correction, min_turn);
                        }
                    }
                } else {
                    pid_reset(&f->pid);
                    /* 已见可靠线但未进入位置捕获时，用剩余航向误差收敛。
                     * 无线/歧义已由上方同向续转分支处理，不在这里锁定目标航向。
                     * 目标超调可回正，单轮目标仍非负；退出确认由route负责。 */
                    if (corner_arc ||
                        turn >= route_turn_threshold_rad(&f->route, JUNCTION_TURN_SLOW_RAD)) {
                        float heading_correction = JUNCTION_CAPTURE_HEADING_KP *
                            (f->route.entry_heading + route_turn_target_rad(&f->route) -
                             f->odometry.heading_rad);
                        correction = fmaxf(-turn_base, fminf(turn_base, heading_correction));
                    } else correction = direction * turn_base;
                }
            } /* 圆弧完成后交还原有捕线；非直角使用原控制。 */
        } else {
            correction = HEADING_KP_CM_S_PER_RAD *
                         (f->route.entry_heading - f->odometry.heading_rad);
        }
        if (action == STRAIGHT) straight_targets(f, turn_base, correction, dt);
        else if (!route_uses_corner_arc(&f->route)) {
            /* 保持外轮规划速度时，位置/航向半差按速度同比换算，保持每厘米纠偏能力。
             * 否则120 cm/s外轮配原低速半差会形成大半径，错过出口线。 */
            float turn=fabsf(f->odometry.heading_rad-f->route.entry_heading);
            if (f->route.turn_capturing && f->route.turn_exit_branch)
                correction *= turn_outer/(2.0f*JUNCTION_CAPTURE_SPEED_CM_S);
            else if (!f->route.turn_capturing && have_turn_line &&
                     turn>=route_turn_threshold_rad(&f->route,JUNCTION_TURN_SLOW_RAD))
                correction *= turn_outer/(2.0f*JUNCTION_TURN_SLOW_CM_S);
            outer_targets(f, turn_outer, correction, dt);
        }
        else {
            if (route_uses_corner_arc(&f->route)) {
                float planned_base=next_base(f,turn_base,target_rise(f),dt);
                float limit=CORNER_MAX_HALF_RATIO*planned_base;
                correction=fmaxf(-limit,fminf(limit,correction));
            }
            targets(f, turn_base, correction, dt);
        }
    } else if (f->route.state == ROUTE_CLEARING && !route_uses_corner_arc(&f->route) &&
               step.action != CUSTOM_TURN && route_current_action(&f->route) != STRAIGHT) {
        /* 出口确认后继续位置PID沿线离口，不能只保持航向而漏掉线反馈。
         * 无可靠观测时冻结PID，短暂抖动不重启大差速扫转。 */
        f->state=narrow?LINE_TRACKING:LINE_JUNCTION;
        f->abnormal=false;
        if (narrow) {
            if(stable) {
                f->have_reliable=true;
                f->last_reliable_ms=now_ms;
                f->reliable_position=obs.position;
            }
            f->route.capture_line_seen=true;
            f->route.capture_line_ms=now_ms;
            float correction;
            if(!pid_update(&f->pid,obs.position,dt,
                fminf(JUNCTION_RECAPTURE_MAX_HALF_RATIO*next_outer(f,base_speed,dt),JUNCTION_CAPTURE_MAX_HALF_DIFF_CM_S),&correction)) {
                line_follow_stop(f,LINE_STOP_FAULT);
                return;
            }
            outer_targets(f,base_speed,correction,dt);
        } else {
            pid_reset(&f->pid);
            float heading=f->route.line_exit_confirmed?f->route.line_exit_heading_rad:
                f->route.entry_heading+route_turn_target_rad(&f->route);
            float correction=JUNCTION_CAPTURE_HEADING_KP*(heading-f->odometry.heading_rad);
            if(f->route.capture_line_seen &&
               (uint32_t)(now_ms-f->route.capture_line_ms)<=JUNCTION_CAPTURE_LOSS_GRACE_MS) correction=0;
            else if(f->exit_anchor_valid) correction=exit_recovery_correction(f,base_speed);
            outer_targets(f,base_speed,correction,dt);
        }
    } else if (f->route.state == ROUTE_CLEARING && route_prepares_straight(&f->route)) {
        /* 圆弧出口尚在本项离口时就可能扫到电阻侧线：全程保持规划出口轴。 */
        pid_reset(&f->pid);
        f->state = LINE_JUNCTION;
        f->abnormal = false;
        straight_targets(f, base_speed,
                HEADING_KP_CM_S_PER_RAD * (f->route.entry_heading +
                    route_turn_target_rad(&f->route) - f->odometry.heading_rad), dt);
    } else if (f->route.state == ROUTE_FOLLOW &&
               route_waiting_after_straight(&f->route, f->odometry.distance_cm)) {
        /* 电阻出口之后仍须直行到下一转弯的标称点；侧线既不能提前派发转弯，也不能进入 PID。 */
        pid_reset(&f->pid);
        f->state = LINE_TRACKING;
        f->abnormal = false;
        straight_targets(f, base_speed,
                HEADING_KP_CM_S_PER_RAD * (f->route.segment_heading_rad - f->odometry.heading_rad), dt);
    } else if (step.action == STRAIGHT && !route_step_is_auto(step) && f->route.state == ROUTE_FOLLOW) {
        /* 连续 STRAIGHT 之间也可能仍在电阻内部：窗口前沿用路段轴，不能重新跟随侧线。 */
        pid_reset(&f->pid);
        f->state = LINE_TRACKING;
        f->abnormal = false;
        straight_targets(f, base_speed,
                HEADING_KP_CM_S_PER_RAD * (f->route.segment_heading_rad - f->odometry.heading_rad), dt);
    } else if (f->route.state == ROUTE_CLEARING && route_current_action(&f->route) == STRAIGHT) {
        /* 直行离口去重也保持入口航向，防止短暂回线后又被电阻侧线拉走。 */
        pid_reset(&f->pid);
        f->state = LINE_JUNCTION;
        f->abnormal = false;
        straight_targets(f, base_speed,
                HEADING_KP_CM_S_PER_RAD * (f->route.entry_heading - f->odometry.heading_rad), dt);
    } else if (f->route.state == ROUTE_CLEARING && f->route.turn_exit_branch) {
        /* 原始观测仍为多区域，禁止送入普通位置 PID；短离口期间仅保持出口航向。 */
        pid_reset(&f->pid);
        f->state=LINE_JUNCTION;
        f->abnormal=false;
        targets(f,fminf(base_speed,JUNCTION_CAPTURE_SPEED_CM_S),
                JUNCTION_CAPTURE_HEADING_KP*(f->route.entry_heading+
                route_turn_target_rad(&f->route)-f->odometry.heading_rad),dt);
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
        float next = next_outer(f,base,dt);
        float correction;
        if (!pid_update(&f->pid, obs.position, dt,
                        .5f*next, &correction)) {
            line_follow_stop(f, LINE_STOP_FAULT);
            return;
        }
        outer_targets(f, base, correction, dt);
    } else {
        /* Unconfirmed junctions, wide/multiple regions and no-line all hold heading.
         * Do not integrate graphics into the normal white-line position PID. */
        if (!f->abnormal) {
            f->abnormal = true;
            f->hold_heading_rad = f->odometry.heading_rad;
            f->abnormal_origin_cm = f->odometry.distance_cm;
        }
        f->state = LINE_GAP_HOLD;
        /* 歧义不额外限速；路径自身的接近/候选速度规划仍生效。 */
        float base = obs.kind == GRAY_NONE ? fminf(base_speed, HOLD_SPEED_CM_S) : base_speed;
        targets(f, base,
                HEADING_KP_CM_S_PER_RAD * (f->hold_heading_rad - f->odometry.heading_rad), dt);
    }
#if PRESET_ROUTE_ENABLED
    if ((f->route.state == ROUTE_EXECUTING || f->route.state == ROUTE_CLEARING) &&
        (f->turn_trace.count == 0 || (uint32_t)(now_ms-f->turn_trace.last_ms) >= 20U)) {
        TurnTrace *trace = &f->turn_trace;
        trace->samples[trace->next] = (TurnTraceSample){
            .ms=now_ms, .distance_cm=f->odometry.distance_cm,
            .turn_rad=f->odometry.heading_rad-f->route.entry_heading,
            .position=obs.position, .left_target=f->left_target_cm_s,
            .right_target=f->right_target_cm_s, .left_speed=f->odometry.left_speed_cm_s,
            .right_speed=f->odometry.right_speed_cm_s, .mask=obs.line_mask,
            .kind=(uint8_t)obs.kind, .route_state=(uint8_t)f->route.state,
            .capturing=f->route.turn_capturing
        };
        trace->next=(uint8_t)((trace->next+1U)%TURN_TRACE_CAPACITY);
        if (trace->count<TURN_TRACE_CAPACITY) ++trace->count;
        trace->last_ms=now_ms;
    }
    if(f->route.state==ROUTE_EXECUTING && f->route.index<2 &&
       f->route.turn_capturing && !f->route.turn_exit_branch && narrow) {
        TurnExitDiagnostics *record=&f->turn_exit[f->route.index];
        if(!record->capture_seen) {
            record->capture_seen=true;
            record->capture_ms=now_ms;
            record->capture_turn_rad=(f->odometry.heading_rad-f->route.entry_heading)*
                (route_current_action(&f->route)==LEFT?-1.0f:1.0f);
            record->capture_mask=obs.line_mask;
            record->capture_left_target_cm_s=f->left_target_cm_s;
            record->capture_right_target_cm_s=f->right_target_cm_s;
        }
    }
#endif
    if (!speed_control_update(&f->speed, f->left_target_cm_s, f->right_target_cm_s,
                              f->odometry.left_speed_cm_s, f->odometry.right_speed_cm_s,
                              dt, &f->command)) line_follow_stop(f, LINE_STOP_ENCODER);
#if PRESET_ROUTE_ENABLED
    StraightEntrySnapshot *entry_snapshot = NULL;
    float entry_axis = f->route.segment_heading_rad;
    if (previous_prepares_straight && previous_route == ROUTE_EXECUTING &&
        f->route.state == ROUTE_CLEARING) {
        entry_snapshot = &f->straight_entry_aligned;
        entry_axis = f->route.entry_heading + route_turn_target_rad(&f->route);
    } else if (previous_prepares_straight && previous_route == ROUTE_CLEARING &&
               f->route.index != previous_index && f->route.state == ROUTE_FOLLOW) {
        entry_snapshot = &f->straight_entry_handoff;
    }
    if (entry_snapshot != NULL && !entry_snapshot->valid) {
        *entry_snapshot = (StraightEntrySnapshot){
            .valid = true, .pid_active = f->pid.primed, .sample_ms = now_ms,
            .route_index = (uint32_t)f->route.index, .route_state = f->route.state,
            .observation = obs, .heading_rad = f->odometry.heading_rad,
            .target_heading_rad = entry_axis, .outgoing_cm = f->route.outgoing_cm,
            .left_target_cm_s = f->left_target_cm_s, .right_target_cm_s = f->right_target_cm_s
        };
    }
#endif
#endif
}

void line_follow_update(LineFollow *f, GrayObservation obs, EncoderCounts counts,
                        uint32_t now_ms, float dt, bool start_request)
{
    line_follow_update_heading(f, obs, counts, now_ms, dt, start_request, false, 0.0f);
}
