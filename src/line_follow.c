#include "braking_planner.h"
#include "line_follow.h"
#include "control_config.h"
#include "motion_config.h"
#include <math.h>
#include <string.h>

/**
 * @brief 判断停止原因是否属于必须复位才能解除的任务或编码器故障。
 */
#if PRESET_ROUTE_ENABLED
/* Ordinary maneuvers own gray loss until their existing phase bounds abort.
 * HEXAGON retains its independent gray-loss contract. */
static bool route_owns_gray_loss(const RoutePlanner *p)
{
    return p->state==ROUTE_BRAKING || p->state==ROUTE_RECOVERING ||
        ((p->state==ROUTE_EXECUTING || p->state==ROUTE_CLEARING) &&
         ((p->index<p->count && p->steps[p->index].action!=HEXAGON) ||
          (p->state==ROUTE_EXECUTING && p->turn_approaching)));
}

/* A front sensor may capture a perpendicular exit while the axle still yaws.
 * Only the adjacent non-target passage may borrow this bounded exit reference;
 * normal narrow-line steering continues to use position PID. */
static float passage_heading(const LineFollow *f, float fallback)
{
    return f->exit_heading_pending &&
        fabsf(f->exit_heading_rad-f->odometry.heading_rad)<=JUNCTION_SETTLE_BAND_RAD ?
        f->exit_heading_rad : fallback;
}
#endif

static bool locked(LineStopReason reason)
{
    return reason == LINE_STOP_FAULT || reason == LINE_STOP_ENCODER ||
           reason == LINE_STOP_IMU || reason == LINE_STOP_TILT;
}

void line_follow_set_braking_accel(LineFollow *f, bool valid, float accel_cm_s2)
{
    f->braking_imu_sample_valid=valid && isfinite(accel_cm_s2);
    f->braking_imu_sample_cm_s2=f->braking_imu_sample_valid?accel_cm_s2:0;
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
#if MPU6050_ENABLED
    f->heading_source = HEADING_IMU;
#else
    f->heading_source = HEADING_ENCODER;
#endif
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
            .component_recovery_age_ms=f->route.component_recovery_used?
                (uint32_t)(f->sample_ms-f->route.component_recovery_ms):0U,
            .component_recovery_progress_cm=f->route.component_recovery_used?
                fabsf(f->odometry.distance_cm-f->route.component_recovery_cm):0.0f,
            .braking_accel_cm_s2=f->braking_accel_cm_s2,
            .braking_required_cm_s2=f->braking_required_cm_s2,
            .braking_reverse_ms=f->braking_reverse_ms,
            .braking_imu_used=f->braking_imu_used,
            .braking_reverse_active=f->braking_reverse_active,
            .turn_exit_branch = 0,
            .exit_line_mask = 0,
            .handoff_early = 0,
            .segment_start_progress_cm = 0,
            .valid = true, .reason = reason, .state_before_stop = f->state,
            .route_state = f->route.state, .route_state_before_update = f->route_state_before_update,
            .route_failure = f->route.failure, .speed_fault = f->speed.fault,
            .speed_fault_wheel = f->speed.fault_wheel, .observation = f->observation,
            .counts = f->sample_counts,
            .sample_ms = f->sample_ms,
            .reliable_age_ms = (uint32_t)(f->sample_ms - f->last_reliable_ms),
            .route_phase_age_ms = (uint32_t)(f->sample_ms - f->route.phase_ms),
            .route_index = (uint32_t)f->route.index, .dt_s = f->sample_dt_s,
            .junctions_seen=f->route.junctions_seen,
            .final_segment_active=f->route.final_segment_active,
            .final_segment_progress_cm=f->odometry.distance_cm-f->route.final_segment_origin_cm,
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
            .exit_confirm_samples = f->route.exit_count,
            .heading_source = f->heading_source,
            .heading_fallback_reason = f->heading_fallback_reason,
            .heading_fallback_ms = f->heading_fallback_ms
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
/* 完成低速离口后快速提速，延续直到达到规划目标；START 保留原包络。 */
static float target_rise(const LineFollow *f)
{
#if PRESET_ROUTE_ENABLED && TURN_EXIT_ACCEL_ENABLED
    if (f->exit_accelerating && f->route.state != ROUTE_CLEARING)
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
    if (!isfinite(correction)) correction = 0.0f;
    /* 逐轮 [0,MAX] 限幅：基准等于天花板时不再把差速钳成零，转向方向永远保留。 */
    f->left_target_cm_s = fmaxf(0.0f, fminf(WHEEL_TARGET_MAX_CM_S, f->base + correction));
    f->right_target_cm_s = fmaxf(0.0f, fminf(WHEEL_TARGET_MAX_CM_S, f->base - correction));
    f->correction = 0.5f * (f->left_target_cm_s - f->right_target_cm_s);
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
/* 转弯混合：外轮保持 outer（受表内 turn_speed 限制），内轮 = outer − 2|correction|。
 * correction=几何差速时内/外比恰为 (R−t/2)/(R+t/2)，与圆弧规划一致；
 * 航向修正可正向加强到内轮为零，不再被几何差速钳死（欠转打滑可补角）。 */
static void turn_targets(LineFollow *f, float outer, float correction, float dt)
{
    float requested = outer;
    outer = next_outer(f, outer, dt);
    if (outer >= requested) f->exit_accelerating = false;
    correction = fmaxf(-0.5f * outer, fminf(0.5f * outer, correction));
    float mean = outer - fabsf(correction);
    f->left_target_cm_s = mean + correction;
    f->right_target_cm_s = mean - correction;
    f->base = 0.5f * (f->left_target_cm_s + f->right_target_cm_s);
    f->correction = 0.5f * (f->left_target_cm_s - f->right_target_cm_s);
    f->startup_elapsed_s = fminf(STARTUP_RAMP_TIME_S, f->startup_elapsed_s + dt);
}
#endif

#if PRESET_ROUTE_ENABLED
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
        f->braking_slew = (BrakingSlew){0};
        f->braking_active=false;
        pid_reset(&f->pid);
    }
    if ((LOST_LINE_STOP_ENABLED || PRESET_ROUTE_ENABLED) && f->abnormal &&
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
/* Forward declaration: the controller body uses this before its definition. */
#if defined(__GNUC__)
#define LINE_FOLLOW_MAYBE_UNUSED __attribute__((unused))
#else
#define LINE_FOLLOW_MAYBE_UNUSED
#endif
static LINE_FOLLOW_MAYBE_UNUSED inline bool gray_plausible(LineFollow *f,GrayObservation obs);

static void line_follow_update_heading_impl(LineFollow *f, GrayObservation obs, EncoderCounts counts,
                        uint32_t now_ms, float dt, bool start_request,
                        bool imu_valid, float imu_heading_rad)
{
#if !MPU6050_ENABLED
    (void)imu_valid;
    (void)imu_heading_rad;
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
    if (f->heading_source == HEADING_IMU && (!imu_valid || !isfinite(imu_heading_rad))) {
        f->heading_source = HEADING_ENCODER;
        f->heading_fallback_reason = f->odometry.primed ? HEADING_FALLBACK_DATA : HEADING_FALLBACK_STARTUP;
        f->heading_fallback_ms = now_ms;
    }
#endif
    float previous_heading=f->odometry.heading_rad;
    bool heading_primed=f->odometry.primed;
    if (!odometry_update_with_heading(&f->odometry, counts, dt,
#if MPU6050_ENABLED
                                      f->heading_source == HEADING_IMU, imu_heading_rad
#else
                                      false, 0.0f
#endif
                                      )) {
        line_follow_stop(f, LINE_STOP_ENCODER);
        if (start_request) f->start_result = LINE_START_REJECTED_FAULT;
        return;
    }
    if (heading_primed) {
        float rate=(f->odometry.heading_rad-previous_heading)/dt;
        float alpha=dt/(JUNCTION_YAW_FILTER_TAU_S+dt);
        f->turn_yaw_rate_rad_s+=alpha*(rate-f->turn_yaw_rate_rad_s);
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
        f->auto_resume_count = f->auto_resume_last_ms = 0;
        f->auto_resume_snapshot = (LineStopSnapshot){0};
        f->straight_entry_aligned = f->straight_entry_handoff = (StraightEntrySnapshot){0};
        memset(f->turn_exit,0,sizeof(f->turn_exit));
        memset(&f->turn_trace,0,sizeof(f->turn_trace));
        memset(&f->navigation_trace,0,sizeof(f->navigation_trace));
        f->exit_anchor_valid=f->exit_recovery_active=false;
        f->exit_heading_pending=false;
        f->exit_lateral_error_cm=0;
        f->start_result = LINE_START_ACCEPTED;
        f->state = LINE_TRACKING;
        f->stop_reason = LINE_STOP_NONE;
        f->stop_snapshot = (LineStopSnapshot){0};
        f->last_reliable_ms = now_ms;
        f->have_reliable = true;
        f->abnormal = false;
        f->component_cross_latched=false;
        f->base = 0.0f;
        f->startup_elapsed_s = 0.0f;
        f->braking_slew = (BrakingSlew){0};
        f->braking_active=false;
        f->braking_speed_primed=false;
        f->braking_reverse_ms=0;
        f->braking_reverse_active=false;
        f->braking_required_cm_s2=0;
        f->turn_yaw_rate_rad_s=0;
        pid_reset(&f->pid);
        speed_control_reset(&f->speed);
    }
    /* Printed component graphics (hollow resistors, transistor V, battery
     * plates) must be crossable. Latch the crossing window as soon as the
     * component signature appears; while latched, hold-and-cross is allowed
     * for a bounded distance/time. A real line loss or an unrelated ambiguity
     * that was never preceded by a component signature keeps 400 ms / 8 cm. */
    if(!f->abnormal) f->component_cross_latched=false;
    else if(!f->component_cross_latched && gray_is_component_pattern(obs)) {
        f->component_cross_latched=true;
        f->component_cross_ms=now_ms;
        f->component_cross_cm=f->odometry.distance_cm;
    }
    bool component_crossing=f->abnormal && f->component_cross_latched &&
        (uint32_t)(now_ms-f->component_cross_ms)<COMPONENT_CROSS_MS &&
        fabsf(f->odometry.distance_cm-f->component_cross_cm)<COMPONENT_CROSS_MAX_CM;
    /* Pose mode: a printed component hides the wire for up to ~10 cm along the
     * segment and the pose says the car is still on it, so reuse the bounded
     * component-crossing window instead of the plain 400 ms / 8 cm cut. */
    bool pose_bounded_loss=f->route.pose_mode &&
        (f->route.state==ROUTE_FOLLOW || f->route.state==ROUTE_CANDIDATE);
    uint32_t lost_ms=pose_bounded_loss?POSE_BOUNDED_LOSS_MS:INTERFERENCE_TIMEOUT_MS;
    float lost_cm=pose_bounded_loss?POSE_BOUNDED_LOSS_CM:INTERFERENCE_MAX_DISTANCE_CM;
    bool gray_loss_expired=(LOST_LINE_STOP_ENABLED || PRESET_ROUTE_ENABLED) && f->abnormal &&
        !component_crossing &&
        ((uint32_t)(now_ms-f->last_reliable_ms)>=lost_ms ||
         fabsf(f->odometry.distance_cm-f->abnormal_origin_cm)>=lost_cm);
#if !PRESET_ROUTE_ENABLED
    if(gray_loss_expired) {line_follow_stop(f,LINE_STOP_LOST);return;}
#endif
#if PRESET_ROUTE_ENABLED
    bool route_owned_gray_loss=route_owns_gray_loss(&f->route);
    RouteState previous_route = f->route.state;
    size_t previous_index = f->route.index;
    float previous_exit_axis=f->route.entry_heading+route_turn_target_rad(&f->route);
    bool right_angle_handoff=previous_route==ROUTE_EXECUTING &&
        !f->route.passing_junction && f->route.steps[previous_index].action!=HEXAGON &&
        fabsf(fabsf(route_turn_target_rad(&f->route))-JUNCTION_TURN_TARGET_RAD)<0.0001f;
    bool completing_turn=previous_route==ROUTE_CLEARING &&
        !f->route.passing_junction && route_current_action(&f->route)!=STRAIGHT;
    float previous_turn=(f->odometry.heading_rad-f->route.entry_heading)*
        (route_current_action(&f->route)==LEFT?-1.0f:1.0f);
    if(previous_route==ROUTE_CLEARING && previous_index<2 && f->turn_exit[previous_index].valid)
        f->turn_exit[previous_index].minimum_turn_rad=fminf(
            f->turn_exit[previous_index].minimum_turn_rad,previous_turn);
    route_set_speed(&f->route, f->odometry.left_speed_cm_s, f->odometry.right_speed_cm_s);
    /* Odometry frame -> track frame. The odometry origin is the START point in
     * the car's initial heading (x forward, y right); the planner's anchored
     * start pose places it in track coordinates (production: P headong north). */
    {
        float start_h=f->route.start_heading_rad;
        float cos_h=cosf(start_h),sin_h=sinf(start_h);
        float axle_x=f->route.start_x_cm+f->odometry.x_cm*cos_h-f->odometry.y_cm*sin_h;
        float axle_y=f->route.start_y_cm+f->odometry.x_cm*sin_h+f->odometry.y_cm*cos_h;
        route_update_pose(&f->route, obs, f->odometry.distance_cm, f->odometry.heading_rad, now_ms,
                          axle_x, axle_y);
    }
    /* Let real same-frame node confirmation transfer the budget. Unconfirmed
     * candidates/FOLLOW still expire before normal line recovery can renew it. */
    if(gray_loss_expired && !route_owned_gray_loss && !route_owns_gray_loss(&f->route)) {
        line_follow_stop(f,LINE_STOP_LOST);return;
    }
    if(f->route.state==ROUTE_EXECUTING && !f->route.passing_junction &&
       previous_route!=ROUTE_EXECUTING) f->exit_heading_pending=false;
    if(right_angle_handoff && f->route.state==ROUTE_CLEARING) {
        f->exit_heading_rad=previous_exit_axis;
        f->exit_heading_origin_cm=f->odometry.distance_cm;
        f->exit_heading_pending=true;
    }
    /* The borrowed axis is local to the front-sensor handoff corridor. */
    if(f->exit_heading_pending &&
       fabsf(f->odometry.distance_cm-f->exit_heading_origin_cm)>GRAY_FORWARD_OFFSET_CM)
        f->exit_heading_pending=false;
    if(f->exit_heading_pending && f->route.state==ROUTE_FOLLOW &&
       f->route.approach_center_count>=LINE_CONFIRM_SAMPLES &&
       (fabsf(f->odometry.heading_rad-f->exit_heading_rad)<=JUNCTION_PASS_ALIGNED_RAD ||
        fabsf(f->turn_yaw_rate_rad_s)<=JUNCTION_PASS_SETTLED_YAW_RAD_S))
        f->exit_heading_pending=false;
    f->exit_recovery_active=f->route.component_recovery_active;
    if (completing_turn && f->route.state==ROUTE_FOLLOW)
        f->exit_accelerating=TURN_EXIT_ACCEL_ENABLED;
    if(previous_route!=ROUTE_EXECUTING && f->route.state==ROUTE_EXECUTING) {
        f->exit_anchor_valid=false;
        f->exit_lateral_error_cm=0;
    }
    if(!f->exit_anchor_valid && (f->route.state==ROUTE_EXECUTING || f->route.state==ROUTE_CLEARING) &&
       !route_uses_corner_arc(&f->route) && f->route.steps[f->route.index].action!=CUSTOM_TURN &&
       f->route.steps[f->route.index].action!=HEXAGON &&
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
        record->line_confirmed_exit=f->route.line_confirmed_exit_used;
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
    if (f->route.state == ROUTE_FAILED &&
        !route_recover_navigation(&f->route,f->odometry.distance_cm,f->odometry.heading_rad,now_ms)) {
        line_follow_stop(f, LINE_STOP_ROUTE); return;
    }
    if(f->route.state==ROUTE_RECOVERING && previous_route!=ROUTE_RECOVERING) {
        pid_reset(&f->pid);f->abnormal=false;f->exit_heading_pending=false;
    }
    if (f->route.state == ROUTE_FINISHED) { line_follow_stop(f, LINE_STOP_ROUTE_COMPLETE); return; }
    if (previous_route == ROUTE_EXECUTING && f->route.state == ROUTE_CLEARING) {
        f->exit_accelerating = false; /* 离口完成才开启下一段加速。 */
        /* Capture and clearance share the same line/velocity loops. Preserve
         * lateral derivative and reverse evidence through this FSM boundary. */
        f->abnormal = false;
        if(f->route.line_exit_confirmed) f->have_reliable = true;
        f->last_reliable_ms = now_ms;
    }
    RouteStep step = f->route.steps[f->route.index];
    float entry_speed_limit = route_entry_speed_limit(&f->route);
    float base_speed = route_cruise_speed(&f->route);
    RouteStep planning_step=f->route.steps[route_planning_index(&f->route)];
    float planning_origin=route_planning_origin(&f->route);
    float planning_entry_speed=route_planning_entry_speed_limit(&f->route);
    float navigation_wheel_cap=WHEEL_TARGET_MAX_CM_S;
    /* No unconditional slow-down before known nodes. Distance-envelope
     * planning decelerates from measured speed; the node's 16 cm/s gate
     * prevents a turn until actually reached. */
    /* A graphic cannot authorize segment-speed blind passage. Only reliable
     * centered exit evidence restores the unchanged distance envelope. */
    bool passing_without_exit=(f->route.passing_junction &&
        f->route.state==ROUTE_CLEARING &&
        (f->route.component_recovery_active || !stable ||
         !route_is_clear_line(&f->route,obs) || f->route.exit_count<LINE_CONFIRM_SAMPLES)) ||
        (f->route.state==ROUTE_CANDIDATE &&
         !route_target_ready(&f->route,f->odometry.distance_cm,true) &&
         (!narrow || fabsf(obs.position)>JUNCTION_CENTER_LIMIT));
    if(passing_without_exit) {
        base_speed=fminf(base_speed,COMPONENT_RECOVERY_SPEED_CM_S);
        navigation_wheel_cap=fminf(navigation_wheel_cap,COMPONENT_RECOVERY_SPEED_CM_S);
    }
    /* Segment length selects cruise speed; every known length needs the same
     * entry-speed envelope, including short segments and clearance previews. */
    if (TURN_APPROACH_SLOWDOWN_ENABLED && planning_step.segment_length_cm>0 &&
        f->route.final_segment_active && (f->route.state==ROUTE_FOLLOW ||
        f->route.state==ROUTE_CANDIDATE || f->route.state==ROUTE_CLEARING)) {
        if(!f->braking_active || f->braking_origin_cm!=planning_origin) {
            f->braking_slew=(BrakingSlew){fmaxf(f->left_target_cm_s,f->right_target_cm_s),0};
            f->braking_origin_cm=planning_origin;
            f->braking_active=true;
            f->braking_speed_primed=false;
            f->braking_reverse_ms=0;
            f->braking_reverse_active=false;
        }
        float speed=fmaxf(fabsf(f->odometry.left_speed_cm_s),
                          fabsf(f->odometry.right_speed_cm_s));
        float raw_accel=f->braking_imu_sample_valid && f->heading_source==HEADING_IMU ?
            f->braking_imu_sample_cm_s2 :
            (f->braking_speed_primed ? (speed-f->braking_previous_speed_cm_s)/dt : 0.0f);
        f->braking_imu_used=f->braking_imu_sample_valid && f->heading_source==HEADING_IMU;
        f->braking_previous_speed_cm_s=speed;
        if(!f->braking_speed_primed) f->braking_accel_cm_s2=raw_accel;
        else f->braking_accel_cm_s2+=dt/(BRAKING_ACCEL_FILTER_TAU_S+dt)*
            (raw_accel-f->braking_accel_cm_s2);
        f->braking_speed_primed=true;
        /* Generated lengths describe junction geometry. Account for the front
         * probe and the previous turn's axle advance once, identically in TS. */
        float exit_advance=route_planning_index(&f->route)>0?WHEEL_TRACK_CM:0;
        float target_distance=fmaxf(0,planning_step.segment_length_cm-
            GRAY_FORWARD_OFFSET_CM-exit_advance);
        BrakingPlan plan = braking_plan(base_speed, fminf(base_speed,planning_entry_speed),
            target_distance, f->odometry.distance_cm-planning_origin,
            fmaxf(fabsf(f->odometry.left_speed_cm_s),fabsf(f->odometry.right_speed_cm_s)),
            BRAKING_SAFE_DECEL_CM_S2, BRAKING_DISTANCE_MARGIN_CM, BRAKING_CALIBRATION_CONFIRMED);
        if (!plan.valid) { line_follow_stop(f,LINE_STOP_ROUTE); return; }
        navigation_wheel_cap=fminf(navigation_wheel_cap,plan.allowed_speed);
        BrakingDecision decision=braking_decide(speed,planning_entry_speed,plan.remaining_cm,
            -f->braking_accel_cm_s2,BRAKING_SAFE_DECEL_CM_S2,BRAKING_REVERSE_GUARD_CM);
        f->braking_required_cm_s2=decision.required_decel;
        f->braking_reverse_active=false;
        if(decision.distance_exhausted && speed>planning_entry_speed+REVERSE_BRAKE_MARGIN_CM_S) {
            /* Distance is an arming threshold, not a physical stopping boundary.
             * Keep braking toward the entry cap; the next real junction still
             * requires measured entry-speed readiness before any turn. */
            f->route.navigation_warning=ROUTE_FAILURE_BRAKE_DISTANCE;
        }
#if BRAKING_REVERSE_HARDWARE_CONFIRMED && MOTOR_REVERSE_BRAKE_ENABLED
        if(decision.reverse_needed && !decision.distance_exhausted && narrow && stable &&
           f->route.state==ROUTE_FOLLOW &&
           fminf(f->odometry.left_speed_cm_s,f->odometry.right_speed_cm_s)>
               BRAKING_REVERSE_MIN_SPEED_CM_S &&
           f->braking_reverse_ms<BRAKING_REVERSE_MAX_MS) {
            uint32_t frame_ms=(uint32_t)(dt*1000.0f+0.5f);
            if(frame_ms<=BRAKING_REVERSE_MAX_MS-f->braking_reverse_ms)
                f->braking_reverse_active=true;
        }
#endif
        /* 斜坡只跟踪速度包络，不得钳到实际轮目标：目标本身含位置曲线降速，
         * 两者相扣会衰减收敛到 rise/(曲线损失)，pos=1 时约 15 cm/s，车辆无法加速。
         * 包络下降由 braking_slew 的 cap 即时钳低，状态限速在下方 fminf 处理，
         * 上升仍受 next_outer/next_base 的 rise*dt 限制，不会跳变。 */
        base_speed = braking_slew(&f->braking_slew,plan.allowed_speed,dt,
                                   TARGET_RISE_CM_S2,BRAKING_TARGET_JERK_CM_S3);
    }
    else {f->braking_active=false;f->braking_reverse_active=false;}
    if (f->route.search_pending && planning_step.segment_length_cm>0)
        base_speed=fminf(base_speed,planning_entry_speed);
    /* Pose mode: keep exit speed until the outgoing line is re-centred, and
     * pre-slow before the known corner so the brake phase does not eat the
     * pivot approach distance. The window grows with v^2/(2*a_safe). */
    if(f->route.pose_mode) {
        if(f->route.post_turn_slow)
            base_speed=fminf(base_speed,fminf(step.turn_speed,JUNCTION_EXIT_SPEED_CM_S));
        float brake_w=(f->odometry.left_speed_cm_s*f->odometry.left_speed_cm_s)/
            (2.0f*BRAKING_SAFE_DECEL_CM_S2);
        if(f->route.pose_gap_bar_cm<=fmaxf(ROUTE_WAYPOINT_SLOW_CM,brake_w+8.0f))
            base_speed=fminf(base_speed,entry_speed_limit);
    }
    /* Exit handoff stays slow even while previewing the next distance envelope. */
    if(f->route.state==ROUTE_CLEARING && route_current_action(&f->route)!=STRAIGHT)
        base_speed=fminf(base_speed,fminf(step.turn_speed,JUNCTION_EXIT_SPEED_CM_S));
    if (f->route.state==ROUTE_CANDIDATE &&
        route_target_ready(&f->route,f->odometry.distance_cm,true))
        base_speed=fminf(base_speed,entry_speed_limit);
    if (f->route.state == ROUTE_BRAKING) {
        /* A confirmed node transferred loss ownership to the bounded entry
         * phase (zero targets, time/distance/yaw guards). Do not let a stale
         * CANDIDATE ambiguity preempt its abort into navigation recovery. */
        f->abnormal=false;
        pid_reset(&f->pid);
        targets(f,0,0,dt);
        f->state=LINE_JUNCTION;
        if(!speed_control_update(&f->speed,0,0,f->odometry.left_speed_cm_s,
                                 f->odometry.right_speed_cm_s,dt,&f->command))
            line_follow_stop(f,LINE_STOP_FAULT);
        return;
    }
#else
    float base_speed = LINE_FOLLOW_SPEED_CM_S;
#endif
#if PRESET_ROUTE_ENABLED
    if(f->route.pose_mode && f->route.pose_cross_valid &&
       f->route.state==ROUTE_FOLLOW && step.action!=HEXAGON) {
        /* Unified pose path-following: the car is on a known straight segment
         * between two table corners, so track that line directly. The 8-channel
         * reading trims the cross-track estimate only while it agrees with the
         * pose; this survives printed symbols, dead-end short legs and the
         * 24 cm look-ahead leaving the wire at a corner. */
        bool trusted=gray_plausible(f,obs);
        bool usable=trusted && narrow;
        float cross=f->route.pose_cross_cm;
        if(usable) {
            cross=(1.0f-POSE_GRAY_TRIM)*cross+POSE_GRAY_TRIM*(-obs.position*0.55f);
            if(stable) {
                f->have_reliable=true;f->last_reliable_ms=now_ms;
                f->reliable_position=obs.position;
            }
            f->abnormal=false;
        } else if(!f->abnormal && fabsf(f->route.pose_cross_cm)>ROUTE_WAYPOINT_GATE_CM) {
            /* The pose itself says the car is off the segment: from here the
             * grey loss is real, so arm the bounded loss window instead of
             * trusting the path follower. */
            f->abnormal=true;f->abnormal_origin_cm=f->odometry.distance_cm;
        }
        float desired=f->route.pose_seg_heading_rad+atan2f(-cross,POSE_PURSUIT_LOOKAHEAD_CM);
        float corr=HEADING_KP_CM_S_PER_RAD*atan2f(sinf(desired-f->odometry.heading_rad),
                                                  cosf(desired-f->odometry.heading_rad));
        float cap=.5f*fmaxf(base_speed,1.0f);
        corr=fmaxf(-cap,fminf(cap,corr));
        f->state=usable?LINE_TRACKING:LINE_GAP_HOLD;
        pid_reset(&f->pid);
        outer_targets(f,base_speed,corr,dt);
    } else if(f->route.state==ROUTE_CANDIDATE &&
       !route_target_ready(&f->route,f->odometry.distance_cm,true)) {
        /* Intermediate graphics are nodes to pass, not a reason to slow down.
         * Candidate and clearance deadlines still bound missing exit evidence. */
        f->state=LINE_JUNCTION;f->abnormal=false;pid_reset(&f->pid);
        outer_targets(f,base_speed,HEADING_KP_CM_S_PER_RAD*
            (passage_heading(f,f->route.entry_heading)-f->odometry.heading_rad),dt);
    } else if(f->route.state==ROUTE_RECOVERING) {
        f->state=LINE_JUNCTION;f->abnormal=false;
        if(narrow && stable) {
            float correction;
            if(!pid_update(&f->pid,obs.position,dt,NAVIGATION_RECOVERY_SPEED_CM_S*.5f,&correction)) {
                line_follow_stop(f,LINE_STOP_FAULT);return;
            }
            outer_targets(f,NAVIGATION_RECOVERY_SPEED_CM_S,correction,dt);
        } else {
            pid_reset(&f->pid);
            outer_targets(f,NAVIGATION_RECOVERY_SPEED_CM_S,
                HEADING_KP_CM_S_PER_RAD*(f->route.entry_heading-f->odometry.heading_rad)-
                    JUNCTION_YAW_DAMPING*f->turn_yaw_rate_rad_s,dt);
        }
    } else if(f->route.component_recovery_active) {
        f->state=LINE_JUNCTION;f->abnormal=false;f->exit_accelerating=false;
        pid_reset(&f->pid);
        outer_targets(f,f->route.passing_junction?base_speed:fminf(step.turn_speed,COMPONENT_RECOVERY_SPEED_CM_S),
            HEADING_KP_CM_S_PER_RAD*((f->route.passing_junction?
                passage_heading(f,f->route.component_recovery_heading):
                f->route.component_recovery_heading)-f->odometry.heading_rad)-
                JUNCTION_YAW_DAMPING*f->turn_yaw_rate_rad_s,dt);
    } else if(f->route.state==ROUTE_EXECUTING && step.action==HEXAGON) {
        HexagonNavigation *x=&f->route.hexagon;
        f->state=LINE_JUNCTION;f->abnormal=false;f->exit_accelerating=false;
        if(previous_route!=ROUTE_EXECUTING) pid_reset(&f->pid);
        float outer=fminf(step.turn_speed,fminf(HEXAGON_SPEED_CM_S,JUNCTION_SETTLE_SPEED_CM_S));
        float correction;
        if(f->route.turn_approaching) {
            pid_reset(&f->pid);
            straight_targets(f,outer,HEADING_KP_CM_S_PER_RAD*
                (f->route.action_start_heading-f->odometry.heading_rad),dt);
            /* Fall through to the speed controller below.  Returning here
             * skipped speed_control_update and kept the previous BRAKING
             * frame's zero/standby-off command, so the car coasted to a stop
             * and timed the hexagon entry out before reaching the vertex. */
        } else if(x->turning) {
            /* Pose-mode ring pivot to the next 30/60 degree vertex heading. */
            pid_reset(&f->pid);
            float err=atan2f(sinf(x->target_heading-f->odometry.heading_rad),
                             cosf(x->target_heading-f->odometry.heading_rad));
            correction=JUNCTION_SETTLE_HEADING_KP*err-
                JUNCTION_YAW_DAMPING*f->turn_yaw_rate_rad_s;
            correction=fmaxf(-.5f*outer,fminf(.5f*outer,correction));
            outer_targets(f,outer,correction,dt);
        } else if(x->use_line) {
            if(!pid_update(&f->pid,obs.position,dt,.5f*outer,&correction)) {
                line_follow_stop(f,LINE_STOP_FAULT);return;
            }
            correction-=JUNCTION_YAW_DAMPING*f->turn_yaw_rate_rad_s;
            outer_targets(f,outer,correction,dt);
        } else {
            pid_reset(&f->pid);
            correction=HEADING_KP_CM_S_PER_RAD*(x->target_heading-f->odometry.heading_rad)-
                JUNCTION_YAW_DAMPING*f->turn_yaw_rate_rad_s;
            if(f->route.pose_mode) {
                /* Inside-ring legs use the leg cross-track; the entry approach
                 * uses the incoming segment cross-track so a drifted approach
                 * lands on V0 instead of beside it. */
                correction-=JUNCTION_CROSS_KP*(x->cross_cm!=0.0f?x->cross_cm:
                    f->route.pose_cross_cm);
            }
            correction=fmaxf(-.5f*outer,fminf(.5f*outer,correction));
            outer_targets(f,outer,correction,dt);
        }
    } else if (f->route.state == ROUTE_EXECUTING) {
        f->exit_accelerating=false;
        if(previous_route!=ROUTE_EXECUTING) {pid_reset(&f->pid);speed_control_reset(&f->speed);}
        f->abnormal=false; f->state=LINE_JUNCTION;
        float direction=route_current_action(&f->route)==LEFT?-1.0f:1.0f;
        float target=route_turn_target_rad(&f->route);
        float turn=direction*(f->odometry.heading_rad-f->route.entry_heading);

        /* The front sensor can cross the exit line in a few control periods.
         * Request the capture speed from the start of the turn: capping only
         * after the first edge leaves the outer wheel moving too fast to
         * follow the line when it reaches center. No reverse brake assumed. */
        float outer=route_current_action(&f->route)==STRAIGHT ? step.turn_speed :
            fminf(step.turn_speed,JUNCTION_SETTLE_SPEED_CM_S);
        float remaining=fabsf(target)-turn;
        float band=fminf(JUNCTION_SETTLE_BAND_RAD,.4f*fabsf(target));
        bool settling=route_current_action(&f->route)!=STRAIGHT && band>0 && remaining<=band;
        float base=.5f*outer;
        float correction=direction*base;
        if(f->route.turn_approaching) {
            pid_reset(&f->pid);
            straight_targets(f,outer,HEADING_KP_CM_S_PER_RAD*
                (f->route.action_start_heading-f->odometry.heading_rad),dt);
        } else if(route_current_action(&f->route)==STRAIGHT) {
            /* 直行过口按过路速度，不再强制降到转弯速度。 */
            float pass=fminf(base_speed,fmaxf(outer,JUNCTION_PASS_SPEED_CM_S));
            straight_targets(f,pass,HEADING_KP_CM_S_PER_RAD*
                (f->route.entry_heading-f->odometry.heading_rad),dt);
        } else {
            if(f->route.pose_mode) {
                /* Pose-mode pivot: saturated heading feedback towards the exact
                 * corner heading. The grey line is deliberately not used inside
                 * the turn, so a printed symbol cannot pull the pivot. */
                pid_reset(&f->pid);
                float target_heading=f->route.entry_heading+direction*fabsf(target);
                float err=atan2f(sinf(target_heading-f->odometry.heading_rad),
                                 cosf(target_heading-f->odometry.heading_rad));
                float corr;
                if(fabsf(err)>0.15f) corr=err>0.0f?base:-base;
                else {
                    corr=JUNCTION_SETTLE_HEADING_KP*err-1.2f*f->turn_yaw_rate_rad_s;
                    corr=fmaxf(-base,fminf(base,corr));
                }
                (void)base;
                correction=corr; /* the shared turn_targets call below applies it */
            } else if(route_uses_corner_arc(&f->route) && !f->route.corner_arc_complete && !settling &&
               !f->route.capture_line_seen) {
                float track=f->odometry.calibration.track_cm;
                if(CORNER_RADIUS_CM<=track*.5f) {line_follow_stop(f,LINE_STOP_CALIBRATION);return;}
                base=outer/(1+track/(2*CORNER_RADIUS_CM));
                float lead=GRAY_FORWARD_OFFSET_CM-CORNER_RADIUS_CM;
                float planned=fmaxf(0,fminf(fabsf(target),
                    (f->odometry.distance_cm-f->route.entry_cm-lead)/CORNER_RADIUS_CM));
                correction=direction*(base*track/(2*CORNER_RADIUS_CM)+CORNER_HEADING_KP*(planned-turn));
            } else if(f->route.turn_capturing || f->route.capture_line_seen || settling) {
                float line=0;
                if(f->route.turn_capturing) {
                    if(!pid_update(&f->pid,obs.position,dt,base,&line)) {line_follow_stop(f,LINE_STOP_FAULT);return;}
                } else pid_reset(&f->pid);
                float heading_kp=settling?JUNCTION_SETTLE_HEADING_KP:JUNCTION_CAPTURE_HEADING_KP;
                correction=f->route.turn_capturing?line:heading_kp*
                    (f->route.entry_heading+target-f->odometry.heading_rad);
                if(f->route.turn_capturing && fabsf(obs.position)<0.01f &&
                   fabsf(line)<0.05f && turn<fabsf(target))
                    correction=direction*fminf(base,JUNCTION_CAPTURE_CENTER_BIAS_CM_S);
                if(!f->route.turn_capturing && turn<fabsf(target))
                    correction=direction*fmaxf(direction*correction,
                        fminf(base,JUNCTION_CAPTURE_MIN_TURN_CM_S));
                correction-=JUNCTION_YAW_DAMPING*f->turn_yaw_rate_rad_s;
            } else pid_reset(&f->pid);
            turn_targets(f,outer,correction,dt);
        }
    } else if(f->route.state==ROUTE_CLEARING && f->route.next_junction_samples>0) {
        /* Bounded new-graphic confirmation after a proven turn exit: freeze
         * position steering instead of starting the generic ambiguity timer. */
        f->state=LINE_JUNCTION;f->abnormal=false;pid_reset(&f->pid);
        outer_targets(f,fminf(base_speed,JUNCTION_SETTLE_SPEED_CM_S),
            HEADING_KP_CM_S_PER_RAD*(f->route.line_exit_heading_rad-f->odometry.heading_rad),dt);
    } else if (f->route.state==ROUTE_CLEARING && route_current_action(&f->route)==STRAIGHT) {
        f->state=LINE_JUNCTION;
        /* Side edges inside a component graphic are not a straight exit.
         * Route owns the separate no-exit deadline while counting is locked. */
        f->abnormal=false;
        float pass=base_speed;
        if (route_is_clear_line(&f->route,obs) && stable &&
            f->route.exit_count>=LINE_CONFIRM_SAMPLES) {
            /* Only a confirmed centered exit may resume position steering. */
            float line_corr;
            if (!pid_update(&f->pid, obs.position, dt, 0.5f*pass, &line_corr)) {
                line_follow_stop(f, LINE_STOP_FAULT); return;
            }
            outer_targets(f, pass, line_corr, dt);
        } else {
            pid_reset(&f->pid);
            outer_targets(f, pass, HEADING_KP_CM_S_PER_RAD*
                ((f->route.passing_junction?passage_heading(f,f->route.entry_heading):
                  f->route.entry_heading)-f->odometry.heading_rad), dt);
        }
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
            f->component_cross_latched=false;
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
        bool trusted=gray_plausible(f,obs);
        if (trusted) {
            if (!pid_update(&f->pid, obs.position, dt,
                            .5f*next, &correction)) {
                line_follow_stop(f, LINE_STOP_FAULT);
                return;
            }
        } else {
            pid_reset(&f->pid);
            correction=HEADING_KP_CM_S_PER_RAD*
                (f->route.pose_bearing_rad-f->odometry.heading_rad);
            f->state=LINE_GAP_HOLD;
        }
#if PRESET_ROUTE_ENABLED
        if(f->route.state==ROUTE_CLEARING && route_current_action(&f->route)!=STRAIGHT)
            correction-=JUNCTION_YAW_DAMPING*f->turn_yaw_rate_rad_s;
#endif
        outer_targets(f, base, correction, dt);
    } else {
        /* Unconfirmed junctions, wide/multiple regions and no-line all hold heading.
         * Do not integrate graphics into the normal white-line position PID. */
        if (!f->abnormal) {
            f->abnormal = true;
            /* Hold along the known segment, not towards the corner: on a
             * zero-length row the corner is behind the car. */
            f->hold_heading_rad = (f->route.pose_mode && f->route.state==ROUTE_FOLLOW)
                ? (f->route.pose_cross_valid ? f->route.pose_seg_heading_rad
                                            : f->route.pose_bearing_rad)
                : f->odometry.heading_rad;
#if PRESET_ROUTE_ENABLED
            /* Do not lock the already overshot yaw as the new exit direction. */
            if(f->route.state==ROUTE_CLEARING && f->route.line_exit_confirmed &&
               route_current_action(&f->route)!=STRAIGHT) {
                float sign=route_turn_target_rad(&f->route)<0.0f?-1.0f:1.0f;
                f->hold_heading_rad=f->route.line_exit_heading_rad-
                    sign*JUNCTION_EXIT_REWIND_RAD;
            }
#endif
            f->abnormal_origin_cm = f->odometry.distance_cm;
        }
        f->state = LINE_GAP_HOLD;
        /* 完全丢线按保持上限；宽线/多区域歧义按 AMBIGUOUS_SPEED_CM_S 限速盲行。 */
        float base = obs.kind == GRAY_NONE ? fminf(base_speed, HOLD_SPEED_CM_S) :
                                            fminf(base_speed, AMBIGUOUS_SPEED_CM_S);
        float correction=HEADING_KP_CM_S_PER_RAD*(f->hold_heading_rad-f->odometry.heading_rad);
#if PRESET_ROUTE_ENABLED
        /* Pose-mode cross-track hold: a heading-only hold drives parallel to the
         * segment and never comes back, so add the lateral term while the wire is
         * hidden by a printed component or rejected by the plausibility gate. */
        if(f->route.pose_mode && f->route.pose_cross_valid &&
           f->route.state==ROUTE_FOLLOW) {
            correction-=JUNCTION_CROSS_KP*f->route.pose_cross_cm;
            correction=fmaxf(-.5f*base,fminf(.5f*base,correction));
        }
#endif
#if PRESET_ROUTE_ENABLED
        if(f->route.state==ROUTE_CLEARING && f->route.line_exit_confirmed &&
           route_current_action(&f->route)!=STRAIGHT) {
            correction-=JUNCTION_YAW_DAMPING*f->turn_yaw_rate_rad_s;
            outer_targets(f,base,correction,dt);
        } else
#endif
            targets(f,base,correction,dt);
    }
#if PRESET_ROUTE_ENABLED
    /* Apply approach/passage caps after every mixer, including GAP_HOLD's
     * additive heading correction. Scale both wheels to retain steering. */
    float peak_target=fmaxf(f->left_target_cm_s,f->right_target_cm_s);
    if(peak_target>navigation_wheel_cap) {
        float ratio=navigation_wheel_cap/peak_target;
        f->left_target_cm_s*=ratio;f->right_target_cm_s*=ratio;
        f->base=.5f*(f->left_target_cm_s+f->right_target_cm_s);
        f->correction=.5f*(f->left_target_cm_s-f->right_target_cm_s);
    }
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
       f->route.turn_capturing && narrow) {
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
    if(f->state!=LINE_TRACKING ||
       fminf(f->odometry.left_speed_cm_s-f->left_target_cm_s,
             f->odometry.right_speed_cm_s-f->right_target_cm_s)<=REVERSE_BRAKE_MARGIN_CM_S)
        f->braking_reverse_active=false;
    if(f->braking_reverse_active)
        f->braking_reverse_ms+=(uint32_t)(dt*1000.0f+0.5f);
    if (!speed_control_update_braking(&f->speed, f->left_target_cm_s, f->right_target_cm_s,
                              f->odometry.left_speed_cm_s, f->odometry.right_speed_cm_s,
                              dt,
#if PRESET_ROUTE_ENABLED
                              f->braking_reverse_active && f->state==LINE_TRACKING && narrow,
#else
                              false,
#endif
                              &f->command)) line_follow_stop(f, LINE_STOP_ENCODER);
#if PRESET_ROUTE_ENABLED
#endif
#endif
}

void line_follow_update_heading(LineFollow *f, GrayObservation obs, EncoderCounts counts,
                        uint32_t now_ms, float dt, bool start_request,
                        bool imu_valid, float imu_heading_rad)
{
#if PRESET_ROUTE_ENABLED
    LineState previous=f->state;
    RouteState before=f->route.state;
#endif
    line_follow_update_heading_impl(f,obs,counts,now_ms,dt,start_request,imu_valid,imu_heading_rad);
#if PRESET_ROUTE_ENABLED
    /* Include the stopping frame, then preserve evidence while stopped/idle.
     * START clears the ring only after the normal acceptance gates succeed. */
    if(previous==LINE_TRACKING || previous==LINE_GAP_HOLD || previous==LINE_JUNCTION ||
       f->state==LINE_TRACKING || f->state==LINE_GAP_HOLD || f->state==LINE_JUNCTION) {
        NavigationTrace *trace=&f->navigation_trace;
        trace->samples[trace->next]=(NavigationTraceSample){
            .ms=now_ms,.dt_s=dt,.heading_rad=f->odometry.heading_rad,
            .distance_cm=f->odometry.distance_cm,
            .left_speed_cm_s=f->odometry.left_speed_cm_s,.right_speed_cm_s=f->odometry.right_speed_cm_s,
            .route_index=(uint32_t)f->route.index,.junctions_seen=f->route.junctions_seen,
            .counts=counts,.raw_mask=obs.raw_mask,.line_mask=obs.line_mask,
            .prefilter_mask=obs.prefilter_mask,.observation=(uint8_t)obs.kind,
            .route_before=(uint8_t)before,.route_after=(uint8_t)f->route.state,
            .evidence=(uint8_t)f->route.candidate_kind,.sides=f->route.candidate_sides,
            .rejection=(uint8_t)f->route.candidate_reject,.heading_source=(uint8_t)f->heading_source,
            .candidate_samples=f->route.candidate_count,.stop_reason=(uint8_t)f->stop_reason
        };
        trace->next=(uint16_t)((trace->next+1U)%NAVIGATION_TRACE_CAPACITY);
        if(trace->count<NAVIGATION_TRACE_CAPACITY) ++trace->count;
    }
#endif
}

/* A grey reading is trusted only when its lateral offset agrees with the pose's
 * offset from the active segment line (+-4 weight units, ~2.2 cm). A printed
 * transistor branch or resistor arm reads like a line but sits where the pose
 * says the car is centred, so it is rejected and the pose takes over. */
static LINE_FOLLOW_MAYBE_UNUSED inline bool gray_plausible(LineFollow *f,GrayObservation obs)
{
#if PRESET_ROUTE_ENABLED
    if(!f->route.pose_mode || !f->route.pose_cross_valid) return true;
    /* A centred narrow line is unambiguous: after a fast pivot the pose can be
     * several cm off, and rejecting this reading would leave the car without
     * the very line it needs to recover on. */
    if(route_is_narrow_line(obs) && fabsf(obs.position)<=JUNCTION_CENTER_LIMIT) return true;
    float expected=-f->route.pose_cross_cm/0.55f;
    return fabsf(obs.position-expected)<=JUNCTION_POSE_GRAY_SPREAD;
#else
    (void)f;(void)obs;return true;
#endif
}

void line_follow_update(LineFollow *f, GrayObservation obs, EncoderCounts counts,
                        uint32_t now_ms, float dt, bool start_request)
{
    line_follow_update_heading(f, obs, counts, now_ms, dt, start_request, false, 0.0f);
}
