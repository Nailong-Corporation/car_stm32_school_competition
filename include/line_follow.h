#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H
#include "odometry.h"
#include "braking_planner.h"
#include "route.h"
#include "speed_control.h"

typedef enum { LINE_IDLE, LINE_TRACKING, LINE_GAP_HOLD, LINE_JUNCTION, LINE_STOPPED } LineState;
typedef enum { HEADING_ENCODER, HEADING_IMU } HeadingSource;
typedef enum { HEADING_FALLBACK_NONE, HEADING_FALLBACK_STARTUP,
               HEADING_FALLBACK_DATA } HeadingFallbackReason;
typedef enum {
    LINE_STOP_NONE, LINE_STOP_REQUEST, LINE_STOP_LOST, LINE_STOP_FAULT,
    LINE_STOP_ENCODER, LINE_STOP_CALIBRATION, LINE_STOP_ROUTE, LINE_STOP_ROUTE_COMPLETE,
    LINE_STOP_IMU, LINE_STOP_TILT
} LineStopReason;
/* 最近一次启动请求或自动恢复的结果；避免启动门禁静默拒绝。 */
typedef enum {
    LINE_START_NONE, LINE_START_ACCEPTED, LINE_START_REJECTED_LINE,
    LINE_START_REJECTED_CALIBRATION, LINE_START_REJECTED_ROUTE,
    LINE_START_REJECTED_FAULT, LINE_START_ALREADY_RUNNING, LINE_START_AUTO_RESUMED
} LineStartResult;
/* 每次人工 START 后首次圆弧接 STRAIGHT 的对齐/交接证据，后续停车不会覆盖。 */
typedef struct {
    bool valid, pid_active;
    uint32_t sample_ms, route_index;
    RouteState route_state;
    GrayObservation observation;
    float heading_rad, target_heading_rad, outgoing_cm;
    float left_target_cm_s, right_target_cm_s;
} StraightEntrySnapshot;
/* 停车时在清零输出之前锁存；正常采样不覆盖，START/自动恢复接受后清空。
 * 只保存在 RAM，复位/断电后丢失；数据用于停稳后读取，不要求运行中断点。 */
typedef struct {
    bool valid;
    LineStopReason reason;
    LineState state_before_stop;
    RouteState route_state, route_state_before_update;
    RouteFailure route_failure;
    SpeedFault speed_fault;
    uint8_t speed_fault_wheel;
    GrayObservation observation;
    EncoderCounts counts;
    uint32_t sample_ms, reliable_age_ms, route_phase_age_ms;
    uint32_t route_index;
    uint16_t junctions_seen;
    bool final_segment_active;
    float final_segment_progress_cm;
    float dt_s, base; /* base 在编码器模式为 cm/s，普通 PWM 模式为 PWM 指令。 */
    float left_target_cm_s, right_target_cm_s;
    float left_speed_cm_s, right_speed_cm_s;
    float distance_cm, route_progress_cm, clear_progress_cm, turn_rad;
    float actual_turn_rad; /* 实际动作起点以来的转角；turn_rad 相对规划入口轴。 */
    uint32_t exit_candidate_age_ms;
    float exit_candidate_progress_cm;
    bool exit_candidate_pending;
    uint8_t exit_confirm_samples;
    bool handoff_early;
    float segment_start_progress_cm;
    bool turn_exit_branch;
    uint8_t exit_line_mask;
    bool exit_recovery_active;
    float exit_lateral_error_cm;
    uint32_t component_recovery_age_ms;
    float component_recovery_progress_cm;
    HeadingSource heading_source;
    HeadingFallbackReason heading_fallback_reason;
    uint32_t heading_fallback_ms;
} LineStopSnapshot;
typedef struct {
    bool valid, handed_off;
    bool capture_seen;
    bool line_confirmed_exit;
    uint32_t capture_ms;
    float capture_turn_rad, capture_left_target_cm_s, capture_right_target_cm_s;
    uint8_t capture_mask;
    uint32_t clearing_ms, handoff_ms;
    float clearing_turn_rad, minimum_turn_rad, handoff_turn_rad;
    uint8_t clearing_mask, handoff_mask;
} TurnExitDiagnostics;
/* 最近的转弯采样，停车后保留；20 ms 间隔，循环覆盖。 */
#define TURN_TRACE_CAPACITY 48U
typedef struct {
    uint32_t ms;
    float distance_cm, turn_rad, position;
    float left_target, right_target, left_speed, right_speed;
    uint8_t mask, kind, route_state, capturing;
} TurnTraceSample;
typedef struct {
    TurnTraceSample samples[TURN_TRACE_CAPACITY];
    uint32_t last_ms;
    uint8_t next, count;
} TurnTrace;
/* Full control-period input/output-state record; freezes at first stop.
 * Raw GPIO enables offline replay through the actual sensor filter. */
#define NAVIGATION_TRACE_CAPACITY 128U
typedef struct {
    uint32_t ms;
    float dt_s, heading_rad, distance_cm, left_speed_cm_s, right_speed_cm_s;
    uint32_t route_index;
    uint16_t junctions_seen;
    EncoderCounts counts;
    uint8_t raw_mask, line_mask, prefilter_mask, observation;
    uint8_t route_before, route_after, evidence, sides, rejection, heading_source;
    uint8_t candidate_samples, stop_reason;
} NavigationTraceSample;
typedef struct {
    NavigationTraceSample samples[NAVIGATION_TRACE_CAPACITY];
    uint16_t next, count;
} NavigationTrace;
typedef struct {
    LineState state;
    LineStopReason stop_reason;
    LineStartResult start_result;
    Odometry odometry;
    RoutePlanner route;
    SpeedControl speed;
    BrakingSlew braking_slew;
    bool braking_active;
    float braking_origin_cm;
    Pid pid; /* White-line position: PWM difference when open, cm/s when closed. */
    MotorCommand command; /* Final PWM output in either mode. */
    bool have_reliable, abnormal;
    uint8_t valid_count;
    float reliable_position, base, correction, left_target_cm_s, right_target_cm_s;
    float hold_heading_rad, abnormal_origin_cm;
    float held_pwm_correction; /* Open-loop interference hold, not measured heading. */
    float turn_yaw_rate_rad_s; /* 当前航向来源实际增量 / dt 的低通值，右转正。 */
    bool exit_anchor_valid, exit_recovery_active;
    float exit_anchor_x_cm, exit_anchor_y_cm, exit_axis_rad, exit_lateral_error_cm;
    bool exit_accelerating; /* 低速离口完成后快速加速，达到规划目标后清除。 */
    float startup_elapsed_s; /* 每次明确 START 重新计时，两种模式均使用 1 s 启动包络。 */
    uint32_t last_reliable_ms;
    uint32_t sample_ms;
    float sample_dt_s;
    GrayObservation observation;
    EncoderCounts sample_counts;
    RouteState route_state_before_update;
    LineStopSnapshot stop_snapshot;
    LineStopSnapshot auto_resume_snapshot; /* 上次自动恢复前的停车证据。 */
    uint32_t auto_resume_count, auto_resume_last_ms;
    StraightEntrySnapshot straight_entry_aligned, straight_entry_handoff;
    TurnExitDiagnostics turn_exit[2]; /* 前两个弯的离口角度记录，START清零。 */
    TurnTrace turn_trace; /* 接受人工 START 清零；停止后读取 follow.turn_trace。 */
    HeadingSource heading_source;
    HeadingFallbackReason heading_fallback_reason;
    uint32_t heading_fallback_ms; /* Sticky until line_follow_init / MCU reset. */
    NavigationTrace navigation_trace;
} LineFollow;
void line_follow_init(LineFollow *follow, OdometryCalibration calibration,
                       const RouteStep *steps, size_t count,
                       RouteStopCallback callback, void *context);
void line_follow_stop(LineFollow *follow, LineStopReason reason);
/* Caller has a fresh level IMU sample and an explicit START; preserves snapshot
 * until START is actually accepted. Other fault stops cannot be cleared here. */
void line_follow_acknowledge_tilt(LineFollow *follow);
void line_follow_update(LineFollow *follow, GrayObservation observation, EncoderCounts counts,
                        uint32_t now_ms, float dt, bool start_request);
/* IMU 首个无效航向锁定降级编码器，从当前航向继续积分；不自动切回 IMU。 */
void line_follow_update_heading(LineFollow *follow, GrayObservation observation,
                        EncoderCounts counts, uint32_t now_ms, float dt, bool start_request,
                        bool imu_valid, float imu_heading_rad);
bool line_follow_have_recent_position(const LineFollow *follow, uint32_t now_ms);
#endif
