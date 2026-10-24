#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H
#include "odometry.h"
#include "route.h"
#include "speed_control.h"

typedef enum { LINE_IDLE, LINE_TRACKING, LINE_GAP_HOLD, LINE_JUNCTION, LINE_STOPPED } LineState;
typedef enum {
    LINE_STOP_NONE, LINE_STOP_REQUEST, LINE_STOP_LOST, LINE_STOP_FAULT,
    LINE_STOP_ENCODER, LINE_STOP_CALIBRATION, LINE_STOP_ROUTE, LINE_STOP_ROUTE_COMPLETE
} LineStopReason;
/* 最近一次启动请求的结果；保留到下一次请求，避免白线门禁静默拒绝。 */
typedef enum {
    LINE_START_NONE, LINE_START_ACCEPTED, LINE_START_REJECTED_LINE,
    LINE_START_REJECTED_CALIBRATION, LINE_START_REJECTED_ROUTE,
    LINE_START_REJECTED_FAULT, LINE_START_ALREADY_RUNNING
} LineStartResult;
/* 第一次停车时在清零输出之前锁存；正常采样不覆盖，明确 START 接受后清空。
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
    float dt_s, base; /* base 在编码器模式为 cm/s，普通 PWM 模式为 PWM 指令。 */
    float left_target_cm_s, right_target_cm_s;
    float left_speed_cm_s, right_speed_cm_s;
    float distance_cm, route_progress_cm, clear_progress_cm, turn_rad;
    uint32_t exit_candidate_age_ms;
    float exit_candidate_progress_cm;
    bool exit_candidate_pending;
    uint8_t exit_confirm_samples;
    bool handoff_early;
    float segment_start_progress_cm;
    bool turn_exit_branch;
    uint8_t exit_line_mask;
} LineStopSnapshot;
typedef struct {
    LineState state;
    LineStopReason stop_reason;
    LineStartResult start_result;
    Odometry odometry;
    RoutePlanner route;
    SpeedControl speed;
    Pid pid; /* White-line position: PWM difference when open, cm/s when closed. */
    MotorCommand command; /* Final PWM output in either mode. */
    bool have_reliable, abnormal;
    uint8_t valid_count;
    float reliable_position, base, correction, left_target_cm_s, right_target_cm_s;
    float hold_heading_rad, abnormal_origin_cm;
    float held_pwm_correction; /* Open-loop interference hold, not measured heading. */
    float startup_elapsed_s; /* 每次明确 START 重新计时，两种模式均使用 1 s 启动包络。 */
    uint32_t last_reliable_ms;
    uint32_t sample_ms;
    float sample_dt_s;
    GrayObservation observation;
    EncoderCounts sample_counts;
    RouteState route_state_before_update;
    LineStopSnapshot stop_snapshot;
} LineFollow;
void line_follow_init(LineFollow *follow, OdometryCalibration calibration,
                       const RouteStep *steps, size_t count,
                       RouteStopCallback callback, void *context);
void line_follow_stop(LineFollow *follow, LineStopReason reason);
void line_follow_update(LineFollow *follow, GrayObservation observation, EncoderCounts counts,
                        uint32_t now_ms, float dt, bool start_request);
bool line_follow_have_recent_position(const LineFollow *follow, uint32_t now_ms);
#endif
