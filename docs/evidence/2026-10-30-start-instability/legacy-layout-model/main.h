#ifndef MAIN_H
#define MAIN_H

#include "line_follow.h"
#include "stm32f1xx_hal.h"

typedef struct {
    uint32_t control_count, last_sample_ms, task_duration_ms, interval_ms;
    uint8_t raw_mask, line_mask, selected_mask;
    uint8_t prefilter_mask, seam_mask;
    GrayKind observation;
    LineState state;
    LineStopReason stop_reason;
    uint32_t button_press_count, start_request_count;
    LineStartResult start_result;
    uint8_t valid_line_samples, start_raw_mask, start_line_mask;
    float position, left_command, right_command;
    uint16_t left_encoder, right_encoder;
    bool encoder_control_enabled, preset_route_enabled, odometry_ready;
    bool imu_enabled, imu_ready;
    int16_t imu_gyro_z_raw;
    float imu_gyro_z_bias_lsb;
    uint8_t imu_init_failure_stage, imu_address_7bit;
    bool pitch_stop_enabled, pitch_stop_latched;
    float pitch_deg, pitch_over_threshold_ms;
    float distance_cm, heading_rad, left_speed_cm_s, right_speed_cm_s;
    float left_target_cm_s, right_target_cm_s;
    RouteState route_state;
    RouteFailure route_failure;
    float route_progress_cm;
    uint32_t route_elapsed_ms; /* 仅诊断用，不参与路段停车判断。 */
    uint32_t route_repeat_count, route_completed_laps;
    bool route_turn_capturing, route_turn_forced;
    uint8_t route_candidate_samples;
    float route_turn_rad;
    uint32_t route_index, route_stop_events, route_stopped_step;
    LineStopSnapshot stop_snapshot; /* 当前停车快照；接受 START/自动恢复时清空。 */
    LineStopSnapshot auto_resume_snapshot; /* 最近一次自动恢复前的快照。 */
    uint32_t auto_resume_count, auto_resume_last_ms;
    uint16_t route_junctions_seen;
    bool route_final_segment_active;
    float route_final_segment_progress_cm;
    bool route_recovery_window_active;
    float route_recovery_window_progress_cm, route_recovery_window_limit_cm;
    StraightEntrySnapshot straight_entry_aligned, straight_entry_handoff;
    TurnExitDiagnostics turn_exit[2];
    bool exit_anchor_valid, exit_recovery_active;
    float exit_lateral_error_cm;
    float imu_gyro_z_calibration_span_lsb;
    float imu_gyro_z_calibration_central_span_lsb;
    uint32_t imu_calibration_attempts;
    uint32_t imu_init_attempts;
    HeadingSource heading_source;
    HeadingFallbackReason heading_fallback_reason;
    uint32_t heading_fallback_ms;
    bool pitch_monitor_available;
} ControlDiagnostics;

/* Debugger access only. 0 = none, 1 = one-shot start, 2 = stop. No ISR writes. */
extern volatile uint32_t g_control_request;
extern volatile ControlDiagnostics g_diagnostics;
void Error_Handler(void);
void route_stop_callback(size_t step_index, void *context);

#endif
