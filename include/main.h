#ifndef MAIN_H
#define MAIN_H

#include "line_follow.h"
#include "stm32f1xx_hal.h"

typedef struct {
    uint32_t control_count, last_sample_ms, task_duration_ms, interval_ms;
    uint8_t raw_mask, line_mask, selected_mask;
    GrayKind observation;
    LineState state;
    LineStopReason stop_reason;
    uint32_t button_press_count, start_request_count;
    LineStartResult start_result;
    uint8_t valid_line_samples, start_raw_mask, start_line_mask;
    float position, left_command, right_command;
    uint16_t left_encoder, right_encoder;
    bool encoder_control_enabled, preset_route_enabled, odometry_ready;
    float distance_cm, heading_rad, left_speed_cm_s, right_speed_cm_s;
    float left_target_cm_s, right_target_cm_s;
    RouteState route_state;
    RouteFailure route_failure;
    float route_progress_cm;
    uint32_t route_elapsed_ms; /* 仅诊断用，不参与路段停车判断。 */
    uint32_t route_index, route_stop_events, route_stopped_step;
} ControlDiagnostics;

/* Debugger access only. 0 = none, 1 = one-shot start, 2 = stop. No ISR writes. */
extern volatile uint32_t g_control_request;
extern volatile ControlDiagnostics g_diagnostics;
void Error_Handler(void);
void route_stop_callback(size_t step_index, void *context);

#endif
