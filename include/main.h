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
    float position, left_command, right_command;
} ControlDiagnostics;

/* Debugger access only. 0 = none, 1 = one-shot start, 2 = stop. No ISR writes. */
extern volatile uint32_t g_control_request;
extern volatile ControlDiagnostics g_diagnostics;
void Error_Handler(void);

#endif
