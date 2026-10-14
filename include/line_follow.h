#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include "gray_sensor.h"
#include "motor_command.h"
#include "pid.h"

typedef enum { LINE_IDLE, LINE_TRACKING, LINE_GAP_HOLD, LINE_SEARCH, LINE_STOPPED } LineState;
typedef enum {
    LINE_STOP_NONE, LINE_STOP_REQUEST, LINE_STOP_LOST,
    LINE_STOP_AMBIGUOUS, LINE_STOP_NO_DIRECTION, LINE_STOP_FAULT
} LineStopReason;
typedef struct {
    LineState state;
    LineStopReason stop_reason;
    Pid pid;
    MotorCommand command;
    bool have_reliable, abnormal, ambiguity_seen;
    uint8_t valid_count;
    float reliable_position, correction, held_correction, base;
    int8_t direction;
    uint32_t last_reliable_ms, direction_ms;
} LineFollow;

void line_follow_init(LineFollow *follow);
void line_follow_stop(LineFollow *follow, LineStopReason reason);
void line_follow_update(LineFollow *follow, GrayObservation observation,
                        uint32_t now_ms, float dt, bool start_request);
bool line_follow_have_recent_position(const LineFollow *follow, uint32_t now_ms);

#endif
