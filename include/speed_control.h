#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H
#include "motor_command.h"
#include "pid.h"
typedef struct {
    Pid left, right;
    float left_stall_s, right_stall_s;
} SpeedControl;
void speed_control_init(SpeedControl *speed);
void speed_control_reset(SpeedControl *speed);
bool speed_control_update(SpeedControl *speed, float left_target, float right_target,
                          float left_measured, float right_measured, float dt,
                          MotorCommand *command);
#endif
