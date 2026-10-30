#ifndef MOTOR_H
#define MOTOR_H

#include "motor_command.h"
void motor_safe_gpio_init(void);
bool motor_init(void);
void motor_stop(void);
/* Explicit bench-only short-brake output; caller must impose a bounded duration. */
void motor_short_brake(void);
void motor_apply(MotorCommand command);

#endif
