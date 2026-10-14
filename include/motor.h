#ifndef MOTOR_H
#define MOTOR_H

#include "motor_command.h"
void motor_safe_gpio_init(void);
bool motor_init(void);
void motor_stop(void);
void motor_apply(MotorCommand command);

#endif
