#ifndef MOTOR_COMMAND_H
#define MOTOR_COMMAND_H

#include <stdbool.h>
#include <stdint.h>

/* PWM uses the 255 scale; negative values are reserved for enabled encoder reverse braking. */
typedef struct { float left, right; bool enabled; } MotorCommand;
MotorCommand motor_mix(float base, float correction);
uint16_t motor_command_to_ccr(float command, float gain, float deadzone, uint16_t period);

#endif
