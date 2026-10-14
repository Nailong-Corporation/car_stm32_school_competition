#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "stm32f1xx_hal.h"

/* S0..S7 viewed from behind the car looking in its forward direction. */
#define GRAY_A_PINS (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5)
#define GRAY_B_PINS (GPIO_PIN_0 | GPIO_PIN_1)
#define GRAY_INPUT_PULL GPIO_PULLUP
#define MOTOR_DIRECTION_PORT GPIOB
#define MOTOR_LEFT_IN1 GPIO_PIN_12
#define MOTOR_LEFT_IN2 GPIO_PIN_13
#define MOTOR_RIGHT_IN1 GPIO_PIN_14
#define MOTOR_RIGHT_IN2 GPIO_PIN_15
#define MOTOR_STBY_PORT GPIOB
#define MOTOR_STBY_PIN GPIO_PIN_5
/* Change only after suspended-wheel forward-direction verification. */
#define MOTOR_LEFT_FORWARD_IN1_HIGH 1
#define MOTOR_RIGHT_FORWARD_IN1_HIGH 1
#define MOTOR_PWM_PORT GPIOA
#define MOTOR_PWM_PINS (GPIO_PIN_6 | GPIO_PIN_7)
#define MOTOR_LEFT_CHANNEL TIM_CHANNEL_1
#define MOTOR_RIGHT_CHANNEL TIM_CHANNEL_2
#define MOTOR_PWM_HZ 10000U
#define BOARD_SYSCLK_HZ 64000000U
#define BOARD_PWM_PERIOD (BOARD_SYSCLK_HZ / MOTOR_PWM_HZ - 1U)
#define START_STOP_PORT GPIOB
#define START_STOP_PIN GPIO_PIN_10
/* PB10 button to GND; internal pull-up. SWD PA13/PA14 remains enabled. */

#endif
