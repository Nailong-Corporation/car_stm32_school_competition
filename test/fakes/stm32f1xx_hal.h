#ifndef TEST_STM32_HAL_H
#define TEST_STM32_HAL_H

/* Minimal host HAL surrogate for motor.c only. Never included in firmware. */
#include <stdint.h>
typedef struct { uint16_t output; } GPIO_TypeDef;
typedef struct { uint32_t compare[2]; } TIM_TypeDef;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
typedef struct { uint32_t Prescaler, CounterMode, Period, ClockDivision; } TIM_Base_InitTypeDef;
typedef struct { TIM_TypeDef *Instance; TIM_Base_InitTypeDef Init; } TIM_HandleTypeDef;
typedef struct { uint32_t OCMode, Pulse, OCPolarity, OCFastMode; } TIM_OC_InitTypeDef;
extern GPIO_TypeDef fake_gpio_a, fake_gpio_b;
extern TIM_TypeDef fake_tim3;
#define GPIOA (&fake_gpio_a)
#define GPIOB (&fake_gpio_b)
#define TIM3 (&fake_tim3)
#define GPIO_PIN_0 (1U << 0)
#define GPIO_PIN_1 (1U << 1)
#define GPIO_PIN_2 (1U << 2)
#define GPIO_PIN_3 (1U << 3)
#define GPIO_PIN_4 (1U << 4)
#define GPIO_PIN_5 (1U << 5)
#define GPIO_PIN_6 (1U << 6)
#define GPIO_PIN_7 (1U << 7)
#define GPIO_PIN_10 (1U << 10)
#define GPIO_PIN_12 (1U << 12)
#define GPIO_PIN_13 (1U << 13)
#define GPIO_PIN_14 (1U << 14)
#define GPIO_PIN_15 (1U << 15)
#define GPIO_PULLUP 1U
#define GPIO_MODE_OUTPUT_PP 1U
#define GPIO_MODE_AF_PP 2U
#define GPIO_SPEED_FREQ_LOW 1U
#define GPIO_SPEED_FREQ_HIGH 2U
#define TIM_COUNTERMODE_UP 0U
#define TIM_CLOCKDIVISION_DIV1 0U
#define TIM_OCMODE_PWM1 1U
#define TIM_OCPOLARITY_HIGH 1U
#define TIM_OCFAST_DISABLE 0U
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 1U
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM3_CLK_ENABLE() ((void)0)
#define __HAL_TIM_SET_COMPARE(timer, channel, value) ((timer)->Instance->compare[channel] = (value))
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pins, GPIO_PinState state);
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *pins);
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *timer);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *timer, TIM_OC_InitTypeDef *oc, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel);

#endif
