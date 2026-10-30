#ifndef TEST_STM32_HAL_H
#define TEST_STM32_HAL_H

/* Minimal host HAL surrogate for motor.c / encoder.c. Never included in firmware. */
#include <stdint.h>
typedef struct { uint16_t output; } GPIO_TypeDef;
typedef struct { uint32_t compare[2], counter; } TIM_TypeDef;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
typedef struct {
    uint32_t ClockSpeed, DutyCycle, OwnAddress1, AddressingMode, DualAddressMode;
    uint32_t OwnAddress2, GeneralCallMode, NoStretchMode;
} I2C_InitTypeDef;
typedef struct { void *Instance; I2C_InitTypeDef Init; } I2C_HandleTypeDef;
#define I2C1 ((void *)1)
#define I2C_MEMADD_SIZE_8BIT 1U
#define I2C_DUTYCYCLE_2 0U
#define I2C_ADDRESSINGMODE_7BIT 0U
#define I2C_DUALADDRESS_DISABLE 0U
#define I2C_GENERALCALL_DISABLE 0U
#define I2C_NOSTRETCH_DISABLE 0U
#define GPIO_MODE_AF_OD 3U
#define __HAL_RCC_AFIO_CLK_ENABLE() ((void)0)
#define __HAL_RCC_I2C1_CLK_ENABLE() ((void)0)
#define __HAL_AFIO_REMAP_I2C1_ENABLE() ((void)0)
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *i2c);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *i2c, uint16_t address, uint16_t reg,
                                uint16_t size, uint8_t *data, uint16_t count, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *i2c, uint16_t address, uint16_t reg,
                                 uint16_t size, uint8_t *data, uint16_t count, uint32_t timeout);
void HAL_Delay(uint32_t ms);
typedef struct { uint32_t Prescaler, CounterMode, Period, ClockDivision; } TIM_Base_InitTypeDef;
typedef struct { TIM_TypeDef *Instance; TIM_Base_InitTypeDef Init; } TIM_HandleTypeDef;
typedef struct { uint32_t OCMode, Pulse, OCPolarity, OCFastMode; } TIM_OC_InitTypeDef;
typedef struct {
    uint32_t EncoderMode, IC1Polarity, IC1Selection, IC1Prescaler, IC1Filter;
    uint32_t IC2Polarity, IC2Selection, IC2Prescaler, IC2Filter;
} TIM_Encoder_InitTypeDef;
extern GPIO_TypeDef fake_gpio_a, fake_gpio_b;
extern TIM_TypeDef fake_tim3;
extern TIM_TypeDef fake_tim1, fake_tim4;
#define GPIOA (&fake_gpio_a)
#define GPIOB (&fake_gpio_b)
#define TIM3 (&fake_tim3)
#define TIM1 (&fake_tim1)
#define TIM4 (&fake_tim4)
#define GPIO_PIN_0 (1U << 0)
#define GPIO_PIN_1 (1U << 1)
#define GPIO_PIN_2 (1U << 2)
#define GPIO_PIN_3 (1U << 3)
#define GPIO_PIN_4 (1U << 4)
#define GPIO_PIN_5 (1U << 5)
#define GPIO_PIN_6 (1U << 6)
#define GPIO_PIN_7 (1U << 7)
#define GPIO_PIN_8 (1U << 8)
#define GPIO_PIN_9 (1U << 9)
#define GPIO_PIN_10 (1U << 10)
#define GPIO_PIN_12 (1U << 12)
#define GPIO_PIN_13 (1U << 13)
#define GPIO_PIN_14 (1U << 14)
#define GPIO_PIN_15 (1U << 15)
#define GPIO_PULLUP 1U
#define GPIO_NOPULL 0U
#define GPIO_MODE_INPUT 0U
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
#define TIM_CHANNEL_ALL 0x3cU
#define TIM_ENCODERMODE_TI12 3U
#define TIM_ICPOLARITY_RISING 0U
#define TIM_ICSELECTION_DIRECTTI 1U
#define TIM_ICPSC_DIV1 0U
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM3_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM1_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM4_CLK_ENABLE() ((void)0)
#define __HAL_TIM_SET_COMPARE(timer, channel, value) ((timer)->Instance->compare[channel] = (value))
#define __HAL_TIM_SET_COUNTER(timer, value) ((timer)->Instance->counter = (value))
#define __HAL_TIM_GET_COUNTER(timer) ((timer)->Instance->counter)
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pins, GPIO_PinState state);
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *pins);
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *timer);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *timer, TIM_OC_InitTypeDef *oc, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Init(TIM_HandleTypeDef *timer, const TIM_Encoder_InitTypeDef *config);
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *timer, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *timer, uint32_t channel);

#endif
