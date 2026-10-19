#include "encoder.h"
#include "board_config.h"
#include "motion_config.h"

static TIM_HandleTypeDef left_timer, right_timer;
static bool ready;

void encoder_gpio_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef pins = {0};
    pins.Mode = GPIO_MODE_INPUT;
    pins.Pull = ENCODER_INPUT_PULL;
    pins.Pin = ENCODER_LEFT_A_PIN | ENCODER_LEFT_B_PIN;
    HAL_GPIO_Init(ENCODER_LEFT_PORT, &pins);
    pins.Pin = ENCODER_RIGHT_A_PIN | ENCODER_RIGHT_B_PIN;
    HAL_GPIO_Init(ENCODER_RIGHT_PORT, &pins);
}

static bool timer_init(TIM_HandleTypeDef *timer, TIM_TypeDef *instance)
{
    timer->Instance = instance;
    timer->Init.Prescaler = 0;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Period = 65535U;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM_Encoder_InitTypeDef config = {0};
    config.EncoderMode = TIM_ENCODERMODE_TI12;
    config.IC1Polarity = config.IC2Polarity = TIM_ICPOLARITY_RISING;
    config.IC1Selection = config.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    config.IC1Prescaler = config.IC2Prescaler = TIM_ICPSC_DIV1;
    config.IC1Filter = config.IC2Filter = ENCODER_INPUT_FILTER;
    return HAL_TIM_Encoder_Init(timer, &config) == HAL_OK;
}

bool encoder_init(void)
{
    ready = false;
    encoder_gpio_init();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    if (!timer_init(&left_timer, TIM1) || !timer_init(&right_timer, TIM4)) return false;
    __HAL_TIM_SET_COUNTER(&left_timer, 0);
    __HAL_TIM_SET_COUNTER(&right_timer, 0);
    if (HAL_TIM_Encoder_Start(&left_timer, TIM_CHANNEL_ALL) != HAL_OK) return false;
    if (HAL_TIM_Encoder_Start(&right_timer, TIM_CHANNEL_ALL) != HAL_OK) {
        (void)HAL_TIM_Encoder_Stop(&left_timer, TIM_CHANNEL_ALL);
        return false;
    }
    ready = true;
    return true;
}

bool encoder_read(EncoderCounts *counts)
{
    if (!ready || counts == 0) return false;
    /* Two sequential hardware snapshots; not exactly simultaneous. */
    counts->left = (uint16_t)__HAL_TIM_GET_COUNTER(&left_timer);
    counts->right = (uint16_t)__HAL_TIM_GET_COUNTER(&right_timer);
    return true;
}
