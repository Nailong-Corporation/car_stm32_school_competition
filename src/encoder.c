#include "encoder.h"
#include "board_config.h"

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
