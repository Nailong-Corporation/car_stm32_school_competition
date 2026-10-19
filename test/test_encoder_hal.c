#include "encoder.h"
#include "board_config.h"
#include "motion_config.h"
#include <assert.h>
#include <stdio.h>

GPIO_TypeDef fake_gpio_a, fake_gpio_b;
TIM_TypeDef fake_tim1, fake_tim3, fake_tim4;
static unsigned calls, fail_at, gpio_calls, left_started, right_started, stops;
static HAL_StatusTypeDef result(void) { return ++calls == fail_at ? HAL_ERROR : HAL_OK; }

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *pins)
{
    assert(pins->Mode == GPIO_MODE_INPUT && pins->Pull == GPIO_NOPULL);
    assert((port == GPIOA && pins->Pin == (GPIO_PIN_8 | GPIO_PIN_9)) ||
           (port == GPIOB && pins->Pin == (GPIO_PIN_6 | GPIO_PIN_7)));
    ++gpio_calls;
}

HAL_StatusTypeDef HAL_TIM_Encoder_Init(TIM_HandleTypeDef *timer, const TIM_Encoder_InitTypeDef *c)
{
    assert(timer->Instance == TIM1 || timer->Instance == TIM4);
    assert(timer->Init.Period == 65535 && timer->Init.Prescaler == 0);
    assert(c->EncoderMode == TIM_ENCODERMODE_TI12);
    assert(c->IC1Filter == ENCODER_INPUT_FILTER && c->IC2Filter == ENCODER_INPUT_FILTER);
    assert(c->IC1Selection == TIM_ICSELECTION_DIRECTTI && c->IC2Selection == TIM_ICSELECTION_DIRECTTI);
    return result();
}

HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
    assert(channel == TIM_CHANNEL_ALL);
    HAL_StatusTypeDef status = result();
    if (status == HAL_OK) {
        if (timer->Instance == TIM1) left_started = 1;
        else right_started = 1;
    }
    return status;
}

HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *timer, uint32_t channel)
{
    assert(timer->Instance == TIM1 && channel == TIM_CHANNEL_ALL);
    left_started = 0;
    ++stops;
    return HAL_OK;
}

int main(void)
{
    EncoderCounts counts;
    assert(!encoder_read(&counts));
    for (unsigned failure=1; failure<=4; ++failure) {
        calls=0; fail_at=failure;
        assert(!encoder_init());
        assert(!encoder_read(&counts) && !left_started && !right_started);
    }
    assert(stops == 1);
    calls=fail_at=0;
    assert(encoder_init() && left_started && right_started && gpio_calls == 10);
    assert(!encoder_read(NULL));
    fake_tim1.counter=65535; fake_tim4.counter=12;
    assert(encoder_read(&counts) && counts.left==65535 && counts.right==12);
    assert(fake_tim3.compare[0]==0 && fake_tim3.compare[1]==0);
    assert(fake_gpio_a.output==0 && fake_gpio_b.output==0);
    puts("PASS: encoder HAL x4, 4 init/start failure paths, cleanup and snapshots");
    return 0;
}
