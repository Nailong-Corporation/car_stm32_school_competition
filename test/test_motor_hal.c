#include "motor.h"
#include "board_config.h"
#include "motion_config.h"
#include "control_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

GPIO_TypeDef fake_gpio_a, fake_gpio_b;
TIM_TypeDef fake_tim3;
static unsigned calls, fail_at, stop_writes, enable_writes;

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pins, GPIO_PinState state)
{
    if (state == GPIO_PIN_SET) port->output |= pins;
    else port->output &= (uint16_t)~pins;
    if (port == MOTOR_STBY_PORT && pins == MOTOR_STBY_PIN) {
        if (state == GPIO_PIN_RESET) ++stop_writes;
        else ++enable_writes;
    }
}

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *pins)
{
    if (port == GPIOB && (pins->Pin & MOTOR_STBY_PIN))
        assert((port->output & MOTOR_STBY_PIN) == 0); /* low preloaded before output mode */
}

static HAL_StatusTypeDef result(void)
{
    return ++calls == fail_at ? HAL_ERROR : HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *timer)
{
    assert(timer->Instance == TIM3 && timer->Init.Period == 6399 && timer->Init.Prescaler == 0);
    return result();
}

HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *timer, TIM_OC_InitTypeDef *oc, uint32_t channel)
{
    assert(channel <= 1 && oc->Pulse == 0);
    timer->Instance->compare[channel] = oc->Pulse;
    return result();
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
    (void)timer;
    assert(channel <= 1);
    return result();
}

static void stopped(void)
{
    assert((fake_gpio_b.output & MOTOR_STBY_PIN) == 0);
    assert((fake_gpio_b.output & (MOTOR_LEFT_IN1 | MOTOR_LEFT_IN2 | MOTOR_RIGHT_IN1 | MOTOR_RIGHT_IN2)) == 0);
}

int main(void)
{
    fake_gpio_b.output = 0xffff;
    motor_safe_gpio_init();
    stopped();
    motor_stop(); /* must not dereference an uninitialized TIM handle */
    stopped();
    for (unsigned failure = 1; failure <= 5; ++failure) {
        calls = 0;
        fail_at = failure;
        assert(!motor_init()); /* failure at each HAL init/config/start step */
        assert(enable_writes == 0);
        motor_stop();
        stopped();
    }
    calls = fail_at = 0;
    assert(motor_init());
    stopped();
    assert(fake_tim3.compare[0] == 0 && fake_tim3.compare[1] == 0);
    motor_apply((MotorCommand){255.0f, 255.0f, true});
    assert(fake_tim3.compare[0] == 6400 && fake_tim3.compare[1] == 6400);
    assert(fake_gpio_b.output & MOTOR_STBY_PIN);
    motor_apply((MotorCommand){50.0f, 25.0f, true});
    assert(fake_tim3.compare[0] == 1255 && fake_tim3.compare[1] == 627);
    assert(fake_gpio_b.output & MOTOR_STBY_PIN);
    assert(!(fake_gpio_b.output & MOTOR_LEFT_IN1));
    assert(fake_gpio_b.output & MOTOR_LEFT_IN2);
    assert(fake_gpio_b.output & MOTOR_RIGHT_IN1);
    assert(!(fake_gpio_b.output & MOTOR_RIGHT_IN2));
    motor_apply((MotorCommand){50.0f, 0.0f, true});
    assert(fake_tim3.compare[1] == 0 && (fake_gpio_b.output & MOTOR_STBY_PIN));
    motor_apply((MotorCommand){0.0f, 0.0f, true});
    stopped();
    assert(fake_tim3.compare[0] == 0 && fake_tim3.compare[1] == 0);
    motor_apply((MotorCommand){50.0f, 50.0f, false});
    stopped();
    motor_apply((MotorCommand){-60.0f, 50.0f, true});
#if MOTOR_REVERSE_BRAKE_ENABLED && ENCODER_CONTROL_ENABLED
    assert(fake_gpio_b.output & MOTOR_STBY_PIN);
    assert(fake_gpio_b.output & MOTOR_LEFT_IN1);
    assert(!(fake_gpio_b.output & MOTOR_LEFT_IN2));
    assert(fake_tim3.compare[0] == motor_command_to_ccr(60, MOTOR_LEFT_GAIN, 0, BOARD_PWM_PERIOD));
#else
    stopped();
#endif
    motor_apply((MotorCommand){-61.0f, 50.0f, true});
    stopped();
    motor_apply((MotorCommand){NAN, 50.0f, true});
    stopped();
    assert(stop_writes > 0 && enable_writes == (3 + (MOTOR_REVERSE_BRAKE_ENABLED && ENCODER_CONTROL_ENABLED)));
    puts("PASS: simulated HAL motor startup, 5 init failure paths, directions, PWM and independent stop");
    return 0;
}
