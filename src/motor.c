#include "motor.h"
#include "board_config.h"
#include "control_config.h"
#include <math.h>

static TIM_HandleTypeDef pwm_timer;
static bool pwm_ready;

void motor_safe_gpio_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /* Preload low BEFORE switching pins from reset input mode to output mode. */
    HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_DIRECTION_PORT,
                     MOTOR_LEFT_IN1 | MOTOR_LEFT_IN2 | MOTOR_RIGHT_IN1 | MOTOR_RIGHT_IN2,
                     GPIO_PIN_RESET);
    GPIO_InitTypeDef pins = {0};
    pins.Pin = MOTOR_STBY_PIN | MOTOR_LEFT_IN1 | MOTOR_LEFT_IN2 | MOTOR_RIGHT_IN1 | MOTOR_RIGHT_IN2;
    pins.Mode = GPIO_MODE_OUTPUT_PP;
    pins.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &pins);
}

void motor_stop(void)
{
    /* Independent of the TIM handle: safe even when PWM initialization failed. */
    HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_RESET);
    if (pwm_ready) {
        __HAL_TIM_SET_COMPARE(&pwm_timer, MOTOR_LEFT_CHANNEL, 0);
        __HAL_TIM_SET_COMPARE(&pwm_timer, MOTOR_RIGHT_CHANNEL, 0);
    }
    HAL_GPIO_WritePin(MOTOR_DIRECTION_PORT,
                     MOTOR_LEFT_IN1 | MOTOR_LEFT_IN2 | MOTOR_RIGHT_IN1 | MOTOR_RIGHT_IN2,
                     GPIO_PIN_RESET);
}

bool motor_init(void)
{
    motor_stop();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    GPIO_InitTypeDef pins = {0};
    pins.Pin = MOTOR_PWM_PINS;
    pins.Mode = GPIO_MODE_AF_PP;
    pins.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MOTOR_PWM_PORT, &pins);
    pwm_timer.Instance = TIM3;
    pwm_timer.Init.Prescaler = 0;
    pwm_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    pwm_timer.Init.Period = BOARD_PWM_PERIOD;
    pwm_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_PWM_Init(&pwm_timer) != HAL_OK) return false;
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&pwm_timer, &oc, MOTOR_LEFT_CHANNEL) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&pwm_timer, &oc, MOTOR_RIGHT_CHANNEL) != HAL_OK ||
        HAL_TIM_PWM_Start(&pwm_timer, MOTOR_LEFT_CHANNEL) != HAL_OK ||
        HAL_TIM_PWM_Start(&pwm_timer, MOTOR_RIGHT_CHANNEL) != HAL_OK) return false;
    pwm_ready = true;
    motor_stop();
    return true;
}

static void set_wheel(uint16_t in1, uint16_t in2, uint32_t channel,
                      float command, float gain, float deadzone, bool in1_forward)
{
    uint16_t ccr = motor_command_to_ccr(command, gain, deadzone, BOARD_PWM_PERIOD);
    /* For a zero individual wheel, PWM low selects short brake while STBY is high. */
    HAL_GPIO_WritePin(MOTOR_DIRECTION_PORT, in1, in1_forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_DIRECTION_PORT, in2, in1_forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&pwm_timer, channel, ccr);
}

void motor_apply(MotorCommand command)
{
    if (!pwm_ready || !command.enabled || !isfinite(command.left) || !isfinite(command.right) ||
        command.left < 0.0f || command.right < 0.0f ||
        (command.left == 0.0f && command.right == 0.0f)) {
        motor_stop();
        return;
    }
    set_wheel(MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_LEFT_CHANNEL, command.left,
              MOTOR_LEFT_GAIN, MOTOR_LEFT_DEADZONE, MOTOR_LEFT_FORWARD_IN1_HIGH);
    set_wheel(MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, MOTOR_RIGHT_CHANNEL, command.right,
              MOTOR_RIGHT_GAIN, MOTOR_RIGHT_DEADZONE, MOTOR_RIGHT_FORWARD_IN1_HIGH);
    HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_SET);
}
