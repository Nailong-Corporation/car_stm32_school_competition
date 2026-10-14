#include "motor_command.h"
#include "control_config.h"
#include <math.h>

MotorCommand motor_mix(float base, float correction)
{
    MotorCommand result = {0};
    if (!isfinite(base) || !isfinite(correction) || base <= 0.0f) return result;
    if (base > MOTOR_PWM_LIMIT) base = MOTOR_PWM_LIMIT;
    float limit = fminf(base, MOTOR_PWM_LIMIT - base);
    correction = fmaxf(-limit, fminf(limit, correction));
    result.left = base + correction;
    result.right = base - correction;
    result.enabled = true;
    return result;
}

uint16_t motor_command_to_ccr(float command, float gain, float deadzone, uint16_t period)
{
    if (!isfinite(command) || !isfinite(gain) || !isfinite(deadzone) ||
        command <= 0.0f || gain <= 0.0f || deadzone < 0.0f) return 0;
    float calibrated = command * gain + deadzone;
    calibrated = fminf(MOTOR_PWM_LIMIT, calibrated);
    /* period + 1 represents 100% in PWM1; operating limit is below that. */
    return (uint16_t)(calibrated * ((float)period + 1.0f) / MOTOR_COMMAND_MAX + 0.5f);
}
