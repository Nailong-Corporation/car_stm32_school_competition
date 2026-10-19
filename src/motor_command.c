#include "motor_command.h"
#include "control_config.h"
#include <math.h>

/**
 * @brief 将基础 PWM 与转向半差混合为左右轮指令。
 * 正 correction 使左轮较大、右轮较小；限幅保证两轮非负且不超运行上限。
 * 非法参数或非正基础值返回禁用的零输出。
 */
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

/**
 * @brief 将单轮 PWM 指令经静态增益和死区补偿后换算成定时器 CCR。
 * period 为 ARR；使用 255 指令标尺，零指令先返回零，补偿不能使停车变成转动。
 */
uint16_t motor_command_to_ccr(float command, float gain, float deadzone, uint16_t period)
{
    if (!isfinite(command) || !isfinite(gain) || !isfinite(deadzone) ||
        command <= 0.0f || gain <= 0.0f || deadzone < 0.0f) return 0;
    float calibrated = command * gain + deadzone;
    calibrated = fminf(MOTOR_PWM_LIMIT, calibrated);
    /* period + 1 represents 100% in PWM1; operating limit is below that. */
    return (uint16_t)(calibrated * ((float)period + 1.0f) / MOTOR_COMMAND_MAX + 0.5f);
}
