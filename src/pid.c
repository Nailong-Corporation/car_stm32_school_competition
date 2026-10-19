#include "pid.h"
#include <math.h>

/**
 * @brief 将 value 对称限制在 [-limit, +limit]，供积分限幅使用。
 */
static float clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

/**
 * @brief 清空积分、前次误差和微分滤波状态，使下一帧不产生历史微分冲击。
 */
void pid_reset(Pid *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->derivative = 0.0f;
    pid->primed = false;
}

/**
 * @brief 保存 PID 参数并复位历史，供位置环和左右速度环共用。
 */
void pid_init(Pid *pid, PidConfig config)
{
    pid->config = config;
    pid_reset(pid);
}

/**
 * @brief 按实际 dt 计算无前馈的对称限幅 PID，将结果写入 output。
 * limit 为输出幅值上限；参数非法时返回 false，并复位 PID 历史。
 */
bool pid_update(Pid *pid, float error, float dt, float limit, float *output)
{
    return pid_update_bounded(pid, error, dt, -limit, limit, 0.0f, output);
}

/**
 * @brief 计算含前馈、微分低通及积分抗饱和的 PID，输出限制在 [low, high]。
 * error 与 dt 分别为本周期误差和秒数；按前馈加 PID 的总输出判断是否接受积分。
 * output 必须可写；参数或结果非法时输出零、复位历史并返回 false。
 */
bool pid_update_bounded(Pid *pid, float error, float dt, float low, float high,
                        float feedforward, float *output)
{
    *output = 0.0f;
    PidConfig c = pid->config;
    if (!isfinite(error) || !isfinite(dt) || dt <= 0.0f || dt > 0.1f ||
        !isfinite(low) || !isfinite(high) || low > high || !isfinite(feedforward) ||
        !isfinite(c.kp) || c.kp < 0.0f ||
        !isfinite(c.ki) || c.ki < 0.0f || !isfinite(c.kd) || c.kd < 0.0f ||
        !isfinite(c.integral_limit) || c.integral_limit < 0.0f ||
        !isfinite(c.derivative_tau) || c.derivative_tau < 0.0f) {
        pid_reset(pid);
        return false;
    }
    float slope = pid->primed ? (error - pid->previous_error) / dt : 0.0f;
    float alpha = dt / (c.derivative_tau + dt);
    pid->derivative += alpha * (slope - pid->derivative);
    float integral = clamp(pid->integral + error * dt, c.integral_limit);
    float candidate = feedforward + c.kp * error + c.ki * integral + c.kd * pid->derivative;
    /* Conditional integration: do not integrate farther into actuator saturation. */
    if (!((candidate > high && error > 0.0f) ||
          (candidate < low && error < 0.0f))) pid->integral = integral;
    float value = feedforward + c.kp * error + c.ki * pid->integral + c.kd * pid->derivative;
    if (!isfinite(value)) {
        pid_reset(pid);
        return false;
    }
    *output = fmaxf(low, fminf(high, value));
    pid->previous_error = error;
    pid->primed = true;
    return true;
}
