#include "pid.h"
#include <math.h>

static float clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

void pid_reset(Pid *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->derivative = 0.0f;
    pid->primed = false;
}

void pid_init(Pid *pid, PidConfig config)
{
    pid->config = config;
    pid_reset(pid);
}

bool pid_update(Pid *pid, float error, float dt, float limit, float *output)
{
    return pid_update_bounded(pid, error, dt, -limit, limit, 0.0f, output);
}

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
