#include "speed_control.h"
#include "control_config.h"
#include "motion_config.h"
#include <math.h>

void speed_control_reset(SpeedControl *s)
{
    pid_reset(&s->left);
    pid_reset(&s->right);
    s->left_stall_s = s->right_stall_s = 0.0f;
}

void speed_control_init(SpeedControl *s)
{
    PidConfig c = {SPEED_KP, SPEED_KI, SPEED_KD, SPEED_INTEGRAL_LIMIT, PID_DERIVATIVE_TAU_S};
    pid_init(&s->left, c);
    pid_init(&s->right, c);
    speed_control_reset(s);
}

static bool wheel(Pid *pid, float target, float measured, float dt, float *stall, float *pwm)
{
    if (!isfinite(target) || target < 0.0f || target > WHEEL_TARGET_MAX_CM_S ||
        !isfinite(measured)) return false;
    if (target == 0.0f) {
        pid_reset(pid);
        *stall = *pwm = 0.0f;
        return true;
    }
    *stall = target >= SPEED_STALL_TARGET_CM_S && measured < SPEED_STALL_MEASURED_CM_S ?
        *stall + dt : 0.0f;
    if (*stall >= SPEED_STALL_TIMEOUT_S || measured < -SPEED_STALL_MEASURED_CM_S) return false;
    return pid_update_bounded(pid, target - measured, dt, 0.0f, MOTOR_PWM_LIMIT,
                              target * SPEED_FEEDFORWARD, pwm);
}

bool speed_control_update(SpeedControl *s, float lt, float rt, float lm, float rm,
                          float dt, MotorCommand *command)
{
    *command = (MotorCommand){0};
    if (!isfinite(dt) || dt <= 0.0f || dt > (float)CONTROL_MAX_INTERVAL_MS / 1000.0f)
        return false;
    float l, r;
    if (!wheel(&s->left, lt, lm, dt, &s->left_stall_s, &l) ||
        !wheel(&s->right, rt, rm, dt, &s->right_stall_s, &r)) {
        speed_control_reset(s);
        return false;
    }
    *command = (MotorCommand){l, r, lt > 0.0f || rt > 0.0f};
    return true;
}
