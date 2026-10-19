#include "speed_control.h"
#include "control_config.h"
#include "motion_config.h"
#include <math.h>

/**
 * @brief 清空左右速度 PID 历史及低速/堵转累计时间。
 */
void speed_control_reset(SpeedControl *s)
{
    pid_reset(&s->left);
    pid_reset(&s->right);
    s->left_stall_s = s->right_stall_s = 0.0f;
}

/**
 * @brief 用集中参数初始化左右独立速度 PID，并清空反馈保护状态。
 */
void speed_control_init(SpeedControl *s)
{
    PidConfig c = {SPEED_KP, SPEED_KI, SPEED_KD, SPEED_INTEGRAL_LIMIT, PID_DERIVATIVE_TAU_S};
    pid_init(&s->left, c);
    pid_init(&s->right, c);
    speed_control_reset(s);
}

/**
 * @brief 根据单轮目标/实测速度（cm/s）与 dt（秒）生成限幅 PWM。
 * 目标为零时复位并输出零；反向反馈、低速持续超限或非法值返回 false。
 * stall 保存低速累计秒数，pwm 接收含前馈及抗饱和 PID 的输出。
 */
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

/**
 * @brief 执行左右轮速度内环，将最终 PWM 写入 command。
 * 任一轮反馈或计算失败时保持禁用零输出、清空两轮历史并返回 false。
 */
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
