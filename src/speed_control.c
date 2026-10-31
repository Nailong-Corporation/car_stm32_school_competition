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
    s->left_reverse_s = s->right_reverse_s = 0.0f;
    s->left_reverse_cm = s->right_reverse_cm = 0.0f;
    s->reverse_pending = false;
    s->fault = SPEED_FAULT_NONE;
    s->fault_wheel = 0;
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
 * @brief 根据单轮目标/实测速度（cm/s）与 dt（秒）生成限幅 PWM；开启反转制动时可输出受限负 PWM。
 * 目标为零时复位并输出零；反向保护在双轮入口统一检测，低速持续超限或非法值返回故障码。
 * stall 保存低速累计秒数，pwm 接收含前馈及抗饱和 PID 的输出。
 */
static SpeedFault wheel(Pid *pid, float target, float measured, float dt, float *stall,
                        bool reverse_brake, float *pwm)
{
    if (!isfinite(target) || target < 0.0f || target > WHEEL_TARGET_MAX_CM_S ||
        !isfinite(measured)) return SPEED_FAULT_INPUT;
    if (target == 0.0f) {
        pid_reset(pid);
        *stall = *pwm = 0.0f;
        return SPEED_FAULT_NONE;
    }
    *stall = target >= SPEED_STALL_TARGET_CM_S && measured < SPEED_STALL_MEASURED_CM_S ?
        *stall + dt : 0.0f;
    if (*stall >= SPEED_STALL_TIMEOUT_S) return SPEED_FAULT_STALL;
#if MOTOR_REVERSE_BRAKE_ENABLED
    /* 仅显式请求才对正向超速施加反向力矩；PID 抗饱和限制负 PWM。 */
    if (reverse_brake && measured > BRAKING_REVERSE_MIN_SPEED_CM_S &&
        measured > target + REVERSE_BRAKE_MARGIN_CM_S) {
        /* Same speed PID error and anti-windup as forward drive; negative PWM
         * represents torque against positive motion, never a reverse speed target. */
        float effort=0;
        if(!pid_update_bounded(pid,target-measured,dt,-REVERSE_BRAKE_MAX_PWM,0,
                               0,&effort)) return SPEED_FAULT_PID;
        *pwm=effort;
        return SPEED_FAULT_NONE;
    }
#else
    (void)reverse_brake;
#endif
    return pid_update_bounded(pid, target - measured, dt, 0.0f, MOTOR_PWM_LIMIT,
                              target * SPEED_FEEDFORWARD, pwm) ? SPEED_FAULT_NONE : SPEED_FAULT_PID;
}

/* Feedback is already filtered by odometry. A single raw reverse count at
 * production scale yields about -0.594 cm/s at 5 ms: inhibit immediately,
 * but require bounded evidence before locking. Never ignore a zero-target wheel. */
static bool reverse_confirmed(float measured, float dt, float *age, float *distance)
{
    if (measured >= -SPEED_REVERSE_THRESHOLD_CM_S) {
        *age = *distance = 0.0f;
        return false;
    }
    *age += dt;
    *distance += -measured * dt;
    return measured <= -SPEED_REVERSE_IMMEDIATE_CM_S ||
           *age + 0.0000001f >= SPEED_REVERSE_CONFIRM_S ||
           *distance + 0.0000001f >= SPEED_REVERSE_CONFIRM_CM;
}

/**
 * @brief 执行左右轮速度内环，将最终 PWM 写入 command。
 * 轻微反向待确认时返回 true 但 command 禁用；确认故障返回 false。
 * 任一轮反馈或计算失败时保持禁用零输出、清空两轮历史并返回 false。
 */
bool speed_control_update_braking(SpeedControl *s, float lt, float rt, float lm, float rm,
                          float dt, bool reverse_brake, MotorCommand *command)
{
    *command = (MotorCommand){0};
    s->fault = SPEED_FAULT_NONE;
    s->fault_wheel = 0;
    s->reverse_pending = false;
    if (!isfinite(dt) || dt <= 0.0f || dt > (float)CONTROL_MAX_INTERVAL_MS / 1000.0f) {
        s->fault = SPEED_FAULT_INTERVAL;
        return false;
    }
    SpeedFault fault = SPEED_FAULT_NONE;
    uint8_t failed_wheel = 0;
    if (!isfinite(lt) || lt < 0 || lt > WHEEL_TARGET_MAX_CM_S || !isfinite(lm)) {
        fault = SPEED_FAULT_INPUT; failed_wheel = 1;
    } else if (!isfinite(rt) || rt < 0 || rt > WHEEL_TARGET_MAX_CM_S || !isfinite(rm)) {
        fault = SPEED_FAULT_INPUT; failed_wheel = 2;
    } else {
        bool left_reverse = reverse_confirmed(lm, dt, &s->left_reverse_s, &s->left_reverse_cm);
        bool right_reverse = reverse_confirmed(rm, dt, &s->right_reverse_s, &s->right_reverse_cm);
        if (left_reverse || right_reverse) {
            fault = SPEED_FAULT_REVERSE;
            failed_wheel = left_reverse ? 1 : 2;
        } else if (s->left_reverse_s > 0 || s->right_reverse_s > 0) {
            pid_reset(&s->left);
            pid_reset(&s->right);
            s->left_stall_s = s->right_stall_s = 0.0f;
            s->reverse_pending = true;
            return true; /* command remains disabled; no integrator windup. */
        }
    }
    float l = 0, r = 0;
    if (fault == SPEED_FAULT_NONE) {
        fault = wheel(&s->left, lt, lm, dt, &s->left_stall_s, reverse_brake, &l);
        failed_wheel = 1;
    }
    if (fault == SPEED_FAULT_NONE) {
        fault = wheel(&s->right, rt, rm, dt, &s->right_stall_s, reverse_brake, &r);
        failed_wheel = 2;
    }
    if (fault != SPEED_FAULT_NONE) {
        speed_control_reset(s);
        s->fault = fault;
        s->fault_wheel = failed_wheel;
        return false;
    }
    *command = (MotorCommand){l, r, lt > 0.0f || rt > 0.0f};
    return true;
}


bool speed_control_update(SpeedControl *s,float lt,float rt,float lm,float rm,
                          float dt,MotorCommand *command)
{
    return speed_control_update_braking(s,lt,rt,lm,rm,dt,false,command);
}
