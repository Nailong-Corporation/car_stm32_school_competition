#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H
#include "motor_command.h"
#include "pid.h"
typedef enum {
    SPEED_FAULT_NONE, SPEED_FAULT_INTERVAL, SPEED_FAULT_INPUT,
    SPEED_FAULT_REVERSE, SPEED_FAULT_STALL, SPEED_FAULT_PID
} SpeedFault;
typedef struct {
    Pid left, right;
    float left_stall_s, right_stall_s;
    float left_reverse_s, right_reverse_s;
    float left_reverse_cm, right_reverse_cm;
    bool reverse_pending; /* True: both outputs inhibited, no latched fault yet. */
    SpeedFault fault;
    uint8_t fault_wheel; /* 0 公共输入/无故障，1 左轮，2 右轮。 */
} SpeedControl;
void speed_control_init(SpeedControl *speed);
void speed_control_reset(SpeedControl *speed);
/* true may still mean disabled output during reverse confirmation; caller must
 * apply command.enabled, not infer motor enable from the return value. */
bool speed_control_update(SpeedControl *speed, float left_target, float right_target,
                          float left_measured, float right_measured, float dt,
                          MotorCommand *command);
#endif
