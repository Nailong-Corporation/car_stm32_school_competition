#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct {
    float kp, ki, kd, integral_limit, derivative_tau;
} PidConfig;
typedef struct {
    PidConfig config;
    float integral, previous_error, derivative;
    bool primed;
} Pid;

void pid_init(Pid *pid, PidConfig config);
void pid_reset(Pid *pid);
bool pid_update(Pid *pid, float error, float dt, float limit, float *output);

#endif
