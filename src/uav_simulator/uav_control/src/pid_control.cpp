#include "pid_control.h"

void PID_Init(PID *pid, float kp, float ki, float kd, float dt, float out_limit, float int_limit)
{
    pid->last_err = pid->cur_err = pid->integral = 0;
    pid->dt = dt;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_limit = int_limit;
    pid->output_limit = out_limit;
}

float PID_calculate(PID *pid, float expect, float feedback)
{
    pid->last_err = pid->cur_err;

    pid->cur_err = expect - feedback;

    pid->integral += pid->cur_err * pid->ki;
    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;

    pid->output = pid->kp * pid->cur_err + pid->integral + pid->kd * (pid->cur_err - pid->last_err) / pid->dt;
    if (pid->output > pid->output_limit)
        pid->output = pid->output_limit;
    else if (pid->output < -pid->output_limit)
        pid->output = -pid->output_limit;

    return pid->output;
}
