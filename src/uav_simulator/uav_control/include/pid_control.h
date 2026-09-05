#ifndef __LLWPID_H__
#define __LLWPID_H__

typedef struct 
{
    float ki, kp, kd;
    float cur_err, last_err;
    float output, integral, dt;
    float integral_limit, output_limit;
} PID;

float PID_calculate(PID *pid, float expect, float feedback);
void PID_Init(PID *pid, float kp, float ki, float kd, float dt, float out_limit, float int_limit);

#endif