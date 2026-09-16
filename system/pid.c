/* ---------- pid.c ---------- */
#include "pid.h"

void pid_init(PidInc *p, float kp, float ki, float kd, float out_min, float out_max)
{
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->out_min = out_min; p->out_max = out_max;
    pid_reset(p);
}

void pid_reset(PidInc *p)
{
    p->err_last  = 0.0f;
    p->err_last2 = 0.0f;
    p->out       = 0.0f;
}

/* 增量式: Δout = kp*(e-e1) + ki*e + kd*(e-2e1+e2)
 * ★ out 全程用 float, 限幅也在 float 里做 —— 绝不能先转 uint16_t 再累加 */
float pid_update(PidInc *p, float err)
{
    p->out += p->kp * (err - p->err_last)
            + p->ki * err
            + p->kd * (err - 2.0f * p->err_last + p->err_last2);

    p->err_last2 = p->err_last;
    p->err_last  = err;

    if (p->out > p->out_max) { p->out = p->out_max; }
    if (p->out < p->out_min) { p->out = p->out_min; }

    return p->out;
}