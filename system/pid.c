/* ============================================================================
 *  pid.c —— 通用 PID 控制器(纯数学, 与具体电机/对象无关)
 * ==========================================================================*/
#include "pid.h"

void pid_init(Pid *p, int32_t kp, int32_t ki, int32_t kd, int32_t div,
              int32_t i_clamp, int32_t out_max, int32_t deadband,
              uint32_t dt_min_ms)
{
    p->kp = kp; p->ki = ki; p->kd = kd; p->div = div;
    p->i_clamp = i_clamp; p->out_max = out_max; p->deadband = deadband;
    p->dt_min_ms = dt_min_ms;
    pid_reset(p);
}

void pid_reset(Pid *p)
{
    p->integ = 0;
    p->prev_err = 0;
    p->first = 1;
}

void pid_set_gains(Pid *p, int32_t kp, int32_t ki, int32_t kd, int32_t div)
{
    p->kp = kp; p->ki = ki; p->kd = kd; p->div = div;
}

int32_t pid_update(Pid *p, int32_t err, uint32_t dt_ms)
{
    int64_t  term;
    int32_t  out;
    uint32_t dt;

    dt = (dt_ms < p->dt_min_ms) ? p->dt_min_ms : dt_ms;

    /* 死区: 已对齐, 清积分防漂移, 输出 0 */
    if ((err > -p->deadband) && (err < p->deadband)) {
        p->integ = 0;
        p->prev_err = err;
        p->first = 0;
        return 0;
    }

    /* P */
    term = (int64_t)p->kp * err;

    /* I: Σ(err*dt), 限幅 */
    p->integ += (int64_t)err * (int64_t)dt;
    if (p->integ >  (int64_t)p->i_clamp) p->integ =  (int64_t)p->i_clamp;
    if (p->integ < -(int64_t)p->i_clamp) p->integ = -(int64_t)p->i_clamp;
    term += (p->integ * p->ki) / 100;

    /* D: (err - prev)/dt, 首拍为 0 */
    if (p->first) { p->first = 0; }
    else { term += ((int64_t)(err - p->prev_err) * p->kd * 100) / (int64_t)dt; }
    p->prev_err = err;

    if (p->div <= 0) p->div = 1;
    out = (int32_t)(term / p->div);

    if (out >  p->out_max) out =  p->out_max;
    if (out < -p->out_max) out = -p->out_max;
    return out;
}

int32_t pid_compute(Pid *p, int32_t setpoint, int32_t feedback, uint32_t dt_ms)
{
    return pid_update(p, setpoint - feedback, dt_ms);
}
