/* ---------- pid.h ---------- */
#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float err_last;      /* e[k-1] */
    float err_last2;     /* e[k-2] */
    float out;           /* 累计输出, ★必须是有符号 */
    float out_min, out_max;
} PidInc;

void  pid_init(PidInc *p, float kp, float ki, float kd, float out_min, float out_max);
void  pid_reset(PidInc *p);
float pid_update(PidInc *p, float err);   /* 返回累计后的输出(已限幅) */

#endif