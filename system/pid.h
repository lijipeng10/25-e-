#ifndef PID_H
#define PID_H

#include <stdint.h>

/* ============================================================================
 *  pid.h —— 通用 PID 控制器(位置式, 输出可为速度/占空比/其它控制量)
 * ----------------------------------------------------------------------------
 *  用途: 云台视觉伺服 / 直流电机巡线 等任何"误差 -> 控制量"的闭环。
 *  输入误差 err 与 输出 out 的单位由调用者约定, 本模块只做纯数学。
 *
 *  控制律:
 *      out = (Kp*err + Ki*integ + Kd*deriv) / div
 *  其中:
 *      integ = Σ(err*dt), 限幅 i_clamp(防积分饱和)
 *      deriv = (err - prev_err)/dt
 *  若 |err| <= deadband, 视为已对齐, 清积分并输出 0(防抖)。
 *
 *  用法:
 *      Pid p;
 *      pid_init(&p, 5,1,4,10, 20000,50,2,5);   // 增益/限幅/死区/dt下限
 *      while(1){
 *          // 直接给误差: 视觉伺服用(已把偏差当作误差)
 *          int32_t out = pid_update(&p, err, dt_ms);
 *          // 或给定设定值与反馈: 巡线用 setpoint - feedback
 *          // int32_t out = pid_compute(&p, setpoint, feedback, dt_ms);
 *          drive_actuator(out);
 *      }
 * ==========================================================================*/

typedef struct {
    /* 参数(可在线修改) */
    int32_t  kp;          /* 比例系数 */
    int32_t  ki;          /* 积分系数 */
    int32_t  kd;          /* 微分系数 */
    int32_t  div;         /* 输出归一分母(实际增益 = kp/div 等) */
    int32_t  i_clamp;     /* 积分累加绝对值上限 */
    int32_t  out_max;     /* 输出绝对值上限 */
    int32_t  deadband;    /* 死区(误差绝对值) */
    uint32_t dt_min_ms;   /* 最小合法 dt(ms), 防除 0 */

    /* 内部状态 */
    int64_t  integ;       /* 积分累加 */
    int32_t  prev_err;    /* 上次误差 */
    uint8_t  first;       /* 首拍: 微分项 = 0 */
} Pid;

void    pid_init(Pid *p, int32_t kp, int32_t ki, int32_t kd, int32_t div,
                 int32_t i_clamp, int32_t out_max, int32_t deadband,
                 uint32_t dt_min_ms);
void    pid_reset(Pid *p);
void    pid_set_gains(Pid *p, int32_t kp, int32_t ki, int32_t kd, int32_t div);

/* 由误差直接算控制量(单位: 调用者约定) */
int32_t pid_update(Pid *p, int32_t err, uint32_t dt_ms);

/* 由设定值与反馈量算, 内部 err = setpoint - feedback */
int32_t pid_compute(Pid *p, int32_t setpoint, int32_t feedback, uint32_t dt_ms);

#endif /* PID_H */
