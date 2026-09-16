#ifndef ENCODER_H
#define ENCODER_H

/*
    //ENCODER PIN MAP//
    E1A <==> B20      E1B <==> A14
    E2A <==> A25      E2B <==> B25

    //测速参数
    编码器线数13线, 电机减速比1:20, 轮子一圈 260 个脉冲(实测确认)
    ★ 数的是 A 相的【上升沿】(SysConfig 里 A 相中断极性 = RISE)。
      改成双边沿(RISE_FALL)一圈就变 520 次中断, 这里的常数要跟着改;
      但双边沿【测不出方向】—— 要方向必须读 B 相, 见下面。
*/

#define PI                      3.14
#define ENCODER_PULSE           260  // 轮子转一圈的脉冲数(实测)
#define ENCODER_WHEEL_D         48   //mm

/* ★★ 转向判据(正交解码): 在 A 相的上升沿读一下 B 相的电平 ——
 *    B 高 = 正转, B 低 = 反转。这样速度就是【带符号】的: 正 = 前进, 负 = 后退。
 *   ★ B 相【不需要开中断】, 只要配成输入就行(已经配好了) —— 中断只挂在 A 相上。
 *   ★ 每路的极性实测反了(手往前转却显示负)就把对应的那个改成 -1。
 *   ★ B 相线没接的话 B 恒为低, 表现为"不管正转反转都显示负", 一眼就能看出来。 */
#define ENCODER_1_SIGN          (+1)
#define ENCODER_2_SIGN          (+1)

#include "ti_msp_dl_config.h"

void encoder_init(void);
void encoder_get_speed(uint8_t id);

/* 原始脉冲计数【带符号】(正 = 前进, 负 = 后退), 由 GROUP1 中断累加(中断在 empty.c 里) */
extern int32_t encoder_1_A;
extern int32_t encoder_2_A;

/* 实测速度 mm/s【带符号】, 由 encoder_get_speed() 每 50ms 刷新 */
extern float speed_1;
extern float speed_2;


#endif /* ENCODER_H */
