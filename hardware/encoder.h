#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* 轮速闭环: 编码器测速 + 每轮速度PI。
 * 接线 E1A=PB20, E2A=PA25; 中断由 GROUP1_IRQHandler 分派(见 sm_encoder.c)。
 * 用法: motor_speed_init() 后, 主循环每10ms调 motor_speed_update(),
 *       要动轮子用 motor_speed_set(id, mm_s)(正=前进, 负=后退, 0=停)。
 * ★ 本模块默认不激活: 没调过 motor_speed_set() 之前绝不碰电机。
 * ★ 只数A相边沿, 数不出方向 -> 实测速度的符号按【命令方向】给。 */

#define ENC_GPIO_IRQN       GPIOA_INT_IRQn   /* GPIOA/B 共用一个向量(IRQ 1) */

void    motor_speed_init(void);
void    motor_speed_isr(uint32_t sta_gpioa, uint32_t sta_gpiob);
void    motor_speed_update(void);
void    motor_speed_set(uint8_t id, int32_t mm_s);
void    motor_speed_stop(uint8_t id);
void    motor_speed_disable(void);
/* ★ 编码器故障标志: 命令有速度但实测一直≈0 时置1, 并且【已经把该轮占空比切到0】。
 *   用来防"编码器没接好 -> PI 以为是0 -> 积分顶到满占空比 -> 车突然全速冲出去" */
uint8_t  motor_speed_get_fault(uint8_t id);

/* ★ 最近一个采样周期(20ms)里数到的【原始脉冲数】。诊断用:
 *     轮子停着的时候是 0        -> 正常
 *     停着还是非 0              -> 输入脚在飘/干扰(接线或供电问题)
 *     手转轮子会跟着变          -> 编码器是好的
 *   "实测速度"是它换算出来的, 所以这个数不对, 速度一定不对。 */
int32_t  motor_speed_get_raw(uint8_t id);

int32_t  motor_speed_get(uint8_t id);         /* 实测速度 mm/s */
int32_t  motor_speed_get_target(uint8_t id);  /* 目标速度 mm/s */
uint16_t motor_speed_get_duty(uint8_t id);    /* PI 给出的占空比 */
uint8_t  motor_speed_is_active(void);

#endif /* ENCODER_H */
