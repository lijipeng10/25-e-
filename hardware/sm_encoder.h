#ifndef SM_ENCODER_H
#define SM_ENCODER_H

#include <stdint.h>

/* MT6816 编码器驱动(两轴)
 * 轴0 = 电机1(Pan), 轴1 = 电机2(Tilt)
 * 用 A/B GPIO 中断做软件四倍频计数(4096 计数/圈), Z 做零点, M法测速。
 */

/* 初始化: 使能 GPIO 中断(GROUP1), 清状态 */
void encoder_init(void);

/* 四倍频计数(带符号), 相对 Z 零点 */
int32_t encoder_get_count(uint8_t axis);

/* 绝对角度(0.1 度), 需要先过 Z 才有意义 */
int32_t encoder_get_angle_x10(uint8_t axis);

/* 是否已见过 Z(拿到绝对零点基准) */
uint8_t encoder_isZeroSeen(uint8_t axis);

/* M 法测速: 必须每 10ms 调用一次, 它会采样计数并算转速 */
void encoder_sample_speed(void);

/* 转速(RPM, 带符号), 由 encoder_sample_speed 每 10ms 刷新 */
int32_t encoder_get_speed_rpm(uint8_t axis);

/* 诊断: GROUP1_IRQHandler 被触发次数 */
uint32_t encoder_get_isr_count(void);

#endif /* ENCODER_H */
