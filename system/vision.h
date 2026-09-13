#ifndef VISION_H
#define VISION_H

#include <stdint.h>

/* ============================================================================
 *  vision.h —— 视觉串口接口模块(二进制帧 + PID 视觉伺服)
 * ----------------------------------------------------------------------------
 *  职责: 从视觉串口(UART1/smotor_vision) 收二进制帧
 *        [0xAA][0x55][pan_lo][pan_hi][tilt_lo][tilt_hi][XOR校验]
 *        解析出两轴偏差(0.1°, 带符号), 做速度输出 PID 驱动云台回中。
 *  用法(主函数只调这几个):
 *     tick_init();     // 建 1ms 时基(供 PID dt / 超时)
 *     vision_init();   // 初始化视觉串口接收
 *     ... 主循环里 ...
 *     vision_poll();   // 收帧->解析->PID->驱动电机(每循环调一次)
 * -------------------------------------------------------------------------*/

void vision_init(void);         /* 初始化: 开启视觉串口接收, 清状态 */

/* 主循环调用: 每收到一帧就解析并做 PID 驱动两轴电机 */
void vision_poll(void);

/* 最近一次收到的偏差(0.1度, 带符号), 调试用 */
int32_t vision_get_pan(void);
int32_t vision_get_tilt(void);

/* 已成功接收并执行的命令帧数(调试/确认通信) */
uint32_t vision_get_frames(void);

/* PC 调试串口(UART2)打印一个字符串(阻塞)。不含视觉数据, 可用于心跳/状态输出 */
void pc_uart_print_str(const char *s);

#endif /* VISION_H */
