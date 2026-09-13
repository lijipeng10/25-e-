#ifndef SM_MOTOR_H
#define SM_MOTOR_H

#include <stdint.h>
#include "ti_msp_dl_config.h"
#include "test_config.h"

/* 电机模块初始化: 使能电机 */
void sm_motor_Init(void);

/* 启动电机计数器 */
void sm_motor_Start(void);

/* 设置电机转动方向
 * id: 电机编号(1 = 电机1 用 DIR1, 2 = 电机2 用 DIR2); direction: 1/0 */
void sm_motor_Set_Direction(uint16_t id, uint16_t direction);

/* 设置某路电机 STEP PWM 的占空比
 * id: 1 = CC0, 2 = CC1; Duty: 0~100(%) */
void sm_motor_Set_Duty(uint16_t id, uint16_t Duty);

/* 以 rpm 让电机持续转动(STEP 50% 占空比)
 * id: 1 = CC0(PA28), 2 = CC1(PA31); rpm: 转速(需 PWM 时钟 1MHz 以支持低速) */
void sm_motor_Run(uint16_t id, uint16_t rpm);

/* 平滑转到指定角度(带加减速, 到位自动停)
 * id: 1/2; angle_deg: 目标角度(度, 正=正向, 负=反向); rpm: 巡航转速 */
void sm_motor_MoveTo(uint16_t id, int32_t angle_deg, uint16_t rpm);

/* 立即停止电机 */
void sm_motor_Stop(uint16_t id);

/* 查询某轴是否正在走位(结合式闭环用) */
uint8_t sm_motor_IsMoving(uint16_t id);

/* 持续速度控制(闭环PID用), rpm 带符号(正/负=两方向, 0=停) */
void sm_motor_SetSpeed(uint16_t id, int32_t rpm);

/* 画圆: 两轴按 sin/cos 连续转动(一直转)
 * radius_x10: 半径(0.1度, 如 150=15.0°); period_ms: 一圈毫秒数 */
void sm_motor_CircleScan(int16_t radius_x10, uint16_t period_ms);

#endif
