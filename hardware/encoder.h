#ifndef ENCODER_H
#define ENCODER_H

/*
    //ENCODER PIN MAP//
    E1A <==> B20
    E1B <==> A14
    E2A <==> A25
    E2B <==> B25

    //测速参数
    编码器线数13线
    电机减速比1:20
    一圈260个脉冲
*/

#define PI                      3.14
#define ENCODER_PULSE           260
#define ENCODER_WHEEL_D         48   //mm

#include "ti_msp_dl_config.h"

void encoder_init(void);
void encoder_get_speed(uint8_t id);

/* 原始脉冲计数, 由 GROUP1 中断累加(中断处理在本模块 encoder.c 里) */
extern uint32_t encoder_1_A;
extern uint32_t encoder_2_A;

/* 实测速度 mm/s, 由 encoder_get_speed() 每 50ms 刷新 */
extern float speed_1;
extern float speed_2;


#endif /* ENCODER_H */
