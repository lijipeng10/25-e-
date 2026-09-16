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

/* ★ 由 GROUP1 中断累加的原始脉冲计数(在 sm_encoder.c 里加) */
extern uint32_t encoder_1_A;    /* E1A = PB20 */
extern uint32_t encoder_2_A;    /* E2A = PA25 */
/* ★ 实测速度 mm/s, 由 encoder_get_speed() 刷新 */
extern float    speed_1;
extern float    speed_2;


#endif /* ENCODER_H */
