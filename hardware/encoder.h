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
#include <stdint.h>

#define PI                      3.14
#define ENCODER_PULSE           260
#define ENCODER_WHEEL_D         48   //mm

/* 注意: 改名成 wheel_encoder_* , 避免与 sm_encoder.c 里的 encoder_init 重复定义 */
void wheel_encoder_init(void);
void wheel_encoder_get_speed(uint8_t id);

/* 轮速(mm/s), 由 wheel_encoder_get_speed() 刷新 */
extern float speed_1;
extern float speed_2;

#endif // ENCODER_H
