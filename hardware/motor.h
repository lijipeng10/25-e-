#ifndef MOTOR_H
#define MOTOR_H

/*
    //MOTOR PIN MAP//
    PWMA <==> B06
    PWMB <==> B07
    AIN1 <==> B17
    AIN2 <==> B18
    BIN1 <==> B19
    BIN2 <==> B23
    STBY <==> A16
*/

#include "ti_msp_dl_config.h"

void motor_init(void);
void motor_set_direction(uint8_t id, uint8_t direction);
void motor_set_duty(uint8_t id, uint16_t duty);
void motor_pid_init(void);
void motor_pid_set(uint8_t id, float target_mm_s);
void motor_pid_update(uint8_t id);


#endif // MOTOR_H
