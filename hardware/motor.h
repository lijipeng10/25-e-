#ifndef MOTOR_H
#define MOTOR_H

#include "ti_msp_dl_config.h"

void motor_init(void);
void motor_set_direction(uint8_t id, uint8_t direction);
void motor_set_duty(uint8_t id, uint16_t duty);
void motor_stop(uint8_t id);

#endif // MOTOR_H
