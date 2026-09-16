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

/* KEY1 阶梯加速测速: 内部把档位加一档(到头回 0), 给两个轮子设方向和目标速度。
 * ★ 档位表和下标都是 motor.c 里的 static, 外面看不见, 只能读 motor_test_duty() */
void     motor_test_step(void);

/* 当前档位的目标速度 (mm/s), 单位与 motor_pid_set() 一致, 给屏幕显示用 */
uint16_t motor_test_duty(void);


#endif // MOTOR_H
