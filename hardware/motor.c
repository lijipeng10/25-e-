#include "pid.h"
#include "motor.h"
#include "encoder.h"

/* 速度环增益。增量式: out += KP*(e-e_last) + KI*e
   KI 绝对不能是 0 —— 纯 P 的话稳态永远差一截, 速度永远追不上目标(实测踩过) */
#define motor_KP        0.5f
#define motor_KI        0.5f
#define motor_KD        0.0f

#define motor_DUTY_MAX  900.0f

static PidInc s_pid[2];         /* 两个轮子各自的 PID 状态 */
static float  s_target[2];      /* 两个轮子各自的目标速度 (mm/s) */

void motor_init(void)
{
    // 使能电机驱动器
    DL_GPIO_setPins(motor_STBY_PORT, motor_STBY_PIN);
    // 开启定时器
    DL_Timer_startCounter(motor_pwm_INST);
    // 设置AIN1和AIN2引脚下拉
    DL_GPIO_clearPins(motor_AIN1_PORT, motor_AIN1_PIN);
    DL_GPIO_clearPins(motor_AIN2_PORT, motor_AIN2_PIN);
    DL_GPIO_clearPins(motor_BIN1_PORT, motor_BIN1_PIN);
    DL_GPIO_clearPins(motor_BIN2_PORT, motor_BIN2_PIN);
    // 设置PWM占空比
    DL_TimerA_setCaptureCompareValue(motor_pwm_INST, 0, GPIO_motor_pwm_C0_IDX);
    DL_TimerA_setCaptureCompareValue(motor_pwm_INST, 0, GPIO_motor_pwm_C1_IDX);
}

void motor_set_direction(uint8_t id, uint8_t direction)
{
    // 设置电机方向
    if(id == 1)
    {
        if(direction == 1) // 正转
        {
            DL_GPIO_clearPins(motor_AIN1_PORT, motor_AIN1_PIN);
            DL_GPIO_setPins(motor_AIN2_PORT, motor_AIN2_PIN);
        }
        else if(direction == 2) // 反转
        {
            DL_GPIO_setPins(motor_AIN1_PORT, motor_AIN1_PIN);
            DL_GPIO_clearPins(motor_AIN2_PORT, motor_AIN2_PIN);
        }
        else // 停止
        {
            DL_GPIO_clearPins(motor_AIN1_PORT, motor_AIN1_PIN);
            DL_GPIO_clearPins(motor_AIN2_PORT, motor_AIN2_PIN);
        }
    }
    else if(id == 2)
    {
        if(direction == 1) // 正转
        {
            DL_GPIO_setPins(motor_BIN1_PORT, motor_BIN1_PIN);
            DL_GPIO_clearPins(motor_BIN2_PORT, motor_BIN2_PIN);
        }
        else if(direction == 2) // 反转
        {
            DL_GPIO_clearPins(motor_BIN1_PORT, motor_BIN1_PIN);
            DL_GPIO_setPins(motor_BIN2_PORT, motor_BIN2_PIN);
        }
        else // 停止
        {
            DL_GPIO_clearPins(motor_BIN1_PORT, motor_BIN1_PIN);
            DL_GPIO_clearPins(motor_BIN2_PORT, motor_BIN2_PIN);
        }
    }
}

void motor_set_duty(uint8_t id, uint16_t duty)
{
    // 设置电机占空比
    if(id == 1)
    {
        DL_TimerA_setCaptureCompareValue(motor_pwm_INST, duty, GPIO_motor_pwm_C0_IDX);
    }
    else if(id == 2)
    {
        DL_TimerA_setCaptureCompareValue(motor_pwm_INST, duty, GPIO_motor_pwm_C1_IDX);
    }
}

void motor_pid_init(void)
{
    uint8_t i;

    for (i = 0; i < 2U; i++)
    {
        pid_init(&s_pid[i], motor_KP, motor_KI, motor_KD, 0.0f, motor_DUTY_MAX);
        s_target[i] = 0.0f;
    }
}

void motor_pid_set(uint8_t id, float target_mm_s)
{
    if ((id < 1U) || (id > 2U))
    {
        return;
    }

    s_target[id - 1U] = target_mm_s;

    if (target_mm_s == 0.0f)
    {
        pid_reset(&s_pid[id - 1U]);
        motor_set_duty(id, 0U);
    }
}

/* 每 50ms 调一次, id = 1 或 2, 两个电机各自独立 */
void motor_pid_update(uint8_t id)
{
    float   now, out;
    uint8_t idx;

    if ((id < 1U) || (id > 2U))
    {
        return;
    }

    idx = (uint8_t)(id - 1U);

    if (s_target[idx] == 0.0f)
    {
        return;                                 /* 目标为 0 就不动它(set 里已经给过 0) */
    }

    now = (idx == 0U) ? speed_1 : speed_2;      /* 实测 mm/s, 来自 encoder.c */
    out = pid_update(&s_pid[idx], s_target[idx] - now);

    motor_set_duty(id, (uint16_t)out);
}
