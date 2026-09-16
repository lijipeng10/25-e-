#include "pid.h"
#include "motor.h"
#include "encoder.h"

/* 速度环增益。增量式: out += KP*(e-e_last) + KI*e
   KI 绝对不能是 0 —— 纯 P 的话稳态永远差一截, 速度永远追不上目标(实测踩过) */
#define motor_KP        0.5f
#define motor_KI        0.5f
#define motor_KD        0.0f

#define motor_DUTY_MAX  900.0f

/* ★ 编码器故障保护: 连续 10 拍(= 10 x 50ms = 0.5s)
   "占空比顶到上限、编码器却一个脉冲都没有" 就判故障停车。
   不拦的话 PID 会把占空比一直顶在最大, 车直接冲出去(实测踩过: "一给速度就全速")。 */
#define motor_FAULT_TICKS   10U

static PidInc s_pid[2];         /* 两个轮子各自的 PID 状态 */
static float  s_target[2];      /* 两个轮子各自的目标速度 (mm/s) */
static uint8_t s_fault_cnt[2];  /* 每个轮子连续"顶死却不动"的拍数 */
static uint8_t s_fault;         /* 1 = 已判故障, 自锁 —— 要 motor_fault_clear() 才解 */

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
        if(direction == 1) // 正转(车往前走) —— ★ 实测: A路(左轮)必须用这一组, 原工程那组是倒转的
        {
            DL_GPIO_setPins(motor_AIN1_PORT, motor_AIN1_PIN);
            DL_GPIO_clearPins(motor_AIN2_PORT, motor_AIN2_PIN);
        }
        else if(direction == 2) // 反转
        {
            DL_GPIO_clearPins(motor_AIN1_PORT, motor_AIN1_PIN);
            DL_GPIO_setPins(motor_AIN2_PORT, motor_AIN2_PIN);
        }
        else // 停止
        {
            DL_GPIO_clearPins(motor_AIN1_PORT, motor_AIN1_PIN);
            DL_GPIO_clearPins(motor_AIN2_PORT, motor_AIN2_PIN);
        }
    }
    else if(id == 2)
    {
        if(direction == 1) // 正转(车往前走) —— ★ 实测: B路(右轮)用这一组才对, 【不要】再对调
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
        s_fault_cnt[i] = 0U;
    }

    s_fault = 0U;
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

/* 两个轮子立刻全停 + 置故障标志。★ 在定时器中断里被调用, 只做寄存器操作, 不带延时 */
static void motor_stop_all(void)
{
    s_target[0] = 0.0f;
    s_target[1] = 0.0f;
    pid_reset(&s_pid[0]);
    pid_reset(&s_pid[1]);
    s_fault_cnt[0] = 0U;
    s_fault_cnt[1] = 0U;
    motor_set_direction(1U, 0U);
    motor_set_direction(2U, 0U);
    motor_set_duty(1U, 0U);
    motor_set_duty(2U, 0U);
    s_fault = 1U;
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

    if (s_fault != 0U)
    {
        return;                                 /* 故障自锁: 不 clear 谁也别想再转 */
    }

    if (s_target[idx] == 0.0f)
    {
        return;                                 /* 目标为 0 就不动它(set 里已经给过 0) */
    }

    now = (idx == 0U) ? speed_1 : speed_2;      /* 实测 mm/s, 来自 encoder.c */
    out = pid_update(&s_pid[idx], s_target[idx] - now);

    /* ★ 编码器故障判据: 占空比已经顶到上限, 轮子却一个脉冲都没有 */
    if ((out >= (motor_DUTY_MAX - 1.0f)) && (now < 1.0f))
    {
        s_fault_cnt[idx]++;

        if (s_fault_cnt[idx] >= motor_FAULT_TICKS)
        {
            motor_stop_all();
            return;
        }
    }
    else
    {
        s_fault_cnt[idx] = 0U;
    }

    motor_set_duty(id, (uint16_t)out);
}

uint8_t motor_is_fault(void)
{
    return s_fault;
}

void motor_fault_clear(void)
{
    s_fault = 0U;
    s_fault_cnt[0] = 0U;
    s_fault_cnt[1] = 0U;
}

/* ---------------------------------------------------------------------------
 *  KEY1 阶梯加速测速
 *
 *  原来这段(speed_table / speed_index / 按键加档逻辑)写在 empty.c 的 main 里,
 *  主程序太冗余。档位表和加档逻辑本来就属于电机模块, 搬到这里。
 *
 *  ★ 档位表和下标都是 static: 外面只能"按一下走一档"(motor_test_step()),
 *    或者读当前档位值去显示(motor_test_duty()), 改不了这张表。
 *  ★ 档位值沿用原来那一张表, 一个数都没动。
 * -------------------------------------------------------------------------*/
static const uint16_t s_test_table[] =
    { 0U, 100U, 200U, 300U, 400U, 500U, 600U, 700U, 800U, 900U };
static uint8_t s_test_index = 0U;      /* 当前档位下标, 记录在这里给屏幕读 */

void motor_test_step(void)
{
    s_test_index++;                     /* 下一档 */

    if (s_test_index >= (uint8_t)(sizeof(s_test_table) / sizeof(s_test_table[0])))
    {
        s_test_index = 0U;              /* 到头回到 0 */
    }

    motor_set_direction(1, 1);
    motor_set_direction(2, 1);

    motor_pid_set(1, (float)s_test_table[s_test_index]);
    motor_pid_set(2, (float)s_test_table[s_test_index]);
}

uint16_t motor_test_duty(void)
{
    return s_test_table[s_test_index];
}
