#include "motor.h"

/* ============================================================================
 *  motor.c —— TB6612 双直流电机驱动
 * ----------------------------------------------------------------------------
 *  引脚(SysConfig 生成宏, 注意是小写 motor_*):
 *      PWM : motor_pwm_INST = TIMG8  CC0=PB6(PWMA), CC1=PB7(PWMB)
 *      A路 : motor_AIN1=PB17, motor_AIN2=PB18
 *      B路 : motor_BIN1=PB19, motor_BIN2=PB23
 *      STBY= motor_STBY=PA16 (高=使能)
 *  说明: motor_pwm 是 TIMG8(TimerG), 所以用通用的 DL_Timer_* 接口,
 *        不要用 DL_TimerA_*(那是 TimerA 专用)。
 *  duty: 对外统一是百分比 0~100, 由 MOTOR_PWM_PERIOD 自动换算成定时器比较值,
 *        所以以后把 SysConfig 里 timerCount 改成 1000(20kHz) 也不用改这里。
 *        (line_follow.c 里的 LF_DUTY_MAX 是它自己的量程, 与本文件无关)
 * ==========================================================================*/
#include "ti_msp_dl_config.h"

/* motor_pwm(TIMG8) 的计数周期 = timerCount(见 SysConfig: PWM3.timerCount = 1000)。
   PWM 频率 = 20MHz / (timerCount+1) ≈ 20kHz(避开 TB6612 上限, 也不在音频段)。
   ⚠ 改 syscfg 的 timerCount 后, 这里必须同步改, 否则占空比会按比例失真。 */
#define MOTOR_PWM_PERIOD    1000U

/* 百分比(0~100) -> 定时器比较值 */
static uint16_t motor_duty_to_cmp(uint16_t duty)
{
    if (duty > 100U) duty = 100U;
    return (uint16_t)(((uint32_t)duty * MOTOR_PWM_PERIOD) / 100U);
}

void motor_init(void)
{
    // 使能电机驱动器
    DL_GPIO_setPins(motor_STBY_PORT, motor_STBY_PIN);
    // 先把占空比清零再启动, 防止上电就全速
    DL_Timer_setCaptureCompareValue(motor_pwm_INST, 0, GPIO_motor_pwm_C0_IDX);
    DL_Timer_setCaptureCompareValue(motor_pwm_INST, 0, GPIO_motor_pwm_C1_IDX);
    // 开启定时器
    DL_Timer_startCounter(motor_pwm_INST);
    // 方向脚清零(停止)
    DL_GPIO_clearPins(motor_AIN1_PORT, motor_AIN1_PIN);
    DL_GPIO_clearPins(motor_AIN2_PORT, motor_AIN2_PIN);
    DL_GPIO_clearPins(motor_BIN1_PORT, motor_BIN1_PIN);
    DL_GPIO_clearPins(motor_BIN2_PORT, motor_BIN2_PIN);
}

void motor_set_direction(uint8_t id, uint8_t direction)
{
    if(id == 1)
    {
        if(direction == 1)       // 正转
        {
            DL_GPIO_clearPins(motor_AIN1_PORT, motor_AIN1_PIN);
            DL_GPIO_setPins(motor_AIN2_PORT, motor_AIN2_PIN);
        }
        else if(direction == 2)  // 反转
        {
            DL_GPIO_setPins(motor_AIN1_PORT, motor_AIN1_PIN);
            DL_GPIO_clearPins(motor_AIN2_PORT, motor_AIN2_PIN);
        }
        else                     // 停止
        {
            DL_GPIO_clearPins(motor_AIN1_PORT, motor_AIN1_PIN);
            DL_GPIO_clearPins(motor_AIN2_PORT, motor_AIN2_PIN);
        }
    }
    else if(id == 2)
    {
        if(direction == 1)
        {
            DL_GPIO_setPins(motor_BIN1_PORT, motor_BIN1_PIN);
            DL_GPIO_clearPins(motor_BIN2_PORT, motor_BIN2_PIN);
        }
        else if(direction == 2)
        {
            DL_GPIO_clearPins(motor_BIN1_PORT, motor_BIN1_PIN);
            DL_GPIO_setPins(motor_BIN2_PORT, motor_BIN2_PIN);
        }
        else
        {
            DL_GPIO_clearPins(motor_BIN1_PORT, motor_BIN1_PIN);
            DL_GPIO_clearPins(motor_BIN2_PORT, motor_BIN2_PIN);
        }
    }
}

void motor_set_duty(uint8_t id, uint16_t duty)   /* duty: 百分比 0~100 */
{
    uint16_t cmp = motor_duty_to_cmp(duty);

    if(id == 1)
    {
        DL_Timer_setCaptureCompareValue(motor_pwm_INST, cmp, GPIO_motor_pwm_C0_IDX);
    }
    else if(id == 2)
    {
        DL_Timer_setCaptureCompareValue(motor_pwm_INST, cmp, GPIO_motor_pwm_C1_IDX);
    }
}

void motor_stop(uint8_t id)
{
    motor_set_direction(1, 0);
    motor_set_direction(2, 0);
    motor_set_duty(1, 0);
    motor_set_duty(2, 0);
}
