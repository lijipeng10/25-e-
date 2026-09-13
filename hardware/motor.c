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

/* 百分比(0~100) -> 定时器比较值
 *
 * ★★★ 这里的式子必须是"反"的, 写成 duty*PERIOD/100 是错的! ★★★
 *
 * MSPM0 的 TimerG 在 EDGE_ALIGN PWM 模式下是【从 LOAD 往下数】的,
 * CCP 输出在"计数 > 比较值"这段时间里为高, 所以
 *
 *      实际占空比 = (周期 - 比较值) / 周期
 *
 * 也就是说 —— 比较值越大, 占空比越小。TI 自己的 SysConfig 就是这么算的,
 * 见 source/ti/driverlib/.meta/pwm/PWMTimerCC.syscfg.js 第 107 行:
 *      proposedccValue = Math.round((100 - inst.dutyCycle) * period / 100) - 1;
 *
 * 也可以看官方例子 examples/.../timx_timer_mode_pwm_edge_sleep:
 *      timerCount = 2000, dutyCycle = 75  ->  ccValue = 500
 *      (如果用 (100-75)% * 2000 = 500, 反过来说 500 对应 75%; 正着写会得到 1500)
 *
 * 【之前写反了的后果——很严重, 不是"速度不对"这么简单】
 *      填 20 -> 实际输出 80%      填 90 -> 实际输出 10%   ("数值越大越慢")
 *      而且循迹的差速是【反的】:
 *          想让左轮快 -> left_cmd 变大 -> 比较值变大 -> 左轮实际更慢
 *      变成正反馈, 车会朝着偏离方向越走越远。
 *      50 是唯一的对称点, 所以"填 50 看起来是对的", 很容易被蒙过去。 */
static uint16_t motor_duty_to_cmp(uint16_t duty)
{
    if (duty > 100U) duty = 100U;
    return (uint16_t)(((uint32_t)(100U - duty) * MOTOR_PWM_PERIOD) / 100U);
}

void motor_init(void)
{
    // 1) 方向脚先清零(停止)。顺序很重要: 一定要在使能驱动器之前,
    //    否则 TB6612 会在方向脚还没定的瞬间吃到 PWM。
    DL_GPIO_clearPins(motor_AIN1_PORT, motor_AIN1_PIN);
    DL_GPIO_clearPins(motor_AIN2_PORT, motor_AIN2_PIN);
    DL_GPIO_clearPins(motor_BIN1_PORT, motor_BIN1_PIN);
    DL_GPIO_clearPins(motor_BIN2_PORT, motor_BIN2_PIN);

    // 2) 占空比清零。
    //    ★ 注意: 这里必须用 motor_duty_to_cmp(0), 不能直接写 0!
    //      直接写 0 的比较值 = 100% 占空比(见上面函数的说明),
    //      上电瞬间就是全速, 很危险。
    DL_Timer_setCaptureCompareValue(motor_pwm_INST, motor_duty_to_cmp(0U), GPIO_motor_pwm_C0_IDX);
    DL_Timer_setCaptureCompareValue(motor_pwm_INST, motor_duty_to_cmp(0U), GPIO_motor_pwm_C1_IDX);

    // 3) 开定时器
    DL_Timer_startCounter(motor_pwm_INST);

    // 4) 最后才使能电机驱动器
    DL_GPIO_setPins(motor_STBY_PORT, motor_STBY_PIN);
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
