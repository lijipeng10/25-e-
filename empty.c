#include "ti_msp_dl_config.h"
#include <stdint.h>
#include "key.h"            /* key_init / key_tick / key_getnum */
#include "motor.h"          /* motor_init / motor_set_direction / motor_set_duty */
#include "encoder.h"        /* encoder_get_speed */
#include "oled.h"
#include "tick.h"           /* tick_init / tick_get_ms */

uint8_t keynum;
extern uint32_t encoder_1_A;
extern uint32_t encoder_2_A;
float speed_1 = 0;
float speed_2 = 0;

/* KEY1 阶梯加速: 每按一次升一档占空比 */
static const uint16_t duty_table[] = { 0U, 100U, 200U, 300U, 400U, 500U, 600U, 700U, 800U, 900U };
static u8 duty_index = 0U;

//OLED 显示
static void show_enc(void)
{
    static u32 last = 0;

    if ((tick_get_ms() - last) < 100U)
    {
        return;
    }
    last = tick_get_ms();

    OLED_ShowString(0, 0, (u8 *)"ENCODER", 16);
    OLED_ShowNum(80, 0, duty_table[duty_index], 4, 16);

    OLED_ShowString(0, 16, (u8 *)"E1", 12);
    OLED_ShowNum(24, 16, encoder_1_A, 6, 12);

    OLED_ShowString(0, 28, (u8 *)"E2", 12);
    OLED_ShowNum(24, 28, encoder_2_A, 6, 12);

    OLED_ShowString(0, 40, (u8 *)"S1", 12);
    OLED_ShowNum(24, 40, (u32)((speed_1 < 0) ? -speed_1 : speed_1), 5, 12);

    OLED_ShowString(0, 52, (u8 *)"S2", 12);
    OLED_ShowNum(24, 52, (u32)((speed_2 < 0) ? -speed_2 : speed_2), 5, 12);

    OLED_Refresh();
}

int main(void)
{
    SYSCFG_DL_init();
    motor_init();
    key_init();             /* 配 key_encoder 定时器(周期模式 + ZERO 中断) */

    encoder_init();
    OLED_Init();
    tick_init();

    OLED_Clear();           /* 清屏只做一次, 放循环里会闪 */

    while (1)
    {
        keynum = key_getnum();

        if (keynum == 1U)
        {
            duty_index++;               /* 下一档 */

            if (duty_index >= (u8)(sizeof(duty_table) / sizeof(duty_table[0])))
            {
                duty_index = 0U;        /* 到头回到 0 */
            }

            motor_set_direction(1, 1);
            motor_set_direction(2, 1);
            motor_set_duty(1, duty_table[duty_index]);
            motor_set_duty(2, duty_table[duty_index]);
        }

        show_enc();
    }
}

void GROUP1_IRQHandler(void)
{
    switch (DL_GPIO_getPendingInterrupt(GPIOB))
    {
        case encoder_E1A_IIDX:
            encoder_1_A++;
            encoder_2_A++;
            break;

        default:
            break;
    }
}

void key_encoder_INST_IRQHandler(void)
{
    DL_Timer_clearInterruptStatus(key_encoder_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);

    key_tick();                 /* 按键扫描(和测速共用这个 50ms 定时器) */

    encoder_get_speed(1);
    encoder_get_speed(2);
}
