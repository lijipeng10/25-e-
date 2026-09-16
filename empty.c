#include "ti_msp_dl_config.h"
#include <stdint.h>
#include "key.h"            /* key_init / key_tick / key_getnum */
#include "motor.h"          /* motor_init / motor_set_direction / motor_set_duty */
#include "encoder.h"          /* encoder_get_speed */
#include "oled.h"
#include "tick.h"           /* tick_init / tick_get_ms */

uint8_t keynum;
extern uint32_t encoder_1_A;
extern uint32_t encoder_2_A;
float speed_1 = 0;
float speed_2 = 0;

/* ============================================================================
 *  OLED 显示 —— 省资源的写法(两招)
 * ----------------------------------------------------------------------------
 *  原来每圈调一次 OLED_Refresh(): 要发 8 页 = 3*8 命令 + 128*8 数据 = 1048 字节,
 *  外加 16 次 SPI flush。SPI 再快也是毫秒级, 每圈都干就把主循环拖住了。
 *
static void show_enc(void)
{
    OLED_Clear();
    OLED_ShowString(0,  0, (u8 *)"ENCODER", 16);

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
    key_init();         /* 配 key_encoder 定时器(周期模式 + ZERO 中断) */

    encoder_init();
    OLED_Init();

    while (1)
    {
        show_enc();
    }
}

void GROUP1_IRQHandler(void)
{
    switch (DL_GPIO_getPendingInterrupt(GPIOB))
    {
        case encoder_E1A_IIDX:
            encoder_1_A ++;
            encoder_2_A ++;
            break;

        default:
            break;

    }
}
void key_encoder_INST_IRQHandler(void)
{
    DL_Timer_clearInterruptStatus(key_encoder_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);

    key_tick();             /* 扫按键 */

    encoder_get_speed(1);   /* 测速, 并把 encoder_1_A 清零 */
    encoder_get_speed(2);
}