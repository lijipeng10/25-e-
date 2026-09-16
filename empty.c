#include "ti_msp_dl_config.h"
#include <stdint.h>
#include "key.h"            /* key_init / key_tick / key_getnum */
#include "motor.h"          /* motor_init / motor_set_direction / motor_set_duty */
#include "encoder.h"          /* encoder_get_speed */
#include "oled.h"

uint8_t keynum;
extern uint32_t encoder_1_A;
extern uint32_t encoder_2_A;
float speed_1 = 0;
float speed_2 = 0;

static void show_enc(void)
{
    OLED_Clear();

    OLED_ShowString(0,  0, (u8 *)"ENCODER", 16);

    OLED_ShowString(0, 16, (u8 *)"E1", 12);
    OLED_ShowNum(24, 16, encoder_1_A, 6, 12);      /* E1A(PB20) 脉冲数 */

    OLED_ShowString(0, 28, (u8 *)"E2", 12);
    OLED_ShowNum(24, 28, encoder_2_A, 6, 12);      /* E2A(PA25) 脉冲数 */

    /* speed_1/2 是 float, OLED 只能画整数, 所以转一下(取了绝对值) */
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

    /* ★★ 必须调 key_init() ★★
     *  它才是把 key_encoder 定时器【配成周期模式 + 使能 ZERO 事件中断】的地方。
     *  encoder_init() 里只有"启动计数器 + 开 NVIC", 光靠它中断永远不会来 ——
     *  表现就是 encoder_1_A/2_A 只涨不清、speed 恒 0。 */
    key_init();

    encoder_init();
    OLED_Init();

    show_enc();

    while (1)
    {
        
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

/* ============================================================================
 *  定时器中断(key_encoder = TIMG7, 50ms) —— 测速靠它
 * ----------------------------------------------------------------------------
 *  ★ 原来的写法有两个问题:
 *    1) case 写的是 DL_TIMER_IIDX_LOAD, 但 key_init() 里使能的是 ZERO 事件 ——
 *       对不上, switch 一个都不匹配, 什么也不做, 而且中断标志没清 -> 反复重进。
 *       改成【无条件清 ZERO 标志】: 只有这一个中断源, 不需要判断。
 *    2) 少调了 key_tick() —— 这个定时器【同时】是按键扫描时基, 不调按键就没反应。
 * ==========================================================================*/
void key_encoder_INST_IRQHandler(void)
{
    DL_Timer_clearInterruptStatus(key_encoder_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);

    key_tick();             /* 扫按键 */

    encoder_get_speed(1);   /* 测速, 并把 encoder_1_A 清零 */
    encoder_get_speed(2);
}