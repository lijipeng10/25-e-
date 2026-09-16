#include "encoder.h"
#include "ti_msp_dl_config.h"
#include "motor.h"
#include "pid.h"
#include "tick.h"
#include "key.h"            /* key_tick: 这个定时器同时也是按键扫描时基 */

uint32_t encoder_1_A;
uint32_t encoder_2_A;
float speed_1 = 0;
float speed_2 = 0;

void encoder_init(void)
{
    /* 只做一件事: 使能 GPIO 中断(GROUP1)。
     * GPIOA 和 GPIOB 共用 IRQ 1, 使能一次两边都通, 所以 E1A/E2A 都能进中断。 */
    NVIC_ClearPendingIRQ(encoder_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(encoder_GPIOB_INT_IRQN);

    /* ★ 定时器(key_encoder = TIMG7)由 key_init() 统一配置和启动, 这里【不要再碰】——
     *   两个地方都去配同一个定时器会互相打架。 */
}

void encoder_get_speed(uint8_t id)
{
    if (id == 1)
    {
        speed_1 = (float)encoder_1_A / ENCODER_PULSE * ENCODER_WHEEL_D * PI * 20;   /* mm/s */
        encoder_1_A = 0;    /* 用掉就清零, 下个周期重新攒 */
    }
    else if (id == 2)
    {
        speed_2 = (float)encoder_2_A / ENCODER_PULSE * ENCODER_WHEEL_D * PI * 20;
        encoder_2_A = 0;
    }
}

/* ============================================================================
 *  定时器中断(key_encoder = TIMG7, 50ms) —— 测速就靠它
 * ----------------------------------------------------------------------------
 *  ★★ 这个函数【必须存在】★★
 *    key_init() 里 NVIC_EnableIRQ(key_encoder_INST_INT_IRQN) 把中断打开了;
 *    如果没有人处理, 中断会跳到启动文件里的【弱定义默认处理】(死循环) ——
 *    结果是刚跑起来就卡死, 而且【烧录不上】。原版这个函数被注释掉了, 就是这个后果。
 *
 *  ★ 这个定时器【同时】给按键扫描和测速用, 所以两个都要调。
 *  ★ 中断标志必须清, 否则会反复重进。
 *  ★ 函数名用 SysConfig 生成的 key_encoder_INST_IRQHandler(= TIMG7_IRQHandler),
 *    以后在 syscfg 里改名字, 这里会自动跟着变。
 * ==========================================================================*/
void key_encoder_INST_IRQHandler(void)
{
    DL_Timer_clearInterruptStatus(key_encoder_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);

    key_tick();             /* 扫按键(在 key.c 里) */

    encoder_get_speed(1);   /* 测速: 用掉这 50ms 攒的脉冲, 算出 mm/s */
    encoder_get_speed(2);
}
