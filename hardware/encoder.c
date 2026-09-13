#include "encoder.h"
#include "ti_msp_dl_config.h"

/* 编码器计数值(由 E1A/E1B 中断累加, 见下方注释) */
uint32_t encoder_1_A = 0;
uint32_t encoder_2_A = 0;

float speed_1 = 0;
float speed_2 = 0;

void wheel_encoder_init(void)
{
    /* 说明: SysConfig 里 encoder(GPIO7) 的 E1A/E1B 目前没有使能 GPIO 中断,
       所以这里不能开 NVIC(否则要用 GROUP1_IRQHandler 计数)。
       需要轮速时: 在 SysConfig 给 E1A/E1B 勾上 interruptEn(双沿),
       再打开下面的 NVIC_EnableIRQ, 并在 GROUP1_IRQHandler 里累加计数。 */
    // NVIC_EnableIRQ(encoder_GPIOA_INT_IRQN);

    /* 说明: main_timer 由 key_init() 统一配置和启动(周期模式 + 中断),
       这里不要再碰它, 否则会和按键扫描抢同一个定时器/中断向量。
       需要轮速采样时, 在主循环里定时调用 wheel_encoder_get_speed() 即可。 */
}

void wheel_encoder_get_speed(uint8_t id)
{
    if(id == 1)
    {
        speed_1 = (float)encoder_1_A / ENCODER_PULSE * ENCODER_WHEEL_D * PI * 20;   // mm/s
        encoder_1_A = 0;
    }
    else if(id == 2)
    {
        speed_2 = (float)encoder_2_A / ENCODER_PULSE * ENCODER_WHEEL_D * PI * 20;   // mm/s
        encoder_2_A = 0;
    }
}
