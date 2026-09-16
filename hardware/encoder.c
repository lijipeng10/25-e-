#include "encoder.h"
#include "ti_msp_dl_config.h"
#include "motor.h"
#include "pid.h"
#include "tick.h"

uint32_t encoder_1_A;
uint32_t encoder_2_A;

/* 实测速度 mm/s。定义放在本模块里(原来定义在 empty.c, 声明却在这里,
   主程序不该持有电机测速的状态) —— encoder.h 里只有 extern 声明。 */
float speed_1 = 0;
float speed_2 = 0;

void encoder_init(void)
{
    /* 只开 GPIO 中断(GROUP1)。GPIOA/GPIOB 共用 IRQ 1, 使能一次两边都通。
       定时器(key_encoder = TIMG7)由 key_init() 统一配置和启动, 这里不要再碰 ——
       两个地方都配同一个定时器会打架。 */
    NVIC_ClearPendingIRQ(encoder_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(encoder_GPIOB_INT_IRQN);
}

void encoder_get_speed(uint8_t id)
{
    if (id == 1)
    {
        speed_1 = (float)encoder_1_A / ENCODER_PULSE * ENCODER_WHEEL_D * PI * 20;   /* mm/s */
        encoder_1_A = 0;                                                            /* 清零 */
    }
    else if (id == 2)
    {
        speed_2 = (float)encoder_2_A / ENCODER_PULSE * ENCODER_WHEEL_D * PI * 20;   /* mm/s */
        encoder_2_A = 0;                                                            /* 清零 */
    }
}

/* 编码器脉冲计数中断。GPIOA 和 GPIOB 共用 IRQ 1, 所以一个函数管两路。
   ★ 名字 GROUP1_IRQHandler 由 SysConfig 的启动文件引用, 不要改名。
   ★★ 两个端口【都要读】: E1A=PB20 在 GPIOB, E2A=PA25 在 GPIOA, 不是同一个端口。
      只读一个端口有两个后果:
      (1) 另一路的脉冲数永远不涨 -> 那一路的速度环拿不到反馈;
      (2) 那一路的中断标志没人清 -> 电平还在, 中断反复重进, 程序直接卡死。 */
void GROUP1_IRQHandler(void)
{
    switch (DL_GPIO_getPendingInterrupt(GPIOA))
    {
        case encoder_E2A_IIDX:
            encoder_2_A++;
            break;

        default:
            break;
    }

    switch (DL_GPIO_getPendingInterrupt(GPIOB))
    {
        case encoder_E1A_IIDX:
            encoder_1_A++;
            break;

        default:
            break;
    }
}
