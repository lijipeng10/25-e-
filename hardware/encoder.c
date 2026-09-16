#include "encoder.h"
#include "ti_msp_dl_config.h"
#include "motor.h"
#include "pid.h"
#include "tick.h"

/* ★ 带符号: 正 = 前进, 负 = 后退。方向由 GROUP1 中断里读 B 相电平决定 */
int32_t encoder_1_A;
int32_t encoder_2_A;

/* 实测速度 mm/s【带符号】: 正 = 前进, 负 = 后退。
   定义放在本模块里(原来定义在 empty.c, 声明却在这里, 主程序不该持有电机测速的状态)。 */
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

