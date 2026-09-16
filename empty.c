#include "ti_msp_dl_config.h"
#include <stdint.h>
#include "key.h"            /* key_init / key_tick / key_getnum */
#include "motor.h"          /* motor_init / motor_set_direction / motor_set_duty */
#include "encoder.h"          /* encoder_get_speed */

uint8_t keynum;
/* encoder_1_A / encoder_2_A / speed_1 / speed_2 的声明都在 encoder.h 里,
 * 定义在 encoder.c 里。★ 这里不要再写一遍(带 = 0 的 extern 是错的, 也会重复定义) */

int main(void)
{
    SYSCFG_DL_init();
    motor_init();
    encoder_init();

    while (1)
    {
        motor_set_direction(1, 1);
        motor_set_duty(1, 100);
        motor_set_direction(2, 1);
        motor_set_duty(2, 100);
    }
}

/* ============================================================================
 *  GPIO 中断(GROUP1) —— GPIOA 和 GPIOB 共用这一个向量
 * ----------------------------------------------------------------------------
 *  ★★ 三个必须注意的点(原版都踩了) ★★
 *   1) DL_GPIO_getPendingInterrupt() 【只读 IIDX, 不会清标志】——
 *      不清的话退出中断后标志还在, 立刻再次触发 -> 中断风暴 -> CPU 卡死。
 *      症状就是【烧录不上】。所以这里用 getEnabledInterruptStatus + 显式清。
 *   2) E1A = PB20 在 GPIOB, 但 E2A = PA25 在 GPIOA —— 两个【不在同一个端口】。
 *      原版只读 GPIOB, 所以 encoder_2_A 永远数不到。
 *   3) 一次把两个端口所有已使能标志都读出来, 处理完【全部清掉】——
 *      漏掉任何一个都会风暴。
 * ==========================================================================*/
void GROUP1_IRQHandler(void)
{
    uint32_t sta_a, sta_b;

    sta_a = DL_GPIO_getEnabledInterruptStatus(GPIOA, 0xFFFFFFFFU);
    sta_b = DL_GPIO_getEnabledInterruptStatus(GPIOB, 0xFFFFFFFFU);

    /* 数脉冲: A 相双沿 */
    if ((sta_b & encoder_E1A_PIN) != 0U) { encoder_1_A++; }
    if ((sta_a & encoder_E2A_PIN) != 0U) { encoder_2_A++; }

    /* ★ 清标志, 一个都不留 */
    if (sta_a != 0U) { DL_GPIO_clearInterruptStatus(GPIOA, sta_a); }
    if (sta_b != 0U) { DL_GPIO_clearInterruptStatus(GPIOB, sta_b); }
}