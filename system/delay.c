/**
 * delay.c - 纯软件忙等延时。
 *
 * 说明：不依赖任何定时器外设（SYSTICK / TIMG 等），完全用 CPU 指令数
 *       估算时间，因此在 SysConfig 未开启 SYSTICK 时也能工作。
 *
 * 注意：由于是“忙等 + 按主频粗算”，实际延时会有一定偏差，且会占用 CPU。
 *       适合 OLED 初始化、上电建立时间、按键/短时序等对精度要求不高的场景；
 *       若需要精确延时（如电机 PWM、编码器握手），请改用定时器方案。
 */
#include "delay.h"

/* 经验系数：在 -O2 优化下，带 volatile 的递减空循环每次迭代约需
 * DELAY_CYCLES_PER_LOOP 个 CPU 时钟周期，据此把“目标周期数”折算成“循环数”。
 * 若觉得偏快/偏慢，可微调该值（越大越慢、越耗时）。 */
#define DELAY_CYCLES_PER_LOOP   5U

void delay_us(uint32_t us)
{
    volatile uint32_t loops = 0U;

    if (us == 0U) return;

    /* CPUCLK_FREQ 由 SysConfig 生成，见 ti_msp_dl_config.h（80MHz 时=80000000）。
     * 1us 主机周期 = CPUCLK_FREQ / 1000000，再除以每次循环所需周期数。 */
    loops = (us * (CPUCLK_FREQ / 1000000U)) / DELAY_CYCLES_PER_LOOP;
    while (loops-- != 0U) { /* 空转 */ }
}

void delay_ms(uint32_t ms)
{
    if (ms == 0U) return;
    while (ms-- != 0U) {
        delay_us(1000U); /* 1ms = 1000us */
    }
}
