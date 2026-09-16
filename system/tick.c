/* ============================================================================
 *  tick.c —— SysTick 1ms 时基
 * ----------------------------------------------------------------------------
 *  SysTick_Config(CPUCLK_FREQ / 1000) -> 1ms 一次中断, HANDLER 里累加毫秒。
 *  覆盖启动文件的弱默认 SysTick_Handler(标准 CMSIS 弱定义, 安全)。
 *  用 CPUCLK_FREQ 宏, 改主频自动跟着变。
 * ==========================================================================*/
#include "tick.h"
#include "ti_msp_dl_config.h"

static volatile uint32_t s_ms;

void tick_init(void)
{
    SysTick_Config(CPUCLK_FREQ / 1000U);
}

uint32_t tick_get_ms(void)
{
    return s_ms;
}

void SysTick_Handler(void)
{
    s_ms++;
}
