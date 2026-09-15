/* ============================================================================
 *  empty.c —— 【只测轮速闭环】最小主程序
 * ----------------------------------------------------------------------------
 *  这一版【故意只有一件事】: 编码器测速 + 每轮速度 PI。
 *  循迹 / 陀螺仪 / 灰度 / 云台 全都不接 —— 先把"速度测得准、控得住"这一件
 *  事确认掉, 再往上加东西。
 *
 *  【接线】E1A = PB20, E2A = PA25 (只数 A 相双沿)
 *
 *  【按键】
 *      KEY2 = 速度档位 +1      (0,100,200,300,400,500,600 mm/s)
 *      KEY3 = 速度档位 -1
 *      KEY1 = 立刻停(目标归 0)
 *
 *  【屏幕】128x64
 *      y=0   档位号
 *      y=16  L  目标 -> 实测      (mm/s)
 *      y=28  R  目标 -> 实测
 *      y=40  d1 d2                (PI 给出的占空比 0~20)
 *      y=52  c1 c2                (★ 每 20ms 数到的原始脉冲数, 诊断用)
 *
 *  ★★ 屏幕第 52 行那两个 c 是这次的重点, 它直接决定问题出在哪 ★★
 *      轮子停着时:
 *          c = 0          -> 正常(没有假信号)
 *          c 一直非 0     -> 输入脚在飘: 查 A 相接线 / 编码器供电
 *      用手慢慢转一个轮子:
 *          对应的 c 跟着变 -> 编码器是好的
 *          只有另一个 c 变 -> 两个轮的编码器接反了
 * ==========================================================================*/
#include "ti_msp_dl_config.h"
#include <stdint.h>

#include "tick.h"               /* tick_init / tick_get_ms */
#include "delay.h"              /* delay_ms */
#include "key.h"                /* key_init / key_getnum */
#include "oled.h"               /* OLED_Init / ShowString / ShowNum / Refresh */
#include "encoder.h"            /* ★ 速度环: motor_speed_* */

/* ---------------- 速度档位 ---------------- */
static const uint16_t k_levels[] = { 0U, 100U, 200U, 300U, 400U, 500U, 600U };
#define LEVEL_N     (sizeof(k_levels) / sizeof(k_levels[0]))
static uint8_t s_lvl = 0U;

/* ---------------- 异常兜底: 只在屏幕上显示 ★ 不打串口 ★ ---------------- */
static void fault_report(const char *name)
{
    OLED_Clear();
    OLED_ShowString(0,  0, (u8 *)"!! FAULT !!", 16);
    OLED_ShowString(0, 24, (u8 *)name,          16);
    OLED_Refresh();
    while (1) { }               /* 停住, 让人能看清 */
}
void NMI_Handler(void)       { fault_report("NMI"); }
void HardFault_Handler(void) { fault_report("HARDFAULT"); }

/* ---------------- 屏幕: 带符号 3 位 ---------------- */
static void show_s3(u8 x, u8 y, int16_t v, u8 size)
{
    uint16_t a;
    OLED_ShowChar(x, y, (u8)((v < 0) ? (u8)'-' : (u8)'+'), size);
    a = (uint16_t)((v < 0) ? -v : v);
    if (a > 999U) { a = 999U; }
    OLED_ShowNum((u8)(x + 6U), y, a, 3U, size);
}

/* ---------------- 测试页 ---------------- */
static void show_page(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"SPEED", 16);
    OLED_ShowNum(104, 0, s_lvl, 1, 16);

    OLED_ShowString(0, 16, (u8 *)"L", 12);
    show_s3(12, 16, (int16_t)motor_speed_get_target(1U), 12);
    OLED_ShowString(48, 16, (u8 *)">", 12);
    show_s3(60, 16, (int16_t)motor_speed_get(1U), 12);

    OLED_ShowString(0, 28, (u8 *)"R", 12);
    show_s3(12, 28, (int16_t)motor_speed_get_target(2U), 12);
    OLED_ShowString(48, 28, (u8 *)">", 12);
    show_s3(60, 28, (int16_t)motor_speed_get(2U), 12);

    OLED_ShowString(0,  40, (u8 *)"d", 12);
    OLED_ShowNum(12, 40, motor_speed_get_duty(1U), 2, 12);
    OLED_ShowString(48, 40, (u8 *)"d", 12);
    OLED_ShowNum(60, 40, motor_speed_get_duty(2U), 2, 12);

    OLED_ShowString(0,  52, (u8 *)"c", 12);
    OLED_ShowNum(12, 52, (u32)motor_speed_get_raw(1U), 3, 12);
    OLED_ShowString(48, 52, (u8 *)"c", 12);
    OLED_ShowNum(60, 52, (u32)motor_speed_get_raw(2U), 3, 12);

    if ((motor_speed_get_fault(1U) != 0U) || (motor_speed_get_fault(2U) != 0U)) {
        OLED_ShowString(96, 52, (u8 *)"ERR", 12);
    }
    OLED_Refresh();
}

/* ---------------- 设定档位 ---------------- */
static void set_level(uint8_t lv)
{
    int32_t v;
    if (lv >= (uint8_t)LEVEL_N) { lv = (uint8_t)(LEVEL_N - 1U); }
    s_lvl = lv;
    v = (int32_t)k_levels[lv];
    motor_speed_set(1U, v);
    motor_speed_set(2U, v);
    show_page();
}

int main(void)
{
    uint32_t last_disp;

    SYSCFG_DL_init();

    tick_init();
    key_init();
    OLED_Init();

    motor_speed_init();                 /* 速度环: 清计数 + 开 GROUP1 中断 */

    set_level(0U);                      /* 上电先停着 */
    delay_ms(300);

    last_disp = tick_get_ms();

    while (1)
    {
        uint8_t code = key_getnum();

        motor_speed_update();           /* 测速 + PI, 内部按 20ms 分频 */

        if (code == 2U)      { set_level((uint8_t)(s_lvl + 1U)); }
        else if (code == 3U) { set_level((uint8_t)(s_lvl - 1U)); }
        else if (code == 1U) { set_level(0U); }

        /* 每 100ms 刷一次屏 */
        if ((tick_get_ms() - last_disp) >= 100U) {
            last_disp = tick_get_ms();
            show_page();
        }
    }
}
