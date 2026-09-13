/* ============================================================================
 *  empty.c —— 循迹小车(最基础版)
 * ----------------------------------------------------------------------------
 *  【怎么用】
 *      上电后车不动, 屏幕显示状态。
 *      KEY1 按一下 -> 开始循迹;  再按一下 -> 停止
 *      KEY2 按一下 -> 电机自检(两轮都按前进方向转 50%);  再按一下 -> 停
 *                    这个模式不经过循迹逻辑, 专门用来验证"电机和接线好不好"
 *
 *  【屏幕显示(调参全靠它)】
 *      LF:STOP       状态: STOP / RUN / TEST(电机自检)
 *      S:00000000    8 路灰度的【原始值】: 从左到右, 车压到黑线时对应位应该变 1
 *      E:  -12       线偏差: -100(线在最左) ~ 0(正中) ~ +100(线在最右)
 *      L:040 R:040   左右轮实际占空比(%)
 *      K1:Run K2:Test
 *
 *  【怎么标定 —— 按顺序做】
 *      第1步 看 S: 那一行(不用按键, 上电就在刷)
 *              把车放到黑线上 / 拿开, 看 1 的位置有没有跟着变。
 *              - 一直全是 0, 变都不变  -> 传感器没工作/没接好/没供电, 查硬件
 *              - 压线时反而是 0        -> 极性反了, 把 line_follow.c 的
 *                                          LF_LINE_LEVEL 从 1 改成 0
 *              - 压线时对应的位变成 1  -> 正常, 进入第2步
 *      第2步 按 KEY1 开始循迹, 看车是"往线上掰"还是"越走越远"
 *              - 越偏越远 -> 把 line_follow.c 的 LF_STEER_SIGN 改成 -1
 *      第3步 速度/灵敏度: LF_BASE_DUTY(速度)、LF_KP(画龙就调小, 拐不过来就调大)
 *              参数都在 line_follow.c 顶部的"可调参数"区
 *
 *  【中断分配】
 *      main_timer(TIMA0, 50ms) -> key_tick() 扫键
 *      SysTick(1ms)            -> tick.c 累加毫秒时基
 * ==========================================================================*/
#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ---- 基础库 ---- */
#include "tick.h"               /* tick_init / tick_get_ms */
#include "delay.h"              /* delay_ms */

/* ---- 外设模块 ---- */
#include "key.h"                /* key_init / key_getnum */
#include "motor.h"              /* motor_init */
#include "led.h"                /* led_on / led_off */
#include "grayscale_sensor.h"   /* Grayscale_Sensor_Init */
#include "oled.h"               /* OLED_Init / OLED_ShowString / OLED_Refresh */

/* ---- 循迹控制 ---- */
#include "line_follow.h"

/* ---------- 时间节拍 ---------- */
#define STEP_MS     10U         /* 循迹控制周期: 每 10ms 跑一次 */
#define DISP_MS     100U        /* 屏幕刷新周期: 每 100ms 一次 */

static uint8_t s_test_mode = 0U;    /* KEY2 的电机自检开关 */

/* ---------------------------------------------------------------------------
 *  中断服务: main_timer(TIMA0, 50ms) 里扫键
 *  (按键的定时器配置/开中断/NVIC 都在 key_init() 里做完了)
 * -------------------------------------------------------------------------*/
void main_timer_INST_IRQHandler(void)
{
    if (DL_Timer_getPendingInterrupt(main_timer_INST) == DL_TIMER_IIDX_ZERO)
    {
        key_tick();
    }
}

/* ============================================================================
 *  屏幕显示相关的小工具
 * ==========================================================================*/

/* 显示带符号的三位数(OLED_ShowNum 只能显示无符号数, 所以自己写)
 * 例如 -12 -> "- 12",  12 -> "+ 12" */
static void show_signed3(u8 x, u8 y, int16_t v, u8 size)
{
    if (v < 0) {
        OLED_ShowChar(x, y, (u8)'-', size);
        OLED_ShowNum((u8)(x + size / 2), y, (u32)(-v), 3, size);
    } else {
        OLED_ShowChar(x, y, (u8)'+', size);
        OLED_ShowNum((u8)(x + size / 2), y, (u32)v, 3, size);
    }
}

/* 显示 8 路灰度的原始值, 最左边那路显示在屏幕左边
 * 例如 8 路都读到 1 -> 屏幕显示 "11111111" */
static void show_raw(u8 x, u8 y, const uint16_t *raw, u8 size)
{
    u8 i;
    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        u8 ch = (raw[i] != 0U) ? (u8)'1' : (u8)'0';
        OLED_ShowChar((u8)(x + (u8)(i * (size / 2))), y, ch, size);
    }
}

/* 把整屏状态刷一遍
 * 注意: 每一行画的内容宽度都是固定的, 所以刷新时不会留下残余字符。 */
static void show_status(void)
{
    uint16_t raw[GRAYSCALE_SENSOR_CHANNELS];

    /* ---- 第 1 行: 状态 ---- */
    if (s_test_mode != 0U) {
        OLED_ShowString(0, 0, (u8 *)"LF:TEST", 16);
    } else if (line_follow_is_running()) {
        OLED_ShowString(0, 0, (u8 *)"LF:RUN ", 16);
    } else {
        OLED_ShowString(0, 0, (u8 *)"LF:STOP", 16);
    }

    /* ---- 第 2 行: 8 路灰度的原始值 ---- */
    line_follow_get_raw(raw);
    OLED_ShowString(0, 16, (u8 *)"S:", 12);
    show_raw(12, 16, raw, 12);

    /* ---- 第 3 行: 线偏差 ---- */
    OLED_ShowString(0, 28, (u8 *)"E:", 12);
    show_signed3(12, 28, line_follow_get_error(), 12);

    /* ---- 第 4 行: 左右轮占空比 ---- */
    OLED_ShowString(0, 40, (u8 *)"L:", 12);
    OLED_ShowNum(12, 40, line_follow_get_left_duty(), 3, 12);
    OLED_ShowString(36, 40, (u8 *)"R:", 12);
    OLED_ShowNum(48, 40, line_follow_get_right_duty(), 3, 12);

    /* ---- 第 5 行: 操作提示 ---- */
    OLED_ShowString(0, 52, (u8 *)"K1:Run K2:Test", 12);

    OLED_Refresh();     /* 改了显存必须刷新, 否则屏幕不会变 */
}

/* ============================================================================
 *  主程序
 * ==========================================================================*/
int main(void)
{
    uint32_t last_step_ms;
    uint32_t last_disp_ms;

    /* ======================= 1. 外设初始化 ======================= */
    SYSCFG_DL_init();
    tick_init();
    key_init();
    motor_init();
    Grayscale_Sensor_Init();
    OLED_Init();

    line_follow_init();

    /* 注意: 这里故意没有初始化 MPU6050:
     *   1) 最基础版循迹只用灰度, 不需要陀螺仪
     *   2) mpu6050.c 的 I2C 读没有超时保护, 传感器没接好会开机卡死
     * 以后要用陀螺仪时再加: mpu6050_init(); mpu6050_zero_yaw();  */

    /* ======================= 2. 开机画面 ======================= */
    OLED_Clear();
    OLED_ShowString(0,  0, (u8 *)"LINE FOLLOW",  16);
    OLED_ShowString(0, 20, (u8 *)"basic version", 12);
    OLED_ShowString(0, 36, (u8 *)"K1 start", 12);
    OLED_Refresh();
    delay_ms(800);

    /* ★ 关键: 进主循环前把屏幕擦干净!
     *   开机画面的行位置和下面的状态行(16/28/40/52)对不齐,
     *   不擦的话会留下一堆残余字符, 看起来就是"乱码"。 */
    OLED_Clear();

    led_off(1);
    led_off(2);

    show_status();

    last_step_ms = tick_get_ms();
    last_disp_ms = last_step_ms;

    /* ======================= 3. 主循环 ======================= */
    while (1)
    {
        uint32_t now = tick_get_ms();
        uint8_t  code = key_getnum();       /* 0=没按, 1=KEY1, 2=KEY2 */

        /* ---------------- KEY1: 开始 / 停止循迹 ---------------- */
        if (code == 1U)
        {
            s_test_mode = 0U;               /* 退出自检模式 */
            if (line_follow_is_running()) {
                line_follow_stop();
            } else {
                line_follow_start();
            }
            show_status();
        }

        /* ---------------- KEY2: 电机自检(两轮都转 50%) ---------------- */
        else if (code == 2U)
        {
            line_follow_stop();             /* 先停循迹, 免得和自检抢电机 */
            s_test_mode = (uint8_t)((s_test_mode == 0U) ? 1U : 0U);
            line_follow_test_wheels(s_test_mode);
            show_status();
        }

        /* ---------------- 每 10ms: 循迹控制 ---------------- */
        if ((now - last_step_ms) >= STEP_MS)
        {
            last_step_ms = now;
            line_follow_step();             /* 没在跑时内部直接 return, 可以无脑调 */
        }

        /* ---------------- 每 100ms: 刷屏 ---------------- */
        if ((now - last_disp_ms) >= DISP_MS)
        {
            last_disp_ms = now;
            show_status();
        }

        /* ---------------- LED1: 亮 = 正在循迹 ---------------- */
        if (line_follow_is_running()) {
            led_on(1);
        } else {
            led_off(1);
        }
    }
}
