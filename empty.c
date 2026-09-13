/* ============================================================================
 *  empty.c —— 循迹小车(最基础版)
 * ----------------------------------------------------------------------------
 *  【怎么用】
 *      上电后车不动, 屏幕显示状态。
 *      KEY1 按一下 -> 开始循迹
 *      KEY1 再按一下 -> 停止
 *
 *  【屏幕显示的是啥(调试用, 很重要)】
 *      LF:RUN        当前状态(在跑 / 停着)
 *      S:00110000    8 路灰度: 从左到右, 1 = 这一路压到黑线了
 *      E:  -12       线偏差: -100(线在最左) ~ 0(正中) ~ +100(线在最右)
 *      L:40 R:40     左右轮实际的占空比(%)
 *      KEY1:Run/Stop 操作提示
 *
 *      调参时重点看 S: 这一行 —— 把车放到线上/线外, 看 1 的位置有没有跟着变。
 *      如果压线时是 0、离开线才是 1, 说明传感器极性反了, 去改
 *      line_follow.c 里的 LF_LINE_LEVEL。
 *
 *  【中断分配】
 *      main_timer(TIMA0, 50ms) -> key_tick() 扫键
 *      SysTick(1ms)            -> tick.c 累加毫秒时基
 *
 *  接线: SCLK=PB9 MOSI=PB8 RES=PB10 DC=PB11 CS=PB14 BLK=PB26 (OLED)
 *        A路电机 PWM=PB6 方向=PB17/PB18   B路 PWM=PB7 方向=PB19/PB23
 *        灰度 OUT=PA22 AD0=PB24 AD1=PA24 AD2=PA26
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
#include "line_follow.h"        /* line_follow_init / start / stop / step */

/* ---------- 时间节拍 ---------- */
#define LF_STEP_MS      10U     /* 循迹控制周期: 每 10ms 跑一次 line_follow_step */
#define DISP_MS         100U    /* 屏幕刷新周期: 每 100ms 刷一次(太快看不清, 也浪费) */

/* ---------------------------------------------------------------------------
 *  中断服务: main_timer(TIMA0, 50ms) 里扫键
 *  注意: 按键的定时器配置、开中断、NVIC 都在 key_init() 里做完了
 * -------------------------------------------------------------------------*/
void main_timer_INST_IRQHandler(void)
{
    if (DL_Timer_getPendingInterrupt(main_timer_INST) == DL_TIMER_IIDX_ZERO)
    {
        key_tick();
    }
}

/* ============================================================================
 *  下面是屏幕显示用的小工具
 * ==========================================================================*/

/* 显示一个带符号的三位数(OLED_ShowNum 只能显示无符号数, 所以自己写一个)
 * 例如 -12 会显示成 "-012", 12 会显示成 "+012" */
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

/* 显示 8 路灰度的位图, 最左边那路显示在左边
 * 例如 bits = 0b00110000 -> 屏幕上显示 "00110000" */
static void show_bits(u8 x, u8 y, uint8_t bits, u8 size)
{
    u8 i;
    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        u8 ch = ((bits >> i) & 0x01U) ? (u8)'1' : (u8)'0';
        OLED_ShowChar((u8)(x + (u8)(i * (size / 2))), y, ch, size);
    }
}

/* 把整屏状态刷一遍 */
static void show_status(void)
{
    /* 第 1 行: 运行状态(16 号字, 大一点看得清) */
    if (line_follow_is_running()) {
        OLED_ShowString(0, 0, (u8 *)"LF:RUN ", 16);
    } else {
        OLED_ShowString(0, 0, (u8 *)"LF:STOP", 16);
    }

    /* 第 2 行: 8 路灰度(12 号字) */
    OLED_ShowString(0, 16, (u8 *)"S:", 12);
    show_bits(12, 16, line_follow_get_bits(), 12);

    /* 第 3 行: 线偏差 */
    OLED_ShowString(0, 28, (u8 *)"E:", 12);
    show_signed3(12, 28, line_follow_get_error(), 12);

    /* 第 4 行: 左右轮占空比 */
    OLED_ShowString(0, 40, (u8 *)"L:", 12);
    OLED_ShowNum(12, 40, line_follow_get_left_duty(), 3, 12);
    OLED_ShowString(36, 40, (u8 *)"R:", 12);
    OLED_ShowNum(48, 40, line_follow_get_right_duty(), 3, 12);

    /* 第 5 行: 操作提示 */
    OLED_ShowString(0, 52, (u8 *)"KEY1:Run/Stop", 12);

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
    SYSCFG_DL_init();               /* SysConfig 生成: 时钟/引脚/外设, 必须最先调用 */

    tick_init();                    /* SysTick 1ms 时基 */
    key_init();                     /* 按键: 配 TIMA0 50ms + 开中断 + NVIC */
    motor_init();                   /* TB6612: STBY 使能, 占空比清零 */
    Grayscale_Sensor_Init();        /* 灰度传感器 */
    OLED_Init();                    /* OLED 复位 + 初始化 + 清屏 */

    line_follow_init();             /* 循迹 PID 初始化(此时不会让车动) */

    /* 注意: 这里**故意没有**初始化 MPU6050, 原因有两个:
     *   1) 最基础版循迹只用灰度, 不需要陀螺仪
     *   2) mpu6050.c 里的 I2C 读没有超时保护, 如果传感器没接好,
     *      开机会卡在 while 等数据那里, 整个程序起不来
     * 以后要用陀螺仪做航向纠偏时, 再把这行加回来:
     *      mpu6050_init();
     *      mpu6050_zero_yaw();
     */

    /* ======================= 2. 开机画面 ======================= */
    OLED_Clear();
    OLED_ShowString(0,  0, (u8 *)"LINE FOLLOW",  16);
    OLED_ShowString(0, 20, (u8 *)"basic version", 12);
    OLED_ShowString(0, 36, (u8 *)"KEY1 to start", 12);
    OLED_Refresh();
    delay_ms(600);                  /* 停一下让人看清 */

    led_off(1);
    led_off(2);

    show_status();                  /* 先显示一次状态 */

    last_step_ms = tick_get_ms();
    last_disp_ms = last_step_ms;

    /* ======================= 3. 主循环 ======================= */
    while (1)
    {
        uint32_t now = tick_get_ms();
        uint8_t  code = key_getnum();       /* 取键码: 0=没按, 1=KEY1, 2=KEY2 */

        /* ---------------- KEY1: 开始 / 停止 ---------------- */
        if (code == 1U)
        {
            if (line_follow_is_running()) {
                line_follow_stop();         /* 里面会把两个轮子关掉 */
            } else {
                line_follow_start();
            }
            show_status();                  /* 立刻刷新, 让状态马上显示出来 */
        }

        /* ---------------- 每 10ms: 跑一次循迹控制 ---------------- */
        if ((now - last_step_ms) >= LF_STEP_MS)
        {
            last_step_ms = now;

            /* 注意: 这个函数内部会问"在跑吗?", 没在跑就直接返回。
             *       所以这里可以无脑调用, 不用自己判断。 */
            line_follow_step();
        }

        /* ---------------- 每 100ms: 刷新屏幕 ---------------- */
        if ((now - last_disp_ms) >= DISP_MS)
        {
            last_disp_ms = now;
            show_status();
        }

        /* ---------------- LED1 指示是否在循迹 ---------------- */
        if (line_follow_is_running()) {
            led_on(1);
        } else {
            led_off(1);
        }
    }
}
