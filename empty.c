/* ============================================================================
 *  empty.c —— 循迹小车(最基础版) + 串口调试
 * ----------------------------------------------------------------------------
 *  【当前阶段: 先把 8 路灰度调通】
 *      串口(UART2, TX=PB15, 115200, 8N1)每 100ms 打印一行:
 *          S=00001000 E=+014 AD=111
 *          S  : 8 路灰度的原始值, 从左到右。有黑线 = 1, 没有 = 0
 *          E  : 由 S 算出来的线偏差(-100~+100)
 *          AD : 通道选择脚 AD2/AD1/AD0 的电平(选完最后一路应该是 111)
 *
 *      怎么用:
 *          不用按任何键, 把车拿在手上, 让传感器在"黑线 / 白底"之间来回移动,
 *          看串口 S= 后面那 8 位有没有跟着变。
 *
 *  【如果 S 一直不动, 按这个顺序隔离】
 *      1) 看 AD= 是不是 111
 *           - 不是 111(比如一直 000) -> 通道选择脚没接好/没驱动, 查 PB24/PA24/PA26
 *      2) 把传感器的 OUT 线从 PA22 上拔下来, 然后手动把 PA22 短接到 3.3V 和 GND
 *           - 短到 3.3V 时 S 变成 11111111, 短到 GND 变成 00000000
 *             -> 说明单片机这侧(PA22 输入)是好的, 问题在传感器模块/供电/接线
 *           - 短接也没反应 -> 问题在单片机这边的引脚配置
 *      3) 传感器模块单独查: 供电(VCC/GND)、OUT 是否接到了 PA22、
 *         模块上的指示灯会不会随黑白变化
 *
 *  【按键】
 *      KEY1 = 循迹 开/关        KEY2 = 电机自检(两轮 50%) 开/关
 *      建议: 传感器调通之前先别按 KEY1, 免得车乱跑
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
#include "grayscale_sensor.h"   /* Grayscale_Sensor_Init / GRAYSCALE_SENSOR_CHANNELS */
#include "oled.h"               /* OLED_Init / OLED_ShowString / OLED_Refresh */

/* ---- 循迹控制 ---- */
#include "line_follow.h"

/* ---------- 时间节拍 ---------- */
#define STEP_MS     10U         /* 循迹控制周期 */
#define DISP_MS     100U        /* 屏幕刷新周期 */
#define UART_MS     100U        /* 串口打印周期 */

static uint8_t s_test_mode = 0U;    /* KEY2 的电机自检开关 */

/* ---------------------------------------------------------------------------
 *  中断服务: main_timer(TIMA0, 50ms) 里扫键
 * -------------------------------------------------------------------------*/
void main_timer_INST_IRQHandler(void)
{
    if (DL_Timer_getPendingInterrupt(main_timer_INST) == DL_TIMER_IIDX_ZERO)
    {
        key_tick();
    }
}

/* ============================================================================
 *  串口调试打印 (PC_uart = UART2, TX = PB15, 115200)
 * ==========================================================================*/

static void pc_putc(char c)
{
    while (DL_UART_isBusy(PC_uart_INST)) { }    /* 等上一个字节发完 */
    DL_UART_transmitData(PC_uart_INST, (uint8_t)c);
}

static void pc_puts(const char *s)
{
    while (*s != 0) { pc_putc(*s++); }
}

/* 打印一个带符号的数, 固定 4 个字符: +014 / -100 / +000 */
static void pc_put_signed4(int16_t v)
{
    uint16_t a;
    pc_putc((v < 0) ? '-' : '+');
    a = (uint16_t)((v < 0) ? -v : v);
    pc_putc((char)('0' + (a / 100) % 10));
    pc_putc((char)('0' + (a / 10) % 10));
    pc_putc((char)('0' + (a % 10)));
}

/* 打印一行传感器状态:  S=00001000 E=+014 AD=111 */
static void pc_print_sensor(void)
{
    uint16_t raw[GRAYSCALE_SENSOR_CHANNELS];
    uint8_t  i;

    line_follow_get_raw(raw);

    pc_puts("S=");
    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        pc_putc((raw[i] != 0U) ? '1' : '0');    /* 1 = 读到(黑线), 0 = 没读到 */
    }

    pc_puts(" E=");
    pc_put_signed4(line_follow_get_error());

    /* 通道选择脚 AD2/AD1/AD0 的实际电平:
     * 读完 8 路后最后一个选的是通道 7, 所以正常应该是 111。
     * 如果这里一直不是 111, 说明"选通道"这一步没生效。 */
    pc_puts(" AD=");
    pc_putc(DL_GPIO_readPins(GrayS_AD2_PORT, GrayS_AD2_PIN) ? '1' : '0');
    pc_putc(DL_GPIO_readPins(GrayS_AD1_PORT, GrayS_AD1_PIN) ? '1' : '0');
    pc_putc(DL_GPIO_readPins(GrayS_AD0_PORT, GrayS_AD0_PIN) ? '1' : '0');

    pc_puts("\r\n");
}

/* ============================================================================
 *  屏幕显示
 * ==========================================================================*/

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

static void show_raw(u8 x, u8 y, const uint16_t *raw, u8 size)
{
    u8 i;
    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        u8 ch = (raw[i] != 0U) ? (u8)'1' : (u8)'0';
        OLED_ShowChar((u8)(x + (u8)(i * (size / 2))), y, ch, size);
    }
}

static void show_status(void)
{
    uint16_t raw[GRAYSCALE_SENSOR_CHANNELS];

    if (s_test_mode != 0U) {
        OLED_ShowString(0, 0, (u8 *)"LF:TEST", 16);
    } else if (line_follow_is_running()) {
        OLED_ShowString(0, 0, (u8 *)"LF:RUN ", 16);
    } else {
        OLED_ShowString(0, 0, (u8 *)"LF:STOP", 16);
    }

    line_follow_get_raw(raw);
    OLED_ShowString(0, 16, (u8 *)"S:", 12);
    show_raw(12, 16, raw, 12);

    OLED_ShowString(0, 28, (u8 *)"E:", 12);
    show_signed3(12, 28, line_follow_get_error(), 12);

    OLED_ShowString(0, 40, (u8 *)"L:", 12);
    OLED_ShowNum(12, 40, line_follow_get_left_duty(), 3, 12);
    OLED_ShowString(36, 40, (u8 *)"R:", 12);
    OLED_ShowNum(48, 40, line_follow_get_right_duty(), 3, 12);

    OLED_ShowString(0, 52, (u8 *)"K1:Run K2:Test", 12);

    OLED_Refresh();
}

/* ============================================================================
 *  主程序
 * ==========================================================================*/
int main(void)
{
    uint32_t last_step_ms;
    uint32_t last_disp_ms;
    uint32_t last_uart_ms;

    /* ======================= 1. 外设初始化 ======================= */
    SYSCFG_DL_init();
    tick_init();
    key_init();
    motor_init();
    Grayscale_Sensor_Init();
    OLED_Init();

    line_follow_init();

    /* 故意不初始化 MPU6050: 本阶段不用, 而且它的 I2C 读没有超时保护 */

    /* ======================= 2. 开机画面 ======================= */
    OLED_Clear();
    OLED_ShowString(0,  0, (u8 *)"SENSOR DEBUG", 16);
    OLED_ShowString(0, 20, (u8 *)"UART2 PB15",   12);
    OLED_ShowString(0, 36, (u8 *)"115200 8N1",   12);
    OLED_Refresh();
    delay_ms(800);

    OLED_Clear();               /* 擦掉开机画面, 免得和状态行错位留残余 */

    pc_puts("\r\n=== LINE SENSOR DEBUG ===\r\n");
    pc_puts("S=8 channel raw (1=line), E=error, AD=channel pins\r\n");

    led_off(1);
    led_off(2);

    show_status();

    last_step_ms = tick_get_ms();
    last_disp_ms = last_step_ms;
    last_uart_ms = last_step_ms;

    /* ======================= 3. 主循环 ======================= */
    while (1)
    {
        uint32_t now = tick_get_ms();
        uint8_t  code = key_getnum();

        /* ---------------- KEY1: 循迹 开/关 ---------------- */
        if (code == 1U)
        {
            s_test_mode = 0U;
            if (line_follow_is_running()) {
                line_follow_stop();
            } else {
                line_follow_start();
            }
            show_status();
            pc_puts("[KEY1] ");
            pc_puts(line_follow_is_running() ? "follow ON\r\n" : "follow OFF\r\n");
        }

        /* ---------------- KEY2: 电机自检 ---------------- */
        else if (code == 2U)
        {
            line_follow_stop();
            s_test_mode = (uint8_t)((s_test_mode == 0U) ? 1U : 0U);
            line_follow_test_wheels(s_test_mode);
            show_status();
            pc_puts("[KEY2] ");
            pc_puts(s_test_mode ? "motor TEST on\r\n" : "motor TEST off\r\n");
        }

        /* ---------------- 每 10ms: 循迹控制 ---------------- */
        if ((now - last_step_ms) >= STEP_MS)
        {
            last_step_ms = now;
            line_follow_step();
        }

        /* ---------------- 每 100ms: 串口打印传感器 ---------------- */
        if ((now - last_uart_ms) >= UART_MS)
        {
            last_uart_ms = now;
            pc_print_sensor();
        }

        /* ---------------- 每 100ms: 刷屏 ---------------- */
        if ((now - last_disp_ms) >= DISP_MS)
        {
            last_disp_ms = now;
            show_status();
        }

        if (line_follow_is_running()) {
            led_on(1);
        } else {
            led_off(1);
        }
    }
}
