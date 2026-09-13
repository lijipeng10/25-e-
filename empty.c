/* ============================================================================
 *  empty.c —— 循迹小车(最基础版) + OLED 调试
 * ----------------------------------------------------------------------------
 *  【当前阶段: 传感器已调通, 开始接电机跑闭环】
 *
 *  ─── 落地之前必须先做的两次"悬空验证"(车拿在手上, 轮子离地) ───
 *
 *   验证 1: 前进方向对不对
 *      按 KEY2 -> 两个轮子都往"前"转(推力应该推着车往前走), 再按一次停。
 *      如果某个轮子反转了, 改 system/line_follow.c 里的
 *      LF_LEFT_FWD_DIR / LF_RIGHT_FWD_DIR。
 *
 *   验证 2: 转向极性对不对(最关键的一步!)
 *      桌上贴一条黑胶带当线, 车拿在手上, 让传感器对着那条线。
 *      按 KEY1 开始循迹, 然后把传感器在线上慢慢左右平移, 盯屏幕最下面一行:
 *          线往左偏  ->  L 变小、R 变大   ✓  左轮慢/右轮快 = 车往左拐
 *          线往右偏  ->  R 变小、L 变大   ✓
 *      如果反了(线往左偏反而左轮变快), 把 line_follow.c 里的
 *      LF_STEER_SIGN 从 +1 改成 -1。
 *
 *   这两项都验证过了, 再把车放到线上按 KEY1, 它才会真的跟着线走。
 *   (不验证直接落地跑, 极性一反车就会全速冲出去)
 *
 *  ─── 调试数据(只看 OLED, 每 100ms 刷新) ───
 *          LF:STOP / LF:RUN / LF:TEST   循迹状态
 *          S: 00011000                  8 路灰度原始值, 1 = 压到黑线
 *          E: -014                      线偏差, 负 = 线在左, 正 = 线在右
 *          L:040 R:040                  左轮/右轮实际输出占空比(%)
 *          K1:Run K2:Test               按键提示
 *
 *      串口打印已由下面的 DBG_UART 关掉(= 0)。串口是"死等发完"的,
 *      一行 40 多字符 ≈ 3.8ms, 会占住主循环、把 10ms 控制周期拖出抖动。
 *      以后想用串口看数据, 把 DBG_UART 改回 1 就行。
 *
 *  【万一以后 S 又不动了, 按这个顺序隔离】
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
 *      屏幕右下角一直显示 "K1:Run K2:Test" 提醒。
 *
 *  【循迹怎么走】两个状态自动切换, 不用管:
 *      LF:RUN   正常循迹(跟着线走, 差速修正)
 *      LF:PIVOT ★ 到弯节点了: 线丢了 -> 先停住 -> 原地把车头拧正 -> 继续循迹
 *      屏幕第一行会实时显示是哪个状态。
 *      为什么这么做: 车重又慢, 靠差速硬拐急弯本来就吃力, 容易冲出去。
 *      停车原地转向反而更可靠。判断"到弯了"靠丢线, 判断"转正了"靠传感器
 *      重新看到线并且落在中间 —— 不需要陀螺仪或编码器。
 *
 *  【速度/参数在哪改】
 *      全部在 system/line_follow.c 最上面那一块 "可调参数":
 *          LF_MAX_DUTY      ★ 最高速度硬顶(任何一轮都不许超过)  20
 *          LF_BASE_DUTY     直行基础速度                      18
 *                           ★ 必须明显高于电机启动死区, 否则左右摆!
 *                             速度调低反而摆得更厉害 = 这个原因
 *          LF_LOST_DUTY     丢线找线速度                      16
 *          LF_TEST_DUTY     KEY2 自检速度                     20
 *          LF_MAX_STEER     ★ 转向量上限 = 转弯力度               18
 *                           ★★ 必须 >= LF_BASE_DUTY, 理由见文件里的推导:
 *                             它决定"慢的一侧能降到多低", 降不到 0 就转不过弯
 *          LF_KP / LF_KD    转向 PID 的 P / D                 20 / 0
 *                           ★ D 必须是 0 或很小, 理由见文件里的推导
 *                           ★ KP 必须和 LF_MAX_STEER 配套改, 见文件里说明
 *          LF_DEADBAND      ★ 误差死区, |误差| 小于它就不修正      15
 *                           ★ 专治直线上的左右摆: 误差最小跳变是 14,
 *                             车根本停不住停在 14 上, 于是 steer 在 ±2 之间
 *                             来回跳 -> 摆。死区把这档压掉就不摆了
 *
 *          --- 弯道: 停车原地转向再前进 ---
 *          LF_PIVOT_TRIGGER_MS  连续丢线多久判定"到弯节点"     80
 *          LF_PIVOT_DUTY        原地转向的占空比(一正一反)      16
 *          LF_PIVOT_OK          |误差| 小于它就算"对准了"        20
 *          LF_PIVOT_TRY_MS      一个方向找多久没找到就掉头找   900
 *                               (两个方向合计 1.8 秒封顶)
 *
 *      ★ 约束(违反了直接编译报错, 不会等跑车才发现):
 *          LF_BASE_DUTY <= LF_MAX_DUTY
 *          LF_LOST_DUTY <= LF_MAX_DUTY
 *          LF_MAX_STEER >= LF_BASE_DUTY      <- 转弯力度靠这条
 *          LF_STEER_SIGN    转向极性 +1 / -1            默认 +1
 *          LF_LINE_LEVEL    灰度"压线"判定电平           默认 1
 *          LF_LEFT/RIGHT_FWD_DIR  两轮"前进"方向值       1 / 2
 *          LF_TRIM          左右电机补偿, 现在关着(0)
 *                           车轻微往一边偏时再启用, 按 KEY2 在长直道上调
 *
 *  ★ 本车较重(带云台 + 相机), 所有速度一律取小值, 见 line_follow.c 顶部说明。
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

/* ---------- 串口调试开关 ----------
 * 1 = 打开串口打印, 0 = 关掉。
 * 为什么要能关: 串口是"一个字符一个字符死等发完"的, 一行 40 多个字符
 * 大概要 3.8ms, 这段时间主循环被占住, 循迹的 10ms 控制周期会被拖出抖动。
 * 对着电脑调参时开着(1)方便看数据; 正式放地上跑车建议关掉(0)。 */
#define DBG_UART    1

/* ---------- 关于板上的 LED ----------
 * 现在用的是【最小系统板】, PB2 / PB3 / PB21 都没有接 LED, 所以一切诊断
 * 都走 OLED + 串口, 代码里不再用 led_on() / led_off()。
 * (hardware/led.c 那个驱动保留着, 等正式板子到了直接就能用。)
 *
 * 由此可见: 之前"三个 LED 都不亮"不是电路坏了, 是引脚上根本没器件。 */

/* ---------- 时间节拍 ---------- */
#define STEP_MS     10U         /* 循迹控制周期 */
#define DISP_MS     100U        /* 屏幕刷新周期 */
#define UART_MS     100U        /* 串口打印周期 */

static uint8_t s_test_mode = 0U;    /* KEY2 的电机自检开关 */

/* ---------- 心跳 ----------
 * 主循环每转一圈 s_hb 加 1, 加够了就翻转 LED2。
 * 关键: 它【不依赖 SysTick, 也不依赖任何定时器和中断】, 纯靠 CPU 转圈。
 * 所以它是判断"CPU 到底还活着没有"的唯一可靠手段 ——
 * 屏幕冻住的时候看 LED2: 还在闪 = CPU 在跑; 不闪 = CPU 真停了。
 * 数值随便, 只影响闪的快慢, 越大越慢。 */
#define HB_LOOPS    300000U
static uint32_t s_hb = 0U;          /* 心跳计数 */

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
 * ----------------------------------------------------------------------------
 *  整段被 DBG_UART 包住: 关掉时这些函数根本不参与编译,
 *  否则"定义了却没人调用"会报 unused function 警告。
 * ==========================================================================*/
#if DBG_UART

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

/* 打印 3 位无符号数, 固定 3 个字符: 040 / 100 / 000 */
static void pc_put_u3(uint8_t v)
{
    if (v > 100U) { v = 100U; }
    pc_putc((char)('0' + (v / 100U) % 10U));
    pc_putc((char)('0' + (v / 10U) % 10U));
    pc_putc((char)('0' + (v % 10U)));
}

/* 打印一行状态:  S=00011000 AD=111 E=-014 L=040 R=040
 *
 *   S   : 8 路灰度原始值, 从左到右, 1 = 黑线
 *   AD  : 通道选择脚 AD2/AD1/AD0 的实际电平
 *   E   : 线偏差, 负 = 线在左边, 正 = 线在右边
 *   L/R : 左轮/右轮"实际输出"的占空比(%)
 *
 * 关键用法(悬空验证转向极性, 不用把车放地上冒险):
 *   用手拿住车让轮子悬空, 按 KEY1 开始循迹,
 *   然后把传感器在黑线上慢慢左右平移, 观察 L 和 R:
 *       线往左偏  ->  左轮变慢(L 变小)、右轮变快(R 变大)   => 极性正确
 *       线往右偏  ->  右轮变慢(R 变小)、左轮变快(L 变大)   => 极性正确
 *   如果反了(线往左偏反而左轮变快), 就把 system/line_follow.c 里的
 *   LF_STEER_SIGN 从 +1 改成 -1。 */
static void pc_print_sensor(void)
{
    uint16_t raw[GRAYSCALE_SENSOR_CHANNELS];
    uint8_t  i;

    line_follow_get_raw(raw);

    pc_puts("S=");
    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        pc_putc((raw[i] != 0U) ? '1' : '0');    /* 1 = 读到(黑线), 0 = 没读到 */
    }

    /* 通道选择脚 AD2/AD1/AD0 的实际电平:
     * 读完 8 路后最后一个选的是通道 7, 所以正常应该是 111。
     * 如果这里一直不是 111, 说明"选通道"这一步没生效。 */
    pc_puts(" AD=");
    pc_putc(DL_GPIO_readPins(GrayS_AD2_PORT, GrayS_AD2_PIN) ? '1' : '0');
    pc_putc(DL_GPIO_readPins(GrayS_AD1_PORT, GrayS_AD1_PIN) ? '1' : '0');
    pc_putc(DL_GPIO_readPins(GrayS_AD0_PORT, GrayS_AD0_PIN) ? '1' : '0');

    pc_puts(" E=");
    pc_put_signed4(line_follow_get_error());

    /* 左右轮实际占空比 —— 悬空验证转向极性就看这两个数 */
    pc_puts(" L=");
    pc_put_u3(line_follow_get_left_duty());
    pc_puts(" R=");
    pc_put_u3(line_follow_get_right_duty());

    pc_puts("\r\n");
}

#else
/* DBG_UART = 0: 打印用的接口全部变成空操作, 主循环里照旧调用即可 */
#define pc_puts(s)          ((void)0)
#define pc_print_sensor()   ((void)0)
#endif

/* 主循环里统一用这两个宏调, 这样开关一改, 调用处不用动 */
#if DBG_UART
#define DBG_SENSOR()        pc_print_sensor()
#define DBG_MSG(s)          pc_puts(s)
#else
#define DBG_SENSOR()        ((void)0)
#define DBG_MSG(s)          ((void)0)
#endif

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

    /* 状态行: 一眼就能看出车现在在干什么 */
    if (s_test_mode != 0U) {
        OLED_ShowString(0, 0, (u8 *)"LF:TEST ", 16);
    } else if (line_follow_is_pivoting()) {
        OLED_ShowString(0, 0, (u8 *)"LF:PIVOT", 16);   /* 弯道: 原地转向中 */
    } else if (line_follow_is_running()) {
        OLED_ShowString(0, 0, (u8 *)"LF:RUN  ", 16);
    } else {
        OLED_ShowString(0, 0, (u8 *)"LF:STOP ", 16);
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
 *  故障兜底 —— 让"死机"看得见(走串口 + OLED, 因为最小系统板上没有 LED)
 * ----------------------------------------------------------------------------
 *  CMSIS 启动文件里, NMI / HardFault 这些异常默认都是弱定义的 while(1) 空转,
 *  也就是说一旦 CPU 跑飞, 现场和"正常运行"长得一模一样, 根本分不出来。
 *  这里改成: 串口打一行 "!!! HARDFAULT !!!" / "!!! NMI !!!", OLED 上也写出来,
 *  然后【停在那里不动】。所以症状特别好认 —— 屏幕定格在 !! FAULT !!。
 *
 *  ★ 为什么串口发送要自己写一份带超时的: 平时的 pc_putc() 是
 *        while (DL_UART_isBusy(...)) { }     死等
 *    异常发生时外设可能已经不正常了, 用死等版会让异常处理本身也卡住,
 *    反而一个字都看不到。所以下面这份加了循环次数上限, 发不出去就算了。
 * ==========================================================================*/
static void fault_putc(char c)
{
    uint32_t guard = 200000U;       /* 最多等这么多圈, 超了就硬发 */

    while (DL_UART_isBusy(PC_uart_INST) && (guard != 0U)) { guard--; }
    DL_UART_transmitData(PC_uart_INST, (uint8_t)c);
}

static void fault_puts(const char *s)
{
    while (*s != 0) { fault_putc(*s++); }
}

static void fault_report(const char *name)
{
    fault_puts("\r\n!!! ");
    fault_puts(name);
    fault_puts(" !!!\r\n");

    /* 屏幕上再写一遍, 这样不接串口也能看到 */
    OLED_Clear();
    OLED_ShowString(0,  0, (u8 *)"!! FAULT !!", 16);
    OLED_ShowString(0, 24, (u8 *)name,          16);
    OLED_Refresh();

    while (1) { }                   /* 停住, 让人能看清 */
}

void NMI_Handler(void)       { fault_report("NMI"); }
void HardFault_Handler(void) { fault_report("HARDFAULT"); }

/* ============================================================================
 *  主程序
 * ==========================================================================*/
int main(void)
{
    uint32_t last_step_ms;
    uint32_t last_disp_ms;
    uint32_t last_uart_ms;

    /* ======================= 1. 外设初始化 ======================= */
    /* ★ 串口的初始化在 SYSCFG_DL_init() 里面, 所以打点只能从它【之后】开始。
     *   判断"有没有卡在开机时钟初始化"的办法:
     *      串口一直不出 [1] 这行  -> 就是卡在 SYSCFG_DL_init() 里 = 时钟没起来
     *      OLED 一直全黑          -> 同上(OLED 的 SPI 也在里面初始化) */
    SYSCFG_DL_init();
    DBG_MSG("\r\n=== BOOT ===\r\n");
    DBG_MSG("[1] SYSCFG_DL_init OK  (clock init did NOT hang)\r\n");

    tick_init();
    key_init();
    motor_init();
    Grayscale_Sensor_Init();
    OLED_Init();
    DBG_MSG("[2] tick/key/motor/grayscale/OLED OK\r\n");

    line_follow_init();
    DBG_MSG("[3] line_follow OK -> entering main loop\r\n");

    /* 故意不初始化 MPU6050: 本阶段不用, 而且它的 I2C 读没有超时保护 */

    /* ======================= 2. 开机画面 ======================= */
    /* 开机画面: 只是告诉人"我起来了", 顺便给 OLED 一点稳定时间。
     * (原来这里写的是 SENSOR DEBUG / UART2 PB15 / 115200 8N1,
     *  那是传感器调试阶段的内容, 串口早就关了, 留着会误导人。) */
    OLED_Clear();
    OLED_ShowString(0,  0, (u8 *)"LINE FOLLOW", 16);
    OLED_ShowString(0, 24, (u8 *)"K1 Run",      12);
    OLED_ShowString(0, 40, (u8 *)"K2 Test",     12);
    OLED_Refresh();
    delay_ms(600);

    OLED_Clear();               /* 擦掉开机画面, 免得和状态行错位留残余 */

    DBG_MSG("\r\n=== LINE FOLLOW ===\r\n");
    DBG_MSG("S=00011000 AD=111 E=-014 L=040 R=040\r\n");
    DBG_MSG("K1=follow on/off   K2=motor test\r\n");



    show_status();

    last_step_ms = tick_get_ms();
    last_disp_ms = last_step_ms;
    last_uart_ms = last_step_ms;

    /* ======================= 3. 主循环 ======================= */
    while (1)
    {
        uint32_t now = tick_get_ms();
        uint8_t  code = key_getnum();

        /* ---------------- 心跳: 证明 CPU 还活着 ----------------
         * ★ 这一段【不看时间, 只数循环圈数】, 所以就算 SysTick 停了它也照样打。
         * 卡住的时候看串口:
         *      心跳(HB)还在往外打 -> CPU 在跑, 是时基(SysTick)停了或者
         *                            哪个 (now - last) >= xx 的判断出问题
         *      心跳也停了         -> CPU 真卡住了
         *                            (如果是异常, 会先打 "!!! HARDFAULT !!!") */
        s_hb++;
        if (s_hb >= HB_LOOPS)
        {
            s_hb = 0U;
            DBG_MSG("HB\r\n");
        }

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
            DBG_MSG("[KEY1] ");
            DBG_MSG(line_follow_is_running() ? "follow ON\r\n" : "follow OFF\r\n");
        }

        /* ---------------- KEY2: 电机自检 ---------------- */
        else if (code == 2U)
        {
            line_follow_stop();
            s_test_mode = (uint8_t)((s_test_mode == 0U) ? 1U : 0U);
            line_follow_test_wheels(s_test_mode);
            show_status();
            DBG_MSG("[KEY2] ");
            DBG_MSG(s_test_mode ? "motor TEST on\r\n" : "motor TEST off\r\n");
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
            DBG_SENSOR();
        }

        /* ---------------- 每 100ms: 刷屏 ---------------- */
        if ((now - last_disp_ms) >= DISP_MS)
        {
            last_disp_ms = now;
            show_status();
        }


    }
}
