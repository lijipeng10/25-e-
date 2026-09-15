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
 *      改 hardware/encoder.c 里的 MS_LEFT_FWD_DIR / MS_RIGHT_FWD_DIR。
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
 *          Emin+000 Emax+000            本次运行误差的最小/最大值(见下)
 *
 *      串口打印已由下面的 DBG_UART 关掉(= 0)。串口是"死等发完"的,
 *      一行 40 多字符 ≈ 3.8ms, 会占住主循环、把 10ms 控制周期拖出抖动。
 *      以后想用串口看数据, 把 DBG_UART 改回 1 就行。
 *
 *  【万一以后 S 又不动了, 按这个顺序隔离】
 *      1) 查通道选择脚 AD0/AD1/AD2 (PB24 / PA24 / PA26) 的接线
 *         (原来这里靠打印 AD= 的电平来判断, 但那个读数是假的, 已删除 ——
 *          原因见 pc_print_sensor() 里的说明: MSPM0 输出脚读不回来)
 *      2) 把传感器的 OUT 线从 PA22 上拔下来, 然后手动把 PA22 短接到 3.3V 和 GND
 *           - 短到 3.3V 时 S 变成 11111111, 短到 GND 变成 00000000
 *             -> 说明单片机这侧(PA22 输入)是好的, 问题在传感器模块/供电/接线
 *           - 短接也没反应 -> 问题在单片机这边的引脚配置
 *      3) 传感器模块单独查: 供电(VCC/GND)、OUT 是否接到了 PA22、
 *         模块上的指示灯会不会随黑白变化
 *
 *  【按键】
 *      KEY1 = 循迹 开/关
 *      KEY2 = 电机自检, 【每按一次升一档占空比】:
 *             0 -> 10 -> 12 -> 14 -> 16 -> 18 -> 20 -> 0 ...
 *             把车拿在手上一直按, 看哪一档轮子开始能持续转动 = 电机启动死区。
 *      屏幕最后一行的 Emin/Emax 是【本次运行】误差到过的最小/最大值 ——
 *      车在跑的时候盯不了屏幕, 所以停下来看这一行就知道它摆得多厉害:
 *          Emin/Emax 都在 ±30 以内 -> 控制稳
 *          Emin/Emax 到 ±70 以上   -> 车已经甩到线边上, 修正太晚/太弱
 *          正负都很大且对称        -> 典型来回超调(画龙), 该降 LF_POS_KP
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
 *          ★★★ 现在这些速度类的数都是【mm/s】, 不是占空比了 ★★★
 *              (电机已上【速度闭环】, 见 hardware/encoder.h)
 *          LF_BASE_SPEED    直行基础速度(mm/s)               300
 *                            ★ 必须台架试一次: 按 KEY2 看稳定后的 d1/d2,
 *                              合理值 12~17; 一直顶 20 = 定高了, 往下调
 *                           ★ 必须明显高于电机启动死区, 否则左右摆!
 *                             速度调低反而摆得更厉害 = 这个原因
 *          LF_LOST_SPEED    丢线找线速度(mm/s)               250
 *          LF_MAX_STEER     ★ 转向量上限 = 转弯力度(mm/s)         300
 *                           ★★ 必须 >= LF_BASE_SPEED, 理由见文件里的推导:
 *                             它决定"慢的一侧能降到多低", 降不到 0 就转不过弯
 *          ★★ 双环(航向环) —— 现在的主力调参就是前面这两个 ★★
 *          LF_POS_KP        外环: 位置误差 -> 目标航向(0.1度/格)    2
 *                           调大 = 回正更积极(但容易冲过头画龙)
 *                           调小 = 更温和(但小偏差时贴不上线)
 *                           ★ 它的天花板是 LF_POS_KP*100/10 度(误差被卡在 ±100),
 *                             取 2 就是 20 度 —— 所以【大角度转弯不是它的活】,
 *                             是原地转向(LF_PIVOT_*)的活
 *                           ★ 同时也是"弯道转速"的旋钮: 调小 = 转得更慢更稳
 *          LF_HEAD_KP       内环: 航向差 -> 转向量(每 10 度给多少)   5
 *                           调大 = 车头追目标更快更跟手(太大直道会抖)
 *                           调小 = 外环给了目标车头却跟不上("想转转不动")
 *          LF_GYRO_KD       陀螺仪阻尼(带符号, 已实测【左转为正】)  +6
 *                           ★ 调大能压摆尾, 但弯道上会转不动/转反 ——
 *                             原因和取舍见 line_follow.c 里这一项的推导
 *          LF_PSI_MAX       目标航向限幅(0.1度)                 600 (=60度)
 *          LF_DEADBAND      ★ 误差死区, 现在给 0, 【不要动它】
 *                           ★ 曾经以为它治左右摆, 实测【反而摆得更凶】:
 *                             它把"完全不管"和"猛踢一下"之间的过渡削掉了
 *
 *          --- 弯道: 停车原地转向再前进 ---
 *          LF_PIVOT_TRIGGER_MS  连续丢线多久判定"到弯节点"    150
 *          LF_PIVOT_SPEED       原地转向的速度(mm/s, 一正一反) 300
 *          LF_PIVOT_OK          |误差| 小于它就算"对准了"        43
 *                               ★ 从 20 放宽到 43: 原地转向是全车最快的旋转,
 *                                 窗口太窄线会整段穿过去抓不到 -> 一直转、找不到线
 *          LF_PIVOT_MAX_DEG     ★ 朝一边最多转这么多就掉头          110度
 *                               ★ 它同时是安全线: 要转到约 158 度才会重新
 *                                 看见来路, 110+22=132 够不着 -> 不会逆行
 *                               ★ 曾经还加过一个"最小角度门"(转过 40 度才许认线),
 *                                 【已删除】: 直道上误进原地转向时正确的线就在原地,
 *                                 有门槛就永远认不回来 -> 实车"直线都会掉头"
 *          LF_PIVOT_TRY_MS      纯兜底超时(只有没接陀螺仪时才用得到) 2500
 *                               (两个方向合计 1.8 秒封顶)
 *
 *      ★ 约束(违反了直接编译报错, 不会等跑车才发现):
 *          LF_LOST_SPEED <= LF_BASE_SPEED
 *          LF_MAX_STEER >= LF_BASE_SPEED     <- 转弯力度靠这条
 *          LF_PIVOT_SPEED >= LF_BASE_SPEED
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
#include "mpu6050.h"            /* mpu6050_ping / init / update / get_rate_x10 */
#include "encoder.h"            /* 轮速闭环: motor_speed_init/set/update/get */

/* ---- 循迹控制 ---- */
#include "line_follow.h"

/* ---------- 串口开关 ----------
 * ★ 这个开关只管【周期性刷屏】: 传感器数据行 + HB 心跳。
 *   为什么要能关: 串口是一个字符一个字符死等发完的, 一行 40 多字符 ≈ 3.8ms,
 *   这段时间主循环被占住, 循迹的 10ms 控制周期会被拖出抖动。
 *   对着电脑调参时开着(1); 正式放地上跑车建议关掉(0)。
 *
 * ★★ 启动日志(=== BOOT === / [1][2][3]) 和故障报告(!!! HARDFAULT !!!)
 *    【不受这个开关影响, 永远都会打印】——
 *    它们的用处就是"下次出问题能立刻定位", 不能因为跑车就关掉。
 *    代价极小: 开机只多打 4 行。详见 DEBUG.md。 */
/* ★ 现在改成 0: 调试信息全部走 OLED, 串口不再刷数据。
 *   注意: 启动日志(=== BOOT === / [1][2][3]) 和故障报告(!!! HARDFAULT !!!)
 *   【不受这个开关影响, 永远会打印】—— 它们排障要用, 见 DEBUG.md。 */
#define DBG_UART    0

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

/* ★ 串口"只在变化时打印"
 * 原来每 100ms 打一行, 一秒 10 行一模一样的内容 —— 调参时真正有用的变化
 * 全被这一堆重复行淹了, 这是调参阶段最大的干扰。
 * 改成: 数据(灰度位图 / 误差 / 左右轮占空比)一变就打; 连续 UART_SAME_MAX 拍
 *       没变就停住不打, 等它变。这样划传感器时日志是"跳着"出的, 一眼看得出。
 * 串口是否还活着由 HB 心跳负责, 不靠这里。 */
#define UART_SAME_MAX   10U     /* 连续 10 拍(1 秒)不变就不重复打了 */

static uint8_t s_test_mode = 0U;    /* KEY2 的电机自检开关(0 = 关) */

/* KEY2 的占空比档位, 每按一次升一档, 到头回到 0。
 * 用来量【电机启动死区】: 手拿着车一直按, 看哪一档轮子开始能持续转动。 */
/* ★ 已经是【速度(mm/s)】了, 不再是占空比档位。
 *   原来那组 (0,10,...,20) 是用来量"电机启动死区"的 —— 上了速度闭环之后
 *   死区由 PI 自己顶过去, 不用再手量。现在这一组是用来【验收闭环】的:
 *   命令多少, 屏幕上实测就该是多少。
 *   ★ 类型是 uint16_t: 600 装不进 uint8_t。 */
static const uint16_t k_test_levels[] = { 0U, 100U, 200U, 300U, 400U, 500U, 600U };
static uint8_t s_test_idx = 0U;     /* 当前在第几档 */

/* MPU6050 探测结果: 0 = 没找到; 0x68 / 0x69 = 找到的从机地址。
 * ★ 为什么不看串口: 用户要求不依赖串口, 所以直接画在参数页上,
 *   开机 4 秒那一屏就能看到 MPU:68 / MPU:69 / MPU:NO。 */
static uint8_t s_mpu_addr = 0U;

/* ---------- 心跳 ----------
 * 主循环每转一圈 s_hb 加 1, 加够了就翻转 LED2。
 * 关键: 它【不依赖 SysTick, 也不依赖任何定时器和中断】, 纯靠 CPU 转圈。
 * 所以它是判断"CPU 到底还活着没有"的唯一可靠手段 ——
 * 屏幕冻住的时候看 LED2: 还在闪 = CPU 在跑; 不闪 = CPU 真停了。
 * 数值随便, 只影响闪的快慢, 越大越慢。 */
#define HB_LOOPS    300000U
static uint32_t s_hb = 0U;          /* 心跳计数 */

/* 串口"只在变化时打印"用的状态: 上一拍打印过的值 + 连续没变的拍数。
 * 初值故意设成"不可能出现的值", 这样开机第一次一定打印。 */
static uint8_t  s_last_bits = 0xFFU;
static int16_t  s_last_err  = 0x7FFF;
static uint8_t  s_last_ld   = 0xFFU;
static uint8_t  s_last_rd   = 0xFFU;
static uint8_t  s_same_cnt  = 0U;

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
 *  串口输出 (PC_uart = UART2, TX = PB15, RX = PB16, 115200 8N1)
 * ----------------------------------------------------------------------------
 *  分成两层, 别混:
 *
 *  【第 1 层】基础发送 + 启动日志 + 故障报告  —— 永远编译、永远打印
 *      这些不需要也【不应该】被开关关掉, 它们就是排障的生命线。
 *
 *  【第 2 层】周期性刷屏(传感器行 + HB 心跳)  —— 由 DBG_UART 控制
 *      它们会占主循环时间, 正式跑车时可以关。
 *
 *  排障流程和每行日志的含义见工程根目录的 DEBUG.md。
 * ==========================================================================*/

/* ---------------- 第 1 层: 永远可用 ---------------- */

static void pc_putc(char c)
{
    while (DL_UART_isBusy(PC_uart_INST)) { }    /* 等上一个字节发完 */
    DL_UART_transmitData(PC_uart_INST, (uint8_t)c);
}

static void pc_puts(const char *s)
{
    while (*s != 0) { pc_putc(*s++); }
}

/* 启动日志 / 按键提示 / 故障报告都用它 —— 不受 DBG_UART 影响 */
#define DBG_MSG(s)          pc_puts(s)

/* ---------------- 第 2 层: 受 DBG_UART 控制 ---------------- */
#if DBG_UART

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

/* 打印一行状态:  S=00011000 E=-014 L=040 R=040 H=+012 P=-030
 *
 *   S   : 8 路灰度原始值, 从左到右, 1 = 黑线
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

    /* ★ 原来这里会打印 AD2/AD1/AD0 的电平(想着"读完 8 路后应该是 111"),
     *   用来判断通道选择有没有生效。但那个读数是【假的】, 已删除:
     *
     *   MSPM0 的 DL_GPIO_initDigitalOutput() 只写 PINCM 的 PC_CONNECTED + 功能号,
     *   并【不】置 INENA_ENABLE 位 —— 也就是说引脚配成普通输出时,
     *   输入缓冲是关掉的, DL_GPIO_readPins() 永远读回 0。
     *   (对比 dl_gpio.h: DL_GPIO_initDigitalInput() 才带 INENA_ENABLE)
     *
     *   所以 AD0/AD1/AD2 虽然确实在往外驱动, 但读不回来, 打出来的 000
     *   跟"有没有选通道"完全无关。判断通道选择是否生效, 只能靠 S= 的变化。 */

    pc_puts(" E=");
    pc_put_signed4(line_follow_get_error());

    /* 左右轮实际占空比 —— 悬空验证转向极性就看这两个数 */
    pc_puts(" L=");
    pc_put_u3(line_follow_get_left_duty());
    pc_puts(" R=");
    pc_put_u3(line_follow_get_right_duty());

    /* ★ 双环的两个关键观测量, 单位都是【度】, 左转为正:
     *     H = 车头现在实际朝哪(陀螺积分出来的航向角)
     *     P = 外环希望车头朝哪(目标航向 psi_ref)
     *   调参就看这两个数的关系:
     *     P 自己跳来跳去        -> 外环太猛, 降 LF_POS_KP
     *     H 老是追不上 P        -> 内环太弱, 加 LF_HEAD_KP
     *     H 冲过 P 再摆回来     -> 阻尼不够, 加 LF_GYRO_KD
     *   (屏幕上只有 H, 没有 P —— 128x64 已经排满了, 所以 P 走串口) */
    pc_puts(" H=");
    pc_put_signed4((int16_t)(mpu6050_get_yaw_x10() / 10));
    pc_puts(" P=");
    pc_put_signed4((int16_t)(line_follow_get_psi_ref() / 10));

    pc_puts("\r\n");
}

/* 主循环里统一用这两个宏调, 这样开关一改, 调用处不用动 */
#define DBG_SENSOR()        pc_print_sensor()
#define DBG_HB()            pc_puts("HB\r\n")

#else
/* DBG_UART = 0: 只有周期性刷屏变成空操作, 主循环里照旧调用即可 */
#define DBG_SENSOR()        ((void)0)
#define DBG_HB()            ((void)0)
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

/* ---------------------------------------------------------------------------
 *  电机速度环测试页(按 KEY2 进入) —— 这就是闭环的验收工具
 * ---------------------------------------------------------------------------
 *  显示每轮的【目标速度】和【实测速度】(mm/s), 以及 PI 给出的占空比。
 *  怎么看:
 *      目标 300 -> 实测应该稳在 300 附近(差几十是正常的, 看趋势会不会收敛)
 *      命令 0  -> 占空比也应该是 0, 轮子停住
 *      占空比会自己变 -> 那就是闭环在顶着死区、在补电池电压, 正是要的效果
 *  ★ 两个轮子给的是同一个目标速度, 所以实测的差异 = 真实机械/编码器差异。
 * -------------------------------------------------------------------------*/
static void show_test(void)
{
    OLED_ShowString(0, 0, (u8 *)"LF:TEST", 16);
    OLED_ShowNum(104, 0, s_test_idx, 1, 16);        /* 当前是第几档 */

    /* 1 号轮: 目标 和 实测 */
    OLED_ShowString(0, 16, (u8 *)"1>", 12);
    show_signed3(12, 16, (int16_t)motor_speed_get_target(1U), 12);
    OLED_ShowString(48, 16, (u8 *)"m", 12);
    show_signed3(60, 16, (int16_t)motor_speed_get(1U), 12);

    /* 2 号轮: 目标 和 实测 */
    OLED_ShowString(0, 28, (u8 *)"2>", 12);
    show_signed3(12, 28, (int16_t)motor_speed_get_target(2U), 12);
    OLED_ShowString(48, 28, (u8 *)"m", 12);
    show_signed3(60, 28, (int16_t)motor_speed_get(2U), 12);

    /* PI 给出的占空比 —— 这一行最能说明"闭环在干活" */
    OLED_ShowString(0, 40, (u8 *)"d1", 12);
    OLED_ShowNum(18, 40, motor_speed_get_duty(1U), 2, 12);
    OLED_ShowString(48, 40, (u8 *)"d2", 12);
    OLED_ShowNum(66, 40, motor_speed_get_duty(2U), 2, 12);

    /* ACT = 本模块在驱动; ERR = 编码器故障(命令有速度但实测≈0, 已切断该轮输出)
     * ★ 看到 ERR 就是【编码器没在计数】, 别怀疑参数 —— 先查接线/中断。 */
    OLED_ShowString(0, 52, (u8 *)"ACT", 12);
    OLED_ShowNum(24, 52, motor_speed_is_active(), 1, 12);
    if ((motor_speed_get_fault(1U) != 0U) || (motor_speed_get_fault(2U) != 0U)) {
        OLED_ShowString(48, 52, (u8 *)"1", 12);
        OLED_ShowString(54, 52, (u8 *)"ERR", 12);
        OLED_ShowNum(72, 52, motor_speed_get_fault(1U), 1, 12);
        OLED_ShowNum(84, 52, motor_speed_get_fault(2U), 1, 12);
    }

    OLED_Refresh();
}

static void show_status(void)
{
    uint16_t raw[GRAYSCALE_SENSOR_CHANNELS];

    /* ★ KEY2 测试模式: 换成电机速度环那一页, 不再画循迹的数据 */
    if (s_test_mode != 0U) {
        show_test();
        return;
    }

    /* 状态行: 一眼就能看出车现在在干什么 */
    if (line_follow_is_pivoting()) {
        OLED_ShowString(0, 0, (u8 *)"LF:PIVOT", 16);   /* 弯道: 原地转向中 */
    } else if (line_follow_is_running()) {
        OLED_ShowString(0, 0, (u8 *)"LF:RUN  ", 16);
    } else {
        OLED_ShowString(0, 0, (u8 *)"LF:STOP ", 16);
    }

    /* 状态行右边挂一个摆动计数: 本次运行 E 的符号翻了几次。
     * 摆得多但幅度小 -> 控制器太灵敏(降 KP);
     * 摆得少但幅度大 -> 控制器太弱(加 KP)。详见 s_e_flips 的说明。 */
    OLED_ShowString(72, 0, (u8 *)"F:", 16);
    OLED_ShowNum(88, 0, line_follow_get_error_flips(), 3, 16);

    line_follow_get_raw(raw);
    OLED_ShowString(0, 16, (u8 *)"S:", 12);
    show_raw(12, 16, raw, 12);

    OLED_ShowString(0, 28, (u8 *)"E:", 12);
    show_signed3(12, 28, line_follow_get_error(), 12);

    /* ★ 陀螺仪角速度(度/秒, 带符号)。
     *   两个用途:
     *     1) 调 LF_GYRO_KD 的【符号】—— 用手把车头往左转, 看 R 是正还是负
     *     2) 平时看摆尾有多猛: 摆动时这个数会在正负之间甩得很大 */
    OLED_ShowString(60, 28, (u8 *)"R:", 12);
    show_signed3(72, 28, (int16_t)(mpu6050_get_rate_x10() / 10), 12);

    OLED_ShowString(0, 40, (u8 *)"L:", 12);
    OLED_ShowNum(12, 40, line_follow_get_left_duty(), 3, 12);
    OLED_ShowString(36, 40, (u8 *)"R:", 12);
    OLED_ShowNum(48, 40, line_follow_get_right_duty(), 3, 12);

    /* ★ 航向角 H:(单位 度, 带符号, 【左转为正】)。
     *   这是双环新增的关键观测量, 调参时主要看它:
     *     正常: 平滑地变化, 抖动的幅度应该【比 R: 那一行小得多】
     *     跳变/锯齿: 外环太猛 -> 降 LF_POS_KP
     *     长期不回到 0 附近: 内环跟不上 -> 加 LF_HEAD_KP, 或者 LF_GYRO_KD 太大
     *   (外环给的目标航向 psi_ref 目前只在串口调试口打印, 屏幕上没位置了) */
    OLED_ShowString(72, 40, (u8 *)"H:", 12);
    show_signed3(84, 40, (int16_t)(mpu6050_get_yaw_x10() / 10), 12);

    /* 第 5 行: 本次运行的误差摆幅 (原来这里是按键提示, 信息量太低)
     * 车在跑的时候盯不了屏幕, 所以把 E 的最小/最大值记下来, 停下来再看。 */
    {
        int16_t e_mn, e_mx;
        line_follow_get_error_range(&e_mn, &e_mx);
        OLED_ShowString(0,  52, (u8 *)"Emin", 12);
        show_signed3(24, 52, e_mn, 12);
        OLED_ShowString(54, 52, (u8 *)"Emax", 12);
        show_signed3(78, 52, e_mx, 12);
    }

    OLED_Refresh();
}

/* ---------------------------------------------------------------------------
 *  参数页: 把这一版固件的【所有可调参数】画在屏幕上
 * ---------------------------------------------------------------------------
 *  开机先显示它 4 秒, 然后自动进状态页。
 *  这样不用翻源码、也不依赖串口, 一眼就能确认芯片里跑的到底是哪一组参数 ——
 *  调参时反复改值烧录, 这个特别省事。
 *  (嫌 4 秒太短就按一下复位再看一遍, 复位现在是可靠的)
 *
 *  布局(128x64): 5 行 x 2 列
 *      y=0   12px  BASE xx      ST   xx    基础速度 / 转向量上限
 *      y=13  12px  POS  xx      HED  xx    ★ 双环的两个主要增益
 *      y=26  12px  TRIM ±xx     CNR  xx    左右补偿 / 急弯判据
 *      y=39  12px  PIV  xxx     GY   ±xx   弯道判据 / 陀螺阻尼
 *      y=52  12px  PD   xx      MPU:xx     原地转向占空比 / 陀螺在不在
 *
 *  ★ 原来 y=13 放的是 KP 和 DB(死区): KP 现在拆成了 POS/HED 两个,
 *    DB 固定是 0、没有信息量, 两个槽位就都让出来了。
 *  ★ 目标航向 psi_ref 和航向角看【状态页】那一行 H:(见 show_status)。
 *
 *  想加参数: 在 line_follow.h 里加 LF_P_xxx 序号, line_follow.c 里补一行,
 *            然后在这里画出来 —— 屏幕只剩这几行, 要腾地方就删旧的。
 * -------------------------------------------------------------------------*/
static void show_params(void)
{
    uint16_t p[LF_P_COUNT];
    int16_t  t;

    line_follow_get_params(p);

    /* ★★ 数值全部改成 3 位: 速度类参数现在是 mm/s, 是三位数(如 300) ★★
     *   BASE/ST = 基础速度 / 转向量上限(mm/s)
     *   POS/HED = 外环 / 内环增益      TRIM = 左右补偿(已置 0, 闭环接管) */
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"BASE", 12);
    OLED_ShowNum(30, 0, p[LF_P_BASE], 3, 12);
    OLED_ShowString(66, 0, (u8 *)"ST", 12);
    OLED_ShowNum(90, 0, p[LF_P_STEER], 3, 12);

    OLED_ShowString(0, 13, (u8 *)"POS", 12);
    OLED_ShowNum(30, 13, p[LF_P_POS], 3, 12);
    OLED_ShowString(66, 13, (u8 *)"HED", 12);
    OLED_ShowNum(90, 13, p[LF_P_HEAD], 3, 12);

    OLED_ShowString(0, 26, (u8 *)"TRIM", 12);
    t = (int16_t)p[LF_P_TRIM];              /* 可能是负数, 要带符号画 */
    OLED_ShowChar(30, 26, (u8)((t < 0) ? (u8)-'-' : (u8)'+'), 12);
    if (t < 0) { t = (int16_t)(-t); }
    OLED_ShowNum(36, 26, (u32)t, 2, 12);
    OLED_ShowString(66, 26, (u8 *)"CNR", 12);       /* 急弯判据: 过弯冲过头的关键 */
    OLED_ShowNum(90, 26, p[LF_P_CORNER], 3, 12);

    OLED_ShowString(0, 39, (u8 *)"PIV", 12);
    OLED_ShowNum(30, 39, p[LF_P_PIV_TRIG], 3, 12);      /* 丢线多久判定到弯节点 */

    /* 陀螺仪阻尼(带符号, 治左右摆尾)。单位已变成 mm/s/(度/秒)。
     * ★ 符号【已实测确认】: 车头往左转 -> 状态页 R: 显示为正 -> 这里取正号。 */
    OLED_ShowString(66, 39, (u8 *)"GY", 12);
    t = (int16_t)p[LF_P_GYRO];
    OLED_ShowChar(90, 39, (u8)((t < 0) ? (u8)-'-' : (u8)'+'), 12);
    if (t < 0) { t = (int16_t)(-t); }
    OLED_ShowNum(96, 39, (u32)t, 3, 12);

    /* 原地转向速度(mm/s): 弯道原地转要克服静摩擦, 调小了拧不动、容易超时 */
    OLED_ShowString(0, 52, (u8 *)"PSPD", 12);
    OLED_ShowNum(30, 52, p[LF_P_PIV_DUTY], 3, 12);

    /* 陀螺仪在不在、用的哪个地址 —— 画在屏幕上, 不用看串口。
     * MPU:68 / MPU:69 都算正常(只是模块 AD0 脚接法不同), MPU:NO 才是没接上。 */
    if (s_mpu_addr == 0x69U) {
        OLED_ShowString(66, 52, (u8 *)"MPU:69", 12);
    } else if (s_mpu_addr != 0U) {
        OLED_ShowString(66, 52, (u8 *)"MPU:68", 12);
    } else {
        OLED_ShowString(66, 52, (u8 *)"MPU:NO", 12);
    }

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

    /* ★ 轮速闭环初始化: 清计数、建 PID、清中断标志、开 NVIC(GROUP1)。
     *   ★ 它默认【不激活】—— 上电后电机还是由 line_follow 用占空比控制;
     *     本模块只有在 motor_speed_set() 被调用之后才会去写电机。
     *   ★ 必须放在 SYSCFG_DL_init() 之后: 引脚和中断得先配好。 */
    motor_speed_init();
    DBG_MSG("[3] line_follow OK -> entering main loop\r\n");

    /* ======================= 3. 陀螺仪(用于循迹的阻尼项) =======================
     * ★ 顺序很重要: 先 ping 再 init。
     *   mpu6050_init() 里的零偏标定要做 200 次读取, 传感器没接的话每次都等满
     *   超时, 合计十几秒 —— 看起来就像死机。ping 只读一次(最坏 30ms)。
     * ★ 标定期间【车必须静止】(约 400ms), 所以这一步放在开机、电机还没转的时候。
     *   如果标定时车在动, 零偏会不准, 航向/角速度都会偏。 */
    s_mpu_addr = (uint8_t)mpu6050_ping();     /* 返回 0 / 0x68 / 0x69 */
    if (s_mpu_addr != 0U) {
        mpu6050_init();     /* 标定约 400ms, 期间车必须静止 */
    }
    /* 没接也不影响: 角速度恒为 0, 阻尼项自然失效, 其他功能照常。
     * 结果会在参数页上显示成 MPU:68 / MPU:69 / MPU:NO, 不用看串口。 */

    /* ======================= 2. 开机画面: 参数页 =======================
     * 开机先把【这一版固件的所有可调参数】画出来, 停 4 秒, 然后自动进状态页。
     * 这样不用翻源码、不用串口, 一眼就知道芯片里跑的是哪一组参数。
     * (原来这里显示的是 LINE FOLLOW / K1 Run / K2 Test, 信息量太低) */
    show_params();
    delay_ms(4000);

    OLED_Clear();               /* 擦掉开机画面, 免得和状态行错位留残余 */

    /* ★ 这里原来打印了两行【写死的样板数据】:
     *       S=00011000 E=-014 L=040 R=040 H=+012 P=-030
     *       K1=follow on/off   K2=motor test
     *   它们跟下面真实的数据行长得一模一样, 极容易看错(实测就被骗过一次)。
     *   所以只留"格式说明", 而且写成一眼能看出不是数据的样子。 */
    DBG_MSG("\r\n=== LINE FOLLOW ===\r\n");
    DBG_MSG("[format] S=8bits(1=on line)  E=-100..100  L/R=duty%%  HB=alive\r\n");



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
            DBG_HB();
        }

        /* ---------------- KEY1: 循迹 开/关 ---------------- */
        if (code == 1U)
        {
            s_test_mode = 0U;
            if (line_follow_is_running()) {
                line_follow_stop();
            } else {
                /* ★★ 注意: 这里【不能】再调 motor_speed_disable() ★★
                 *   循迹现在【就是】通过速度环输出电机的(命令单位 mm/s),
                 *   把它关掉车就不动了。
                 *   (上一版是开环占空比时才需要"交还控制权", 那一步已经作废) */
                line_follow_start();
            }
            show_status();
            DBG_MSG("[KEY1] ");
            DBG_MSG(line_follow_is_running() ? "follow ON\r\n" : "follow OFF\r\n");
        }

        /* ---------------- KEY2: 电机自检(升档量死区) ----------------
         * 每按一次占空比升一档, 到头回到 0:
         *      0 -> 10 -> 12 -> 14 -> 16 -> 18 -> 20 -> 0 ...
         * 用法: 把车【拿在手上】(轮子离地), 一直按 KEY2 升档,
         *       看哪一档两个轮子开始能【持续转动】—— 那个值就是电机启动死区。
         *       LF_BASE_DUTY 必须明显高于它, 否则会出现
         *       "速度调低反而左右摆得更凶"那种怪现象。
         * 屏幕上的 L: / R: 会实时显示当前档位的占空比。 */
        else if (code == 2U)
        {
            char buf[4];

            line_follow_stop();

            s_test_idx++;
            if (s_test_idx >= (uint8_t)(sizeof(k_test_levels) / sizeof(k_test_levels[0]))) {
                s_test_idx = 0U;
            }
            s_test_mode = (uint8_t)((k_test_levels[s_test_idx] != 0U) ? 1U : 0U);

            /* ★ 现在是【命令速度】, 不是直接给占空比 —— 占空比交给速度环自己算。
             *   两个轮子给的是【同一个速度】, 所以车如果还跑偏, 那就只剩
             *   机械和编码器的问题了(两个电机的差异已经被闭环拉平)。 */
            motor_speed_set(1U, (int32_t)k_test_levels[s_test_idx]);
            motor_speed_set(2U, (int32_t)k_test_levels[s_test_idx]);
            show_status();

            /* 打一行出来, 免得只靠屏幕看 */
            buf[0] = (char)('0' + (k_test_levels[s_test_idx] / 100U) % 10U);
            buf[1] = (char)('0' + (k_test_levels[s_test_idx] / 10U) % 10U);
            buf[2] = (char)('0' + (k_test_levels[s_test_idx]) % 10U);
            buf[3] = 0;
            DBG_MSG("[KEY2] speed target ");
            DBG_MSG(buf);
            DBG_MSG(" mm/s\r\n");
        }

        /* ---------------- 每 10ms: 循迹控制 ---------------- */
        if ((now - last_step_ms) >= STEP_MS)
        {
            last_step_ms = now;
            /* ★ 先更新陀螺仪再跑循迹: 阻尼项要用【这一拍】的角速度。
             *   传感器没接时这个函数会直接返回, 开销极小。 */
            mpu6050_update();
            /* ★ 速度环: 内部按 MS_LOOP_MS(20ms) 自己分频。
             *   没被 motor_speed_set() 激活时它直接返回, 不会和 line_follow 抢电机。 */
            motor_speed_update();
            line_follow_step();
        }

        /* ---------------- 每 100ms: 串口打印传感器(只在变化时) ----------------
         * 详见 UART_SAME_MAX 的说明: 一变就打; 连续 1 秒没变就停, 免得刷屏。 */
        if ((now - last_uart_ms) >= UART_MS)
        {
            uint8_t bits = line_follow_get_bits();
            int16_t err  = line_follow_get_error();
            uint8_t ld   = line_follow_get_left_duty();
            uint8_t rd   = line_follow_get_right_duty();

            last_uart_ms = now;

            if ((bits != s_last_bits) || (err != s_last_err) ||
                (ld   != s_last_ld)   || (rd  != s_last_rd))
            {
                s_last_bits = bits;  s_last_err = err;
                s_last_ld   = ld;    s_last_rd  = rd;
                s_same_cnt  = 0U;
                DBG_SENSOR();
            }
            else if (s_same_cnt < UART_SAME_MAX)
            {
                s_same_cnt++;
                DBG_SENSOR();
            }
            /* 连续 UART_SAME_MAX 拍都一样 -> 不打了, 等它变 */
        }

        /* ---------------- 每 100ms: 刷屏 ---------------- */
        if ((now - last_disp_ms) >= DISP_MS)
        {
            last_disp_ms = now;
            show_status();
        }


    }
}
