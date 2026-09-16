/* ============================================================================
 *  empty.c —— 主程序 + 两个中断服务函数 + OLED 显示
 * ----------------------------------------------------------------------------
 *  按键:  KEY1 = 阶梯加速测速(轮流测两个轮子)   KEY2 = 循迹开始 / 停止
 *  控制:  每 10ms 跑一次 line_follow_step()  (灰度 -> 差速 -> 两个轮子的速度环)
 *  屏幕:  每 100ms 刷一次, 显示偏差/差速/轮速/灰度/状态
 *  ★ 陀螺仪【不在循迹回路里】: 只在开机时 ping + 标定零偏, 循迹过程完全不读。
 * ==========================================================================*/
#include "ti_msp_dl_config.h"
#include <stdint.h>
#include "key.h"            /* key_init / key_getnum / key_tick */
#include "motor.h"          /* motor_init / motor_pid_init / motor_pid_update / motor_is_fault */
#include "encoder.h"        /* encoder_init / encoder_get_speed / encoder_1_A / encoder_2_A / speed_1 / speed_2 */
#include "oled.h"
#include "tick.h"           /* tick_init / tick_get_ms */
#include "grayscale_sensor.h"   /* Grayscale_Sensor_Init / GRAYSCALE_SENSOR_CHANNELS */
#include "mpu6050.h"            /* mpu6050_ping / mpu6050_init —— 只给开机标定用 */
#include "line_follow.h"        /* line_follow_init / start / stop / step / get_* */

/* 屏幕刷新周期(ms)。只影响"看得多勤", 不影响控制 —— 控制跑在 10ms 那一拍上 */
#define SHOW_PERIOD_MS  100U

/* MPU6050 的 I2C 地址: 0 = 没找到, 0x68 / 0x69 = 找到了(现在只用来提示) */
static uint8_t s_mpu = 0U;

/* ============================================================================
 *  崩溃取证 —— 用来把"跑着跑着就退出程序了"分成三种情况, 不靠猜:
 *    A. 屏幕上又出现 BOOT 屏              -> MCU 复位了(不是卡死)
 *    B. 屏幕定格, 也没有 FAULT            -> CPU 卡在某个死等里
 *    C. 屏幕显示 !! FAULT !!              -> CPU 跑飞了(HardFault / NMI)
 *    D. 状态行在 RUN/LOST 之间正常变化     -> 程序活着, 是循迹本身的逻辑/参数问题
 *   (D 已经确认: 状态行显示 LOST, 且 L/R 有读数 -> 程序活着, 是"看不到线"导致的停车)
 * ==========================================================================*/

/* 1 = OLED 已经初始化好, 故障处理函数才能安全地往上写 */
static volatile uint8_t s_oled_ready = 0U;

/* 按【电机通道号】取实测速度: 1 = A路 -> speed_1, 2 = B路 -> speed_2。
 * ★ 通道号和"左/右轮"的对应关系在 line_follow.h 的 LF_LEFT_ID / LF_RIGHT_ID,
 *   这里【不要】直接写 speed_1 / speed_2, 否则屏幕上 L/R 会和真实轮子对不上。 */
static int32_t speed_of(uint8_t id)
{
    return (int32_t)((id == 1U) ? speed_1 : speed_2);
}

/* 显示带符号整数: 1 位符号 + digits 位数字, 12px 字体每位 6 像素 */
static void show_signed(u8 x, u8 y, int32_t v, u8 digits)
{
    u32 mag;

    if (v < 0)
    {
        mag = (u32)(-v);
        OLED_ShowChar(x, y, (u8)'-', 12);
    }
    else
    {
        mag = (u32)v;
        OLED_ShowChar(x, y, (u8)'+', 12);
    }

    OLED_ShowNum((u8)(x + 6U), y, mag, digits, 12);
}

/* 显示: 循迹调试屏。5 行 12px, 分别占 y = 0 / 12 / 24 / 36 / 48 */
static void show_status(void)
{
    static u32 last = 0;
    u32 now = tick_get_ms();
    uint8_t bits = line_follow_get_bits();
    u8 bitmap[GRAYSCALE_SENSOR_CHANNELS + 1U];
    u8 i;

    if ((now - last) < SHOW_PERIOD_MS)
    {
        return;
    }
    last = now;

    /* 第 1 行: E = 偏差(-100 线在最左 ~ +100 线在最右), S = 差速量(mm/s) */
    OLED_ShowString(0, 0, (u8 *)"E", 12);
    show_signed(6, 0, (int32_t)line_follow_get_error(), 3);
    OLED_ShowString(40, 0, (u8 *)"S", 12);
    show_signed(46, 0, (int32_t)line_follow_get_steer(), 3);

    /* 第 2 行: 小写 l / r = 两个轮子的【命令】速度 mm/s。
     * ★ 车停着(不按 KEY2)也在更新 —— 把车压在线上左右挪, 看是不是
     *   "线偏右 -> l 大、r 小", 转向方向对不对一测就知道, 不用让车跑 */
    OLED_ShowString(0, 12, (u8 *)"l", 12);
    show_signed(6, 12, (int32_t)line_follow_get_cmd_left(), 4);
    OLED_ShowString(40, 12, (u8 *)"r", 12);
    show_signed(46, 12, (int32_t)line_follow_get_cmd_right(), 4);

    /* 第 3 行: 大写 L / R = 两个轮子的【实测】速度 mm/s(编码器测的)。
     * ★ 手转一个轮子, 只有对应的那个数会动 */
    OLED_ShowString(0, 24, (u8 *)"L", 12);
    show_signed(6, 24, speed_of(LF_LEFT_ID), 4);
    OLED_ShowString(40, 24, (u8 *)"R", 12);
    show_signed(46, 24, speed_of(LF_RIGHT_ID), 4);

    /* 第 4 行: 8 路灰度位图, bit0(最左) 在最左边, 1 = 压线 */
    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++)
    {
        if ((bits & (uint8_t)(1U << i)) != 0U)
        {
            bitmap[i] = (u8)'1';
        }
        else
        {
            bitmap[i] = (u8)'0';
        }
    }
    bitmap[GRAYSCALE_SENSOR_CHANNELS] = (u8)'\0';

    OLED_ShowString(0, 36, (u8 *)"G", 12);
    OLED_ShowString(12, 36, bitmap, 12);        /* 8 位 x 6px 占 x=12~59 */

    /* 第 4 行右边: 本次运行 |error| 的最大值(0~100)。
     * ★ 顶到 100 就是线已经甩到传感器最边上 —— 说明车真的跑偏出线了 */
    OLED_ShowString(64, 36, (u8 *)"|E|", 12);
    OLED_ShowNum(84, 36, (u32)line_follow_get_error_max_abs(), 3, 12);

    /* 第 5 行: 状态 + 已经跑了多少毫秒。
     * ★★ 每条都正好铺满 11 个字符(66 像素), 不然短的那条盖不掉长的, 屏上留残字。
     * ★ 毫秒数是关键诊断: 几百毫秒就 LOST = 一起步就跑偏; 跑了几秒才 LOST = 能跟一段 */
    if (motor_is_fault() != 0U)
    {
        OLED_ShowString(0, 48, (u8 *)"ERR-ENCODER", 12);        /* 11 字符 */
    }
    else if ((line_follow_is_running() == 0U) && (line_follow_is_lost() == 0U))
    {
        OLED_ShowString(0, 48, (u8 *)((s_mpu == 0U) ? "MPU:NO K2GO" : "STOP  K2=GO"), 12);
    }
    else
    {
        /* 5 字符标题 + 6 位毫秒 = 11 字符, 正好铺满 */
        OLED_ShowString(0, 48, (u8 *)((line_follow_is_lost() != 0U) ? "LOST " : "RUN  "), 12);
        OLED_ShowNum(30, 48, (u32)line_follow_get_run_ms(), 6, 12);
    }

    OLED_Refresh();
}

void GROUP1_IRQHandler(void)
{
    switch (DL_GPIO_getPendingInterrupt(GPIOA))
    {
        case encoder_E2A_IIDX:
            /* 右轮(B路): A 跳变时看 B */
            if (DL_GPIO_readPins(encoder_E2B_PORT, encoder_E2B_PIN) != 0)
            {
                encoder_2_A += ENCODER_2_SIGN;
            }
            else
            {
                encoder_2_A -= ENCODER_2_SIGN;
            }
            break;

        default:
            break;
    }

    switch (DL_GPIO_getPendingInterrupt(GPIOB))
    {
        case encoder_E1A_IIDX:
            /* 左轮(A路): A 跳变时看 B */
            if (DL_GPIO_readPins(encoder_E1B_PORT, encoder_E1B_PIN) != 0)
            {
                encoder_1_A += ENCODER_1_SIGN;
            }
            else
            {
                encoder_1_A -= ENCODER_1_SIGN;
            }
            break;

        default:
            break;
    }
}

/* 50ms 定时器(key_encoder = TIMG7)的"计数到 0"中断:
 * 扫按键 -> 测速(顺便清零脉冲) -> 两个轮子的速度环。
 * ★ 名字 key_encoder_INST_IRQHandler 由 SysConfig 生成, 不要改名。 */
void key_encoder_INST_IRQHandler(void)
{
    DL_Timer_clearInterruptStatus(key_encoder_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);

    key_tick();                 /* 按键扫描 */

    encoder_get_speed(1);       /* 测速: mm/s, 单位换算在 encoder.c */
    encoder_get_speed(2);

    motor_pid_update(1);        /* 速度环: 每 50ms 调一次 */
    motor_pid_update(2);
}

/* CPU 跑飞(HardFault)。★ 名字由启动文件的弱定义引用, 不要改名。
 * ★★ 屏幕上出现 !! FAULT !! 就是这里进来的 —— 和"卡死"是两回事, 要分开看 */
void HardFault_Handler(void)
{
    if (s_oled_ready != 0U)
    {
        OLED_Clear();
        OLED_ShowString(0, 0, (u8 *)"!! FAULT !!", 16);
        OLED_Refresh();
    }

    while (1)
    {
        /* 停在这里让屏幕定格, 方便人看见 */
    }
}

/* 不可屏蔽中断(时钟失效之类): 和跑飞同样处理 */
void NMI_Handler(void)
{
    HardFault_Handler();
}

int main(void)
{
    uint8_t keynum;
    uint32_t last_10ms = 0U;

    SYSCFG_DL_init();

    /* ★ 只初始化, 【不】启动循迹: 上电后车必须是停着的, 要按 KEY2 才动 */
    line_follow_init();

    motor_init();
    motor_pid_init();       /* 速度闭环: 建两个轮子的 PID */
    key_init();             /* 配 key_encoder 定时器(周期模式 + ZERO 中断) */

    encoder_init();
    OLED_Init();
    s_oled_ready = 1U;      /* 从这里起, 故障处理函数可以往屏幕上写字了 */
    tick_init();

    OLED_Clear();           /* 清屏只做一次, 放循环里会闪 */

    Grayscale_Sensor_Init();

    /* 陀螺仪只在开机标定一次(要 ~400ms, 此时车必须静止);
     * ★ 必须先 ping 再 init: 传感器不在时 init 里的标定要等十几秒, 像死机。
     * 循迹回路里【不读它】—— 少一个变量就少一个怀疑对象 */
    s_mpu = mpu6050_ping();

    if (s_mpu != 0U)
    {
        mpu6050_init();
    }

    /* ---------- 开机屏: 一直显示到按任意键 ----------
     * ★★ 跑着跑着又看见这个屏 = MCU 复位过(不是卡死)。这就是要的证据。
     *    屏幕会【停在这里不动】, 不会自己消失, 所以跑不掉 ★★ */
    OLED_ShowString(0, 0, (u8 *)"BOOT", 16);
    OLED_ShowString(0, 24, (u8 *)"press any key", 12);
    OLED_ShowString(0, 40, (u8 *)"K1=TEST K2=GO", 12);
    OLED_Refresh();

    while (key_getnum() == 0U)
    {
        /* 等按键 */
    }

    /* ★ 10ms 分频只在这一处做 */
    while (1)
    {
        keynum = key_getnum();

        if (keynum == 1U)
        {
            line_follow_stop();     /* ★ 测速和循迹都驱动电机, 必须先互斥 */
            motor_test_step();      /* 阶梯加速测速(表和时间逻辑在 motor.c) */
        }

        /* ★ KEY2: 按一下开始循迹, 再按一下停。编码器故障的自锁也在这里解 */
        if (keynum == 2U)
        {
            if (line_follow_is_running() != 0U)
            {
                line_follow_stop();
            }
            else
            {
                motor_fault_clear();

                if (s_mpu != 0U)
                {
                    mpu6050_zero_yaw();     /* 起步瞬间的航向当 0 度, 以后要用再取 */
                }

                line_follow_start();
            }
        }

        if ((tick_get_ms() - last_10ms) >= 10U)
        {
            last_10ms = tick_get_ms();
            line_follow_step();     /* 灰度 -> 差速 -> 速度环。陀螺仪不参与 */
        }

        show_status();              /* 内部自带 100ms 限速 */
    }
}
