#include "ti_msp_dl_config.h"
#include <stdint.h>
#include "key.h"            /* key_init / key_getnum */
#include "motor.h"          /* motor_init / motor_pid_init / motor_test_step / motor_is_fault */
#include "encoder.h"        /* encoder_init + speed_1 / speed_2 (测速和脉冲中断都在 encoder.c) */
#include "oled.h"
#include "tick.h"           /* tick_init / tick_get_ms */
#include "grayscale_sensor.h"   /* Grayscale_Sensor_Init / GRAYSCALE_SENSOR_CHANNELS */
#include "mpu6050.h"            /* mpu6050_ping / mpu6050_init / mpu6050_update / mpu6050_get_yaw_x10 */
#include "line_follow.h"        /* line_follow_init / start / stop / step / get_* */

/* 屏幕刷新周期(ms)。只影响"看得多勤", 不影响控制 —— 控制跑在 10ms 那一拍上 */
#define SHOW_PERIOD_MS  100U

/* MPU6050 的 I2C 地址: 0 = 没找到, 0x68 / 0x69 = 找到了 */
static uint8_t s_mpu = 0U;

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

/* 显示: 循迹调试屏。全部 12px, 5 行分别占 y = 0 / 12 / 24 / 36 / 48 */
static void show_status(void)
{
    static u32 last = 0;
    u32 now = tick_get_ms();
    uint8_t bits = line_follow_get_bits();
    int16_t mn, mx;
    u8 bitmap[GRAYSCALE_SENSOR_CHANNELS + 1U];
    u8 i;

    if ((now - last) < SHOW_PERIOD_MS)
    {
        return;
    }
    last = now;

    /* 第 1 行: E = 循迹偏差(-100 线在最左 ~ +100 线在最右), P = 外环给的目标航向(度) */
    OLED_ShowString(0, 0, (u8 *)"E", 12);
    show_signed(6, 0, (int32_t)line_follow_get_error(), 3);
    OLED_ShowString(34, 0, (u8 *)"P", 12);
    show_signed(40, 0, (int32_t)(line_follow_get_psi_ref() / 10), 3);

    /* 第 2 行: Y = 陀螺仪实测航向(度, 能直接和 P 比), G = 8 路灰度位图(1 = 压线) */
    OLED_ShowString(0, 12, (u8 *)"Y", 12);
    show_signed(6, 12, (int32_t)(mpu6050_get_yaw_x10() / 10), 3);
    OLED_ShowString(34, 12, (u8 *)"G", 12);

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

    OLED_ShowString(40, 12, bitmap, 12);        /* 8 位 x 6px 占 x=40~87 */

    /* 第 3 行: 两个轮子的实测速度(mm/s)。★ 手转一个轮子, 只有对应的那个数会动;
     * 两个数总是一起动 = 编码器中断只读了一个端口(见 encoder.c 的 GROUP1_IRQHandler) */
    OLED_ShowString(0, 24, (u8 *)"L", 12);
    show_signed(6, 24, (int32_t)speed_1, 4);
    OLED_ShowString(62, 24, (u8 *)"R", 12);
    show_signed(68, 24, (int32_t)speed_2, 4);

    /* 第 4 行: 本次运行期间偏差到过的最小 / 最大值 —— 车在跑时没法盯屏幕, 停下来看这个判断摆得凶不凶 */
    line_follow_get_error_range(&mn, &mx);
    OLED_ShowString(0, 36, (u8 *)"Emn", 12);
    show_signed(18, 36, (int32_t)mn, 3);
    OLED_ShowString(52, 36, (u8 *)"Emx", 12);
    show_signed(70, 36, (int32_t)mx, 3);

    /* 第 5 行: 状态。★ 几条字符串都补齐成 11 个字符, 不然短的那条盖不掉长的, 屏幕上会留残字 */
    if (motor_is_fault() != 0U)
    {
        OLED_ShowString(0, 48, (u8 *)"ERR-ENCODER", 12);
    }
    else if (s_mpu == 0U)
    {
        OLED_ShowString(0, 48, (u8 *)"MPU:NO     ", 12);
    }
    else if (line_follow_is_pivoting() != 0U)
    {
        OLED_ShowString(0, 48, (u8 *)"PIVOT      ", 12);
    }
    else if (line_follow_is_running() != 0U)
    {
        OLED_ShowString(0, 48, (u8 *)"RUN        ", 12);
    }
    else
    {
        OLED_ShowString(0, 48, (u8 *)"STOP  K2=GO", 12);
    }

    OLED_Refresh();
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
    tick_init();

    OLED_Clear();           /* 清屏只做一次, 放循环里会闪 */

    Grayscale_Sensor_Init();

    /* 必须先 ping 再 init: 传感器不在时 init 里的标定要等十几秒, 像死机 */
    s_mpu = mpu6050_ping();

    if (s_mpu != 0U)
    {
        mpu6050_init();     /* 标定零偏, 这 ~400ms 车必须静止 */
    }

    /* ★ 10ms 分频【只在这一处做】: 陀螺仪积分和循迹 step 共用同一拍, 两处分频会各走各的 */
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
                line_follow_start();    /* ★ 按下的这一瞬间车要摆正、对着线, 那是 0 度 */
            }
        }

        if ((tick_get_ms() - last_10ms) >= 10U)
        {
            last_10ms = tick_get_ms();
            mpu6050_update();       /* 必须先更新陀螺仪 */
            line_follow_step();     /* 循迹要用这一拍刚更新好的航向 */
        }

        show_status();              /* 内部自带 100ms 限速 */
    }
}
