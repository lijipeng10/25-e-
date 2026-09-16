#include "ti_msp_dl_config.h"
#include <stdint.h>
#include "key.h"            /* key_init / key_getnum */
#include "motor.h"          /* motor_init / motor_pid_init / motor_test_step / motor_test_duty */
#include "encoder.h"        /* encoder_init (测速和脉冲中断都在 encoder.c) */
#include "oled.h"
#include "tick.h"           /* tick_init / tick_get_ms */
#include "grayscale_sensor.h"   /* Grayscale_Sensor_Init / Grayscale_Sensor_Read_All */
#include "mpu6050.h"            /* mpu6050_ping / mpu6050_init / mpu6050_update */
#include "line_follow.h"        /* line_follow_init / line_follow_step / line_follow_get_* */

/* 灰度 8 路最近一次读数, 由 show_imu() 每 100ms 刷一次 */
static uint16_t gray_buf[GRAYSCALE_SENSOR_CHANNELS];

/* MPU6050 的 I2C 地址: 0 = 没找到, 0x68 / 0x69 = 找到了 */
static uint8_t s_mpu = 0U;

/* 显示带符号整数: 1 位符号 + 5 位数字, 12px 字体共占 6+30 = 36 像素 */
static void show_signed5(u8 x, u8 y, int32_t v)
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

    /* 6 轴原始值是 int16, 最多 5 位数字, 正好占满这个宽度 */
    OLED_ShowNum((u8)(x + 6U), y, mag, 5, 12);
}

/* 显示: MPU6050 六轴原始值 + 8 路灰度, 每 100ms 刷一次屏 */
/* ★ 这一屏只用来"看传感器通不通", 不参与控制 */
static void show_imu(void)
{
    static u32 last = 0;
    int16_t raw[6];
    u8 bitmap[GRAYSCALE_SENSOR_CHANNELS + 1U];
    u8 i;

    if ((tick_get_ms() - last) < 100U)
    {
        return;
    }
    last = tick_get_ms();

    /* 内部约 400us 延时, 跟着刷屏 100ms 一次就够 */
    Grayscale_Sensor_Read_All(gray_buf);

    mpu6050_get_raw(raw);       /* raw[0..2] = AX AY AZ, raw[3..5] = GX GY GZ */

    /* 12px 一行放两个轴: 左列 标签 x=0 / 数值 x=14, 右列 标签 x=64 / 数值 x=78 */
    OLED_ShowString(0, 0, (u8 *)"AX", 12);
    show_signed5(14, 0, (int32_t)raw[0]);
    OLED_ShowString(64, 0, (u8 *)"AY", 12);
    show_signed5(78, 0, (int32_t)raw[1]);

    OLED_ShowString(0, 12, (u8 *)"AZ", 12);
    show_signed5(14, 12, (int32_t)raw[2]);
    OLED_ShowString(64, 12, (u8 *)"GX", 12);
    show_signed5(78, 12, (int32_t)raw[3]);

    OLED_ShowString(0, 24, (u8 *)"GY", 12);
    show_signed5(14, 24, (int32_t)raw[4]);
    OLED_ShowString(64, 24, (u8 *)"GZ", 12);
    show_signed5(78, 24, (int32_t)raw[5]);

    /* 第 4 行: 8 路灰度位图, 读到 = 1, 没读到 = 0 */
    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++)
    {
        if (gray_buf[i] != 0U)
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

    /* 第 5 行: 陀螺仪没接时报警 —— 否则一排 +00000 会被当成"读到了" */
    if (s_mpu == 0U)
    {
        OLED_ShowString(0, 48, (u8 *)"MPU:NO", 12);
    }

    OLED_Refresh();
}

int main(void)
{
    uint8_t keynum;
    uint32_t last_10ms = 0U;

    SYSCFG_DL_init();

    /* ★ 只初始化, 【不】调 line_follow_start(): 没启动时 step() 只读传感器、算偏差, 不动电机 */
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

    /* ★ 10ms 分频【只在这一处做】: 陀螺仪积分和循迹 step 共用同一拍。
     *   原来 empty.c 里的 mpu_tick() 已拆掉 —— 它的 10ms 分频搬到这个循环里,
     *   和 line_follow_step() 共用, 避免两个 10ms 分频器各走各的。
     *   因此【不】再给 mpu6050.c 加 mpu6050_poll(): 那会和这里重复分频,
     *   陀螺仪的 dt 和循迹的节拍会漂开, 而且多套一层反而更难读。
     *   mpu6050_update() 本身留在 mpu6050.c, 只是由这里按 10ms 节拍调。 */
    while (1)
    {
        keynum = key_getnum();

        if (keynum == 1U)
        {
            motor_test_step();      /* 阶梯加速测速(表和时间逻辑在 motor.c) */
        }

        if ((tick_get_ms() - last_10ms) >= 10U)
        {
            last_10ms = tick_get_ms();
            mpu6050_update();       /* 必须先更新陀螺仪 */
            line_follow_step();     /* 循迹要用这一拍刚更新好的航向 */
        }

        show_imu();                 /* 六轴原始值 + 灰度 + 刷屏: 内部自带 100ms 限速 */
    }
}
