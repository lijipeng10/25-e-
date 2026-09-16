#include "ti_msp_dl_config.h"
#include <stdint.h>
#include "key.h"            /* key_init / key_tick / key_getnum */
#include "motor.h"          /* motor_init / motor_set_direction / motor_set_duty / motor_pid_* */
#include "encoder.h"        /* encoder_get_speed / speed_1 / speed_2 */
#include "oled.h"
#include "tick.h"           /* tick_init / tick_get_ms */
#include "grayscale_sensor.h"   /* Grayscale_Sensor_Init / Grayscale_Sensor_Read_All */
#include "mpu6050.h"            /* mpu6050_ping / mpu6050_init / mpu6050_update */
#include "line_follow.h"        /* line_follow_init / line_follow_step / line_follow_get_* */

uint8_t keynum;
extern uint32_t encoder_1_A;
extern uint32_t encoder_2_A;
float speed_1 = 0;
float speed_2 = 0;

/* KEY1 阶梯加速: 每按一次升一档【目标速度】(mm/s) */
static const uint16_t speed_table[] = { 0U, 100U, 200U, 300U, 400U, 500U, 600U, 700U, 800U, 900U };
static u8 speed_index = 0U;

/* 灰度 8 路最近一次读数, 由 show_sensors() 每 100ms 刷一次 */
static uint16_t gray_buf[GRAYSCALE_SENSOR_CHANNELS];

/* MPU6050 的 I2C 地址: 0 = 没找到, 0x68 / 0x69 = 找到了 */
static uint8_t s_mpu = 0U;

/* 显示带符号整数: 1 位符号 + 3 位数字, 12px 字体共占 6+18 = 24 像素 */
static void show_signed3(u8 x, u8 y, int32_t v)
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

    if (mag > 999U)
    {
        mag = 999U;             /* 只占 3 位, 超了就截断, 免得压到右边的 Y */
    }

    OLED_ShowNum((u8)(x + 6U), y, mag, 3, 12);
}

/* MPU6050 必须固定 10ms 更新一次: 它内部按 tick 差值积分航向 */
/* 只在 100ms 的刷屏里调会让 dt 变大, 航向会跳 */
/* 循迹 step 也挂在这个 10ms 节拍上, 并且必须排在 mpu6050_update() 之后 */
static void mpu_tick(void)
{
    static uint32_t last = 0U;

    if ((tick_get_ms() - last) < 10U)
    {
        return;
    }

    last = tick_get_ms();

    mpu6050_update();

    /* ★ 必须排在 mpu6050_update() 【之后】: 循迹内环要用这一拍刚更新好的航向/角速度 */
    /* ★ 全程不调 line_follow_start() -> s_running 恒为 0, step() 只读灰度算偏差, 不驱动电机 */
    line_follow_step();
}

/* 显示: 每 100ms 读一次灰度 + 刷一次屏 */
static void show_sensors(void)
{
    static u32 last = 0;
    u8 bitmap[GRAYSCALE_SENSOR_CHANNELS + 1U];
    u8 i;

    if ((tick_get_ms() - last) < 100U)
    {
        return;
    }
    last = tick_get_ms();

    /* 内部约 400us 延时, 跟着刷屏 100ms 一次就够 */
    Grayscale_Sensor_Read_All(gray_buf);

    /* 第 1 行 16px: 标题 FOLLOW(6 字符 x 8px 占 x=0~47) + 当前档位 4 位(占 x=96~127) */
    OLED_ShowString(0, 0, (u8 *)"FOLLOW", 16);
    OLED_ShowNum(96, 0, speed_table[speed_index], 4, 16);

    /* 标题和档位中间的 x=48~95 是 16px 的 6 个字符位: 陀螺仪没接时在这里报 MPU:NO */
    /* 接上了就整段不画, 位置留空 —— 免得 Y 一直显示 +000 让人以为是"航向不动" */
    if (s_mpu == 0U)
    {
        OLED_ShowString(48, 0, (u8 *)"MPU:NO", 16);
    }

    /* 第 2 行 12px: 8 路灰度位图, 读到 = 1, 没读到 = 0 */
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

    OLED_ShowString(0, 16, (u8 *)"G", 12);
    OLED_ShowString(12, 16, bitmap, 12);        /* 8 位 x 6px 占 x=12~59 */

    /* 第 3 行 12px: 循迹偏差 error(线最左 -100 ~ 最右 +100), 符号 + 3 位占 x=12~35 */
    OLED_ShowString(0, 28, (u8 *)"E", 12);
    show_signed3(12, 28, (int32_t)line_follow_get_error());

    /* 第 4 行 12px: 外环给的目标航向 psi_ref, 换算成度(psi_ref 原始单位是 0.1 度, 除以 10) */
    /* ★ 单位必须和第 5 行的 Y 一致(都是度), 不然 P 和 Y 摆在一起没法直接比 */
    OLED_ShowString(0, 40, (u8 *)"P", 12);
    show_signed3(12, 40, (int32_t)(line_follow_get_psi_ref() / 10));

    /* 第 5 行 12px: 陀螺仪实测航向, 单位也是度(= yaw_x10 / 10), 可直接和上一行的 P 对着看 */
    OLED_ShowString(0, 52, (u8 *)"Y", 12);
    show_signed3(12, 52, (int32_t)(mpu6050_get_yaw_x10() / 10));

    OLED_Refresh();
}

int main(void)
{
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

    while (1)
    {
        keynum = key_getnum();

        if (keynum == 1U)
        {
            speed_index++;              /* 下一档 */

            if (speed_index >= (u8)(sizeof(speed_table) / sizeof(speed_table[0])))
            {
                speed_index = 0U;       /* 到头回到 0 */
            }

            motor_set_direction(1, 1);
            motor_set_direction(2, 1);

            motor_pid_set(1, (float)speed_table[speed_index]);
            motor_pid_set(2, (float)speed_table[speed_index]);
        }

        mpu_tick();             /* MPU6050: 固定 10ms 一次 */
        show_sensors();         /* 灰度 + 刷屏: 100ms 一次 */
    }
}

void GROUP1_IRQHandler(void)
{
    switch (DL_GPIO_getPendingInterrupt(GPIOB))
    {
        case encoder_E1A_IIDX:
            encoder_1_A++;
            encoder_2_A++;
            break;

        default:
            break;
    }
}

void key_encoder_INST_IRQHandler(void)
{
    DL_Timer_clearInterruptStatus(key_encoder_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);

    key_tick();                 /* 按键扫描 */

    encoder_get_speed(1);       /* 测速(顺便清零脉冲计数) */
    encoder_get_speed(2);

    motor_pid_update(1);
    motor_pid_update(2);
}
