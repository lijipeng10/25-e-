/* ============================================================================
 *  empty.c —— 主程序(待开发初始版本)
 * ----------------------------------------------------------------------------
 *  本文件只做一件事: 把所有外设初始化好, 然后留一个空主循环给后续开发。
 *  要加功能就往最下面标了 TODO 的地方写, 不要在这里堆外设配置。
 *
 *  已初始化的外设:
 *      tick      1ms 时基(SysTick)         -> PID/超时/软件定时
 *      key       2 个按键(中断识别键码)     -> main_timer 50ms 中断里 key_tick()
 *      motor     TB6612 双直流电机         -> 小车左右轮
 *      grayscale 8 路灰度(循迹)            -> 循迹
 *      mpu6050   陀螺仪(航向)              -> 循迹纠偏
 *      oled      1.3寸 128x64 SH1106(SPI)  -> 显示状态
 *
 *  中断分配:
 *      main_timer(TIMG0, 50ms) -> key_tick()  扫键(见下方 IRQHandler)
 *      SysTick(1ms)            -> 由 tick.c 自己累加毫秒, 不用管
 *
 *  注意: 新增 GPIO 中断前先看 PROJECT.md —— GROUP1 已被 sm_encoder 占用。
 * ==========================================================================*/
#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ---- 基础库 ---- */
#include "tick.h"               /* tick_init / tick_get_ms */
#include "delay.h"              /* delay_ms / delay_us (忙等, 精度要求不高时用) */

/* ---- 外设模块 ---- */
#include "key.h"                /* key_init / key_getnum / key_get_state */
#include "motor.h"              /* motor_init / motor_set_direction / motor_set_duty / motor_stop */
#include "led.h"                /* led_on / led_off */
#include "grayscale_sensor.h"   /* Grayscale_Sensor_Init / Grayscale_Sensor_Read_All */
#include "mpu6050.h"            /* mpu6050_init / mpu6050_update / mpu6050_get_yaw_x10 */
#include "oled.h"               /* OLED_Init / OLED_ShowString / OLED_ShowNum / OLED_Refresh */

/* ---------------------------------------------------------------------------
 *  中断服务: main_timer(50ms, 由 key_init 配置成周期模式)里扫键
 * -------------------------------------------------------------------------*/
void main_timer_INST_IRQHandler(void)
{
    if (DL_Timer_getPendingInterrupt(main_timer_INST) == DL_TIMER_IIDX_ZERO)
    {
        key_tick();     /* 按键扫描, 识别键码 -> key_getnum() 取 */
    }
}

int main(void)
{
    /* ======================= 1. 外设初始化 ======================= */

    SYSCFG_DL_init();           /* SysConfig 生成: 时钟/引脚/外设, 必须最先调用 */

    tick_init();                /* SysTick 1ms 时基 */
    key_init();                 /* 按键: 配 main_timer 50ms 周期 + 开中断 + NVIC */
    motor_init();               /* TB6612: STBY 使能, PWM 占空比清零, 方向脚清零 */
    Grayscale_Sensor_Init();    /* 灰度传感器(循迹) */
    mpu6050_init();             /* MPU6050 陀螺仪(I2C0) */
    mpu6050_zero_yaw();         /* 上电把当前朝向设为目标 0° */
    OLED_Init();                /* OLED 显示屏(复位 + 初始化 + 清屏) */

    /* ======================= 2. 开机画面 ======================= */

    OLED_Clear();
    OLED_ShowString(0,  0, (u8 *)"SYSTEM READY",  16);
    OLED_ShowString(0, 20, (u8 *)"128x64 SH1106", 12);
    OLED_ShowString(0, 36, (u8 *)"wait for dev",  12);
    OLED_Refresh();

    led_off(1);
    led_off(2);

    /* ======================= 3. 主循环(待开发) ======================= */
    while (1)
    {
        /* TODO: 应用逻辑写在这里。常用接口速查:
         *
         *  [按键]  uint8_t code = key_getnum();   // 0=无, 1=KEY1, 2=KEY2
         *          if (code == 1U) { ... }
         *
         *  [电机]  motor_set_direction(1, 1);     // id=1(A路) 方向: 1=正转 2=反转 0=停
         *          motor_set_duty(1, 50);         // 占空比 0~100 (%)
         *          motor_stop(1);                 // 两路都停
         *
         *  [时基]  uint32_t now = tick_get_ms();          // 毫秒
         *          if (now - last >= 10U) { last = now; } // 每 10ms 执行一次
         *
         *  [陀螺]  mpu6050_update();              // 必须固定周期调用(建议 10ms)
         *          int32_t yaw = mpu6050_get_yaw_x10();   // 航向 0.1°
         *
         *  [灰度]  uint16_t g[GRAYSCALE_SENSOR_CHANNELS];
         *          Grayscale_Sensor_Read_All(g);  // 8 路, 值 1 = 压线
         *
         *  [显示]  OLED_ShowString(x, y, (u8 *)"text", 16);
         *          OLED_ShowNum(x, y, value, 位数, 16);
         *          OLED_Refresh();                // 改完显存必须刷新
         *
         *  [LED]   led_on(1) / led_off(1);
         */
    }
}
