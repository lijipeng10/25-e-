#include "led.h"

/*
 * LED 为高电平有效: 引脚输出高(HIGH) = 亮, 输出低(LOW) = 灭。
 * 若你的板子是低电平点亮, 把下面 setPins/clearPins 对调即可。
 * 注: SysConfig 里实例名是 "led", LED1/LED2 共用 led_PORT, 所以宏是
 *     led_PORT / led_LED1_PIN / led_LED2_PIN。
 */
void led_on(uint8_t id)
{
    if(id == 1)
    {
        DL_GPIO_setPins(led_PORT, led_LED1_PIN);
    }
    else if(id == 2)
    {
        DL_GPIO_setPins(led_PORT, led_LED2_PIN);
    }
}

void led_off(uint8_t id)
{
    if(id == 1)
    {
        DL_GPIO_clearPins(led_PORT, led_LED1_PIN);
    }
    else if(id == 2)
    {
        DL_GPIO_clearPins(led_PORT, led_LED2_PIN);
    }
}
