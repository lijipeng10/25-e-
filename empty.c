#include "ti_msp_dl_config.h"
#include <stdint.h>
#include "key.h"            /* key_init / key_tick / key_getnum */
#include "motor.h"          /* motor_init / motor_set_direction / motor_set_duty */
#include "encoder.h"          /* encoder_get_speed */

uint8_t keynum;
extern uint32_t encoder_1_A;
extern uint32_t encoder_2_A;
extern float speed_1 = 0;
extern float speed_2 = 0;

int main(void)
{
    SYSCFG_DL_init();
    motor_init();
    encoder_init();

    while (1)
    {
        motor_set_direction(1, 1);
        motor_set_duty(1, 100);
        motor_set_direction(2, 1);
        motor_set_duty(2, 100);
    }
}

void GROUP1_IRQHandler(void)
{
    switch (DL_GPIO_getPendingInterrupt(GPIOB))
    {
        case encoder_E1A_IIDX:
            encoder_1_A ++;
            encoder_2_A ++;
            break;

        default:
            break;

    }
}