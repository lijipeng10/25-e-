#include "encoder.h"
#include "ti_msp_dl_config.h"
#include "motor.h"
#include "pid.h"
#include "tick.h"

uint32_t encoder_1_A;
uint32_t encoder_2_A;
float speed_1 = 0;
float speed_2 = 0;

void encoder_init(void)
{
    // 使能 GPIOB 中断
    NVIC_EnableIRQ(encoder_GPIOB_INT_IRQN);
    // 清除定时器挂起位并启动，使能定时器中断
    NVIC_ClearPendingIRQ(encoder_GPIOB_INT_IRQN);
    DL_Timer_startCounter(key_encoder_INST);
    NVIC_EnableIRQ(key_encoder_INST_INT_IRQN);
}

void encoder_get_speed(uint8_t id) 
{
    if(id == 1)
    {
        speed_1 = (float)encoder_1_A / ENCODER_PULSE * ENCODER_WHEEL_D * PI * 20;   // mm/s
        encoder_1_A = 0; // 清零
    }
    else if(id == 2)
    {
        speed_2 = (float)encoder_2_A / ENCODER_PULSE * ENCODER_WHEEL_D * PI * 20;   // mm/s
        encoder_2_A = 0; // 清零
    }
}

// void key_encoder_INST_IRQHandler()
// {
//     switch(DL_Timer_getPendingInterrupt(key_encoder_INST))
//     {
//         case DL_TIMER_IIDX_LOAD:
//             encoder_get_speed(1);
//             encoder_get_speed(2);
//             break;
//        
//         default:
//             break;
//     }
// }