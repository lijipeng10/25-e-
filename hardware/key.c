#include "key.h"
#include "encoder.h"        /* encoder_get_speed */
#include "motor.h"          /* motor_pid_update */

extern int status;
uint8_t Key_Num = 0;    /* 键码 */

void key_init(void)
{
    /* 必须清零: DL_Timer_initTimerMode 会读 genIntermInt / counterVal 写寄存器,
       不清零就是栈上的随机值, 会把定时器配坏(表现为中断永远不进)。 */
    DL_Timer_TimerConfig cfg = { 0 };

    /* 按键扫描时基: key_encoder, 用 SysConfig 配的装载值(50ms), 配成周期模式并启动。
       生成的代码是"一次性 + 不启动", 不重新配的话计数器起来就停, 中断进不去。 */
    cfg.timerMode  = DL_TIMER_TIMER_MODE_PERIODIC;
    cfg.period     = key_encoder_INST_LOAD_VALUE;
    cfg.startTimer = DL_TIMER_START;
    DL_Timer_initTimerMode(key_encoder_INST, &cfg);

    /* 开"计数到 0"中断, 中断里调 key_tick() 扫键 */
    DL_Timer_clearInterruptStatus(key_encoder_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
    DL_Timer_enableInterrupt(key_encoder_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);

    NVIC_ClearPendingIRQ(key_encoder_INST_INT_IRQN);
    NVIC_EnableIRQ(key_encoder_INST_INT_IRQN);
}

uint8_t key_getnum(void)
{
    uint8_t Temp;

    if (Key_Num)
    {
        Temp = Key_Num;
        Key_Num = 0;
        return Temp;
    }

    return 0;
}

uint8_t key_get_state(void)
{
    if (DL_GPIO_readPins(key_KEY1_PORT, key_KEY1_PIN) == 0)
    {
        return 1;
    }
    else if (DL_GPIO_readPins(key_KEY2_PORT, key_KEY2_PIN) == 0)
    {
        return 2;
    }

    return 0;
}

void key_tick(void)
{
    static uint8_t Count;
    static uint8_t CurrState, PrevState;

    Count++;
    if (Count >= 1)
    {
        Count = 0;

        PrevState = CurrState;
        CurrState = key_get_state();

        if (CurrState == 0 && PrevState != 0)
        {
            Key_Num = PrevState;
        }
    }
}

/* 按键扫描时基(key_encoder = TIMG7)的"计数到 0"中断。
   原来写在 empty.c 里 —— 用哪个定时器、干什么活, 都是按键模块自己的事。
   ★ 名字 key_encoder_INST_IRQHandler 由 SysConfig 生成, 不要改名。 */
void key_encoder_INST_IRQHandler(void)
{
    DL_Timer_clearInterruptStatus(key_encoder_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);

    key_tick();                 /* 按键扫描 */

    encoder_get_speed(1);       /* 测速(顺便清零脉冲计数) */
    encoder_get_speed(2);

    motor_pid_update(1);
    motor_pid_update(2);
}
