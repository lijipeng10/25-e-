#include "sm_encoder.h"
#include "ti_msp_dl_config.h"

/* MT6816: 1000 线 x 四倍频 = 4096 计数/圈 */
#define ENC_COUNTS_PER_REV   4000U

typedef struct {
    volatile int32_t  count;        /* 四倍频计数(带符号), 相对Z零点 */
    volatile int32_t  prev;         /* 上一次 (A,B) 状态 0~3 */
    volatile uint8_t  zero_seen;    /* 是否已见过 Z */
    volatile int32_t  last_sample;  /* 上次测速采样计数 */
    volatile int32_t  rpm;          /* 转速(RPM, 带符号) */
} EncState;

static volatile EncState s_enc[2];
static volatile uint32_t s_isr_count;    /* GROUP1_IRQHandler 被调次数(诊断) */

/* 四倍频方向查找表: 索引 = prev*4 + cur, 值 = +1 / -1 / 0
 *  bit0=A, bit1=B。正向序列 00->01->11->10->00, 反向相反。
 *  如果实际方向反了, 把表里 +/- 互换或加一个全局符号即可。 */
static const int8_t s_dir_tab[16] = {
     0, +1, -1,  0,   /* prev=00 */
    -1,  0,  0, +1,   /* prev=01 */
    +1,  0,  0, -1,   /* prev=10 */
     0, -1, +1,  0    /* prev=11 */
};

/* 读取某 GPIO 引脚的逻辑电平 */
static uint8_t enc_read_pin(GPIO_Regs *port, uint32_t pin)
{
    return ((DL_GPIO_readPins(port, pin) & pin) != 0U) ? 1U : 0U;
}

/* A/B 边沿中断里: 读当前 AB 状态, 由状态变化方向累加计数 */
static void enc_step(uint8_t axis)
{
    uint8_t a, b, cur;

    if (axis == 0U) {
        a = enc_read_pin(sm_motor_encoder_A1_PORT, sm_motor_encoder_A1_PIN);
        b = enc_read_pin(sm_motor_encoder_B1_PORT, sm_motor_encoder_B1_PIN);
    } else {
        a = enc_read_pin(sm_motor_encoder_A2_PORT, sm_motor_encoder_A2_PIN);
        b = enc_read_pin(sm_motor_encoder_B2_PORT, sm_motor_encoder_B2_PIN);
    }

    cur = (uint8_t)(a | (b << 1));
    s_enc[axis].count += (int32_t)s_dir_tab[(s_enc[axis].prev * 4) + cur];
    s_enc[axis].prev = (int32_t)cur;
}

/* ---------------------------------------------------------------------------
 * GPIO 中断服务(GROUP1)
 *  GPIOB: A1(双沿), Z1(上升), B2(双沿), Z2(上升)
 *  GPIOA: B1(双沿), A2(双沿)
 * -------------------------------------------------------------------------*/
void GROUP1_IRQHandler(void)
{
    uint32_t sta;

    s_isr_count++;

    /* GPIOB */
    sta = DL_GPIO_getEnabledInterruptStatus(GPIOB,
                sm_motor_encoder_A1_PIN | sm_motor_encoder_Z1_PIN |
                sm_motor_encoder_B2_PIN | sm_motor_encoder_Z2_PIN);
    if (sta != 0U) {
        if (sta & sm_motor_encoder_A1_PIN) enc_step(0U);
        if (sta & sm_motor_encoder_B2_PIN) enc_step(1U);
        if (sta & sm_motor_encoder_Z1_PIN) {
            s_enc[0].count = 0; s_enc[0].zero_seen = 1U;
        }
        if (sta & sm_motor_encoder_Z2_PIN) {
            s_enc[1].count = 0; s_enc[1].zero_seen = 1U;
        }
        DL_GPIO_clearInterruptStatus(GPIOB, sta);
    }

    /* GPIOA */
    sta = DL_GPIO_getEnabledInterruptStatus(GPIOA,
                sm_motor_encoder_B1_PIN | sm_motor_encoder_A2_PIN);
    if (sta != 0U) {
        if (sta & sm_motor_encoder_B1_PIN) enc_step(0U);
        if (sta & sm_motor_encoder_A2_PIN) enc_step(1U);
        DL_GPIO_clearInterruptStatus(GPIOA, sta);
    }
}

/* ---------------------------------------------------------------------------
 * 初始化: 清状态, 使能 NVIC(注意 GPIOA/GPIOB 中断都走 GROUP1)
 * -------------------------------------------------------------------------*/
void encoder_init(void)
{
    s_enc[0].count = 0; s_enc[0].prev = 0; s_enc[0].zero_seen = 0;
    s_enc[0].last_sample = 0; s_enc[0].rpm = 0;
    s_enc[1].count = 0; s_enc[1].prev = 0; s_enc[1].zero_seen = 0;
    s_enc[1].last_sample = 0; s_enc[1].rpm = 0;

    NVIC_ClearPendingIRQ(sm_motor_encoder_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(sm_motor_encoder_GPIOA_INT_IRQN);
    NVIC_ClearPendingIRQ(sm_motor_encoder_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(sm_motor_encoder_GPIOB_INT_IRQN);
}

int32_t encoder_get_count(uint8_t axis)
{
    return (axis < 2U) ? s_enc[axis].count : 0;
}

int32_t encoder_get_angle_x10(uint8_t axis)
{
    int32_t c;
    int64_t ang;
    if (axis >= 2U) return 0;
    c = s_enc[axis].count;
    /* 角度(0.1度) = count * 3600 / 每圈计数 */
    ang = ((int64_t)c * 3600LL) / (int64_t)ENC_COUNTS_PER_REV;
    return (int32_t)ang;
}

uint8_t encoder_isZeroSeen(uint8_t axis)
{
    return (axis < 2U) ? s_enc[axis].zero_seen : 0U;
}

void encoder_sample_speed(void)
{
    int32_t d0 = s_enc[0].count - s_enc[0].last_sample;
    int32_t d1 = s_enc[1].count - s_enc[1].last_sample;
    s_enc[0].last_sample = s_enc[0].count;
    s_enc[1].last_sample = s_enc[1].count;
    /* M法: RPM = n * 100 * 60 / 4000, n = 10ms 内计数 */
    s_enc[0].rpm = (int32_t)((((int64_t)d0) * 100LL * 60LL) / (int64_t)ENC_COUNTS_PER_REV);
    s_enc[1].rpm = (int32_t)((((int64_t)d1) * 100LL * 60LL) / (int64_t)ENC_COUNTS_PER_REV);
}

int32_t encoder_get_speed_rpm(uint8_t axis)
{
    return (axis < 2U) ? s_enc[axis].rpm : 0;
}

/* 诊断: GROUP1_IRQHandler 被触发的次数 */
uint32_t encoder_get_isr_count(void)
{
    return s_isr_count;
}
