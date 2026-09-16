#include "sm_encoder.h"
#include "encoder.h"        /* encoder_E1A_PIN / encoder_1_A 等 */
#include "ti_msp_dl_config.h"
#include "encoder.h"            /* ★ 轮速编码器的计数也走这个 GROUP1 向量 */

/* ★★ GPIOA / GPIOB 的中断共用 IRQ 1, 也就是都进 GROUP1_IRQHandler ★★
 *
 * SysConfig 会把"同一个中断组里注册了中断的多个 GPIO 模块"合并成一个组,
 * 生成的名字形如 GPIO_MULTIPLE_GPIOA_INT_IRQN —— 它其实就是 GPIOA_INT_IRQn。
 *
 * ★ 坑(实测踩到): 这个【名字】会随 syscfg 里"哪些模块开了中断"而变 ——
 *   原来只有云台编码器开中断时它叫 sm_motor_encoder_GPIOA_INT_IRQN,
 *   给轮速编码器(E1A/E2A)也开中断之后, 名字变成了 GPIO_MULTIPLE_*,
 *   于是本文件第 99~102 行直接编译失败。
 * ★ 所以不要绑这个名字, 直接用芯片头文件里的 IRQ 号(它不会变)。
 *   ENC_GPIO_IRQN 定义在 hardware/encoder.h 里(本文件已经 include 了)。 */

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
 * GPIO 中断服务(GROUP1) —— 云台编码器 + 轮速编码器
 *  云台:  A1(PA15), B1(PA17), Z1(PA30), A2(PB22), B2(PB1), Z2(PA7)
 *  轮速:  E1A(PB20), E2A(PA25)      <- 循迹速度闭环用
 *
 * ★★ 这里必须【把两个端口的全部已使能标志都读出来, 处理完再全部清掉】★★
 *   原来是用掩码只读自己关心的那几位。这在只有云台编码器开了中断时没问题,
 *   但只要【再多一个模块】在同一个端口上开中断, 那些新位就永远清不掉 ——
 *   退出中断后它立刻再次触发 -> 中断反复重进 -> 表现就是"车卡死/跑不动"。
 *   这个坑很隐蔽: 编译没问题, 一烧进去就死。
 *   (实测: 给 E1A/E2A 打开中断时, 正是踩在这一条上)
 * -------------------------------------------------------------------------*/
/* ★★ 本函数【不再叫 GROUP1_IRQHandler】★★
 *   主程序 empty.c 里已经有一个 GROUP1_IRQHandler 了, 同名会【链接冲突】。
 *   现在真正在跑的是 empty.c 里那个: 它负责数轮速编码器的脉冲 + 清所有标志。
 *   ★ 云台以后要用的话, 把下面这段云台计数逻辑【合并进 empty.c 那个函数里】,
 *     不要再单独定义一个同名处理函数。 */
void sm_encoder_group1_isr(void)
{
    uint32_t sta_a, sta_b;
    /* 下面用 encoder_E1A_PIN / encoder_E2A_PIN 和 encoder_1_A / encoder_2_A,
       来自 encoder.h */

    s_isr_count++;

    sta_a = DL_GPIO_getEnabledInterruptStatus(GPIOA, 0xFFFFFFFFU);
    sta_b = DL_GPIO_getEnabledInterruptStatus(GPIOB, 0xFFFFFFFFU);

    /* --- 云台编码器(当前未接入主循环, 但标志必须照清) --- */
    if ((sta_b & sm_motor_encoder_A1_PIN) != 0U) enc_step(0U);
    if ((sta_b & sm_motor_encoder_B2_PIN) != 0U) enc_step(1U);
    if ((sta_b & sm_motor_encoder_Z1_PIN) != 0U) {
        s_enc[0].count = 0; s_enc[0].zero_seen = 1U;
    }
    if ((sta_b & sm_motor_encoder_Z2_PIN) != 0U) {
        s_enc[1].count = 0; s_enc[1].zero_seen = 1U;
    }
    if ((sta_a & sm_motor_encoder_B1_PIN) != 0U) enc_step(0U);
    if ((sta_a & sm_motor_encoder_A2_PIN) != 0U) enc_step(1U);

    /* --- ★ 轮速编码器: 只数 A 相(方向由命令决定, 不需要 B 相) ---
     *   E1A 在 GPIOB(PB20), E2A 在 GPIOA(PA25), 和上面云台那些不是一回事 */
    if ((sta_b & encoder_E1A_PIN) != 0U) { encoder_1_A++; }
    if ((sta_a & encoder_E2A_PIN) != 0U) { encoder_2_A++; }

    /* --- ★ 最后把读到的标志全部清掉, 一个都不留 --- */
    if (sta_a != 0U) { DL_GPIO_clearInterruptStatus(GPIOA, sta_a); }
    if (sta_b != 0U) { DL_GPIO_clearInterruptStatus(GPIOB, sta_b); }
}

/* ---------------------------------------------------------------------------
 * 初始化: 清状态, 使能 NVIC(注意 GPIOA/GPIOB 中断都走 GROUP1)
 * -------------------------------------------------------------------------*/
/* ★★ 注意函数名: 不能叫 encoder_init() ★★
 *   hardware/encoder.c(轮速编码器)里已经有一个 encoder_init() 了,
 *   同名会【链接失败】。这个模块是云台编码器, 所以叫 sm_encoder_init()。 */
void sm_encoder_init(void)
{
    s_enc[0].count = 0; s_enc[0].prev = 0; s_enc[0].zero_seen = 0;
    s_enc[0].last_sample = 0; s_enc[0].rpm = 0;
    s_enc[1].count = 0; s_enc[1].prev = 0; s_enc[1].zero_seen = 0;
    s_enc[1].last_sample = 0; s_enc[1].rpm = 0;

    /* ★ 直接用芯片头文件里的 IRQ 号, 不绑 SysConfig 那个会变的名字。
     *   GPIOA 和 GPIOB 本来就是同一个 IRQ 1, 使能一次两边都通。 */
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
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
