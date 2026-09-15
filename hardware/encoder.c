/* ============================================================================
 *  encoder.c —— 轮速闭环(编码器测速 + 每轮一个速度 PI)
 * ----------------------------------------------------------------------------
 *  原理、用法、和 line_follow 的互斥关系, 见 encoder.h 的开头。
 *  接线: E1A = PB20(GPIOB), E2A = PA25(GPIOA) —— 只数 A 相, 双沿。
 * ==========================================================================*/
#include "encoder.h"
#include "ti_msp_dl_config.h"

#include "motor.h"              /* motor_set_direction / motor_set_duty */
#include "pid.h"                /* 复用现成的通用 PID */
#include "tick.h"               /* tick_get_ms */

/* ============================================================================
 *  可调参数
 * ==========================================================================*/

/* ---------- 编码器换算 ----------
 * 【已知】编码器 13 线, 电机减速比 1:20
 *      -> 轮子转一圈, A 相出 13*20 = 260 个脉冲
 *      -> A 相取【双沿】, 一圈 520 个计数
 * ★★ 这两个数【必须台架实测确认】(见文档的"台架标定"), 猜错了速度会整体
 *    差一个固定系数 —— 表现就是"命令 300, 屏幕显示 260"这种。 */
#define MS_COUNTS_PER_REV   (260 * 2)       /* 520 计数 / 轮子一圈 */
#define MS_WHEEL_D_MM       48              /* 轮径 mm。★ 实测值, 别照抄 */
#define MS_WHEEL_C_MM       ((MS_WHEEL_D_MM * 314) / 100)   /* 周长 ≈ 150 mm */

/* ---------- 速度环 ----------
 * 【量纲】out = (MS_KP*err + MS_KI*积分/100) / 100
 *      err 单位 mm/s, out 单位占空比(0~MS_MAX_DUTY)
 *   参考: err = 100 mm/s 时 P 项 = 8*100/100 = 8 个占空比单位。
 *   ★ I 项负责把占空比顶过【死区】(实测 <=10) —— 这正是闭环的价值所在。
 *   ★ 调法: 跟不上/超调 -> 动 MS_KP; 稳态总是差一截 -> 动 MS_KI。 */
#define MS_KP               8
#define MS_KI               1
#define MS_I_CLAMP          60000
#define MS_MAX_DUTY         20      /* 占空比硬顶(和 LF_MAX_DUTY 一致) */
#define MS_MAX_MM_S         900     /* 目标速度上限, 防止误传一个离谱的值 */
#define MS_LOOP_MS          20U     /* 速度环周期(ms)。★ 别调太小:
                                       低速档才够计数。300mm/s 时 20ms 约 20 个计数。 */

/* ---------- 轮子映射 ----------
 * ★ 必须和 line_follow.c 里的 LF_LEFT_FWD_DIR / LF_RIGHT_FWD_DIR 一致:
 *      A路(1号) 前进 = 1,  B路(2号) 前进 = 2
 *   如果发现某个轮子方向反了, 改这里。 */
#define MS_LEFT_FWD_DIR     1U
#define MS_RIGHT_FWD_DIR    2U

/* ============================================================================
 *  内部状态
 * ==========================================================================*/
static volatile int32_t s_count[2];     /* 中断里累加的 A 相边沿计数 */
static Pid      s_pid[2];
static int32_t  s_target[2];            /* 目标速度 mm/s(带符号) */
static volatile int32_t s_meas[2];      /* 实测速度 mm/s(带符号, 符号来自命令方向) */
static volatile int8_t  s_dir[2];       /* 最近一次的转动方向: +1 / -1 / 0 */
static uint16_t s_duty[2];              /* PI 给出的占空比(0~MS_MAX_DUTY) */
static uint32_t s_last_ms;
static uint8_t  s_active;               /* 1 = 本模块正在驱动电机 */

/* ============================================================================
 *  中断部分
 * ==========================================================================*/
void motor_speed_isr(uint32_t sta_gpioa, uint32_t sta_gpiob)
{
    /* E1A 在 GPIOB(PB20), E2A 在 GPIOA(PA25) */
    if ((sta_gpiob & encoder_E1A_PIN) != 0U) { s_count[0]++; }
    if ((sta_gpioa & encoder_E2A_PIN) != 0U) { s_count[1]++; }
}

/* ============================================================================
 *  初始化
 * ==========================================================================*/
void motor_speed_init(void)
{
    uint8_t i;

    for (i = 0U; i < 2U; i++) {
        s_count[i]  = 0;
        s_target[i] = 0;
        s_meas[i]   = 0;
        s_dir[i]    = 0;
        s_duty[i]   = 0U;
        /* pid_init(&pid, kp, ki, kd, div, 积分限幅, 输出限幅, 死区, 最小dt) */
        pid_init(&s_pid[i], MS_KP, MS_KI, 0, 100,
                 MS_I_CLAMP, MS_MAX_DUTY, 0, MS_LOOP_MS);
    }
    s_last_ms = 0U;
    s_active  = 0U;         /* ★ 默认【不激活】: 交给 line_follow 用占空比控制 */

    /* ★★ 开 NVIC 之前必须先把中断标志清干净 ★★
     *   syscfg 里云台编码器(A1/B1/Z1/A2/B2/Z2)的引脚中断也是使能的,
     *   上电后它们的标志可能已经置起来了。不清就直接开 NVIC, 会立刻进一次中断。 */
    DL_GPIO_clearInterruptStatus(GPIOA, 0xFFFFFFFFU);
    DL_GPIO_clearInterruptStatus(GPIOB, 0xFFFFFFFFU);
    NVIC_ClearPendingIRQ(ENC_GPIO_IRQN);
    NVIC_EnableIRQ(ENC_GPIO_IRQN);
}

/* ============================================================================
 *  命令接口
 * ==========================================================================*/
void motor_speed_set(uint8_t id, int32_t mm_s)
{
    uint8_t idx;
    uint8_t dir;

    if ((id < 1U) || (id > 2U)) { return; }
    idx = (uint8_t)(id - 1U);

    if (mm_s == 0) {
        s_target[idx] = 0;
        s_dir[idx]    = 0;
        pid_reset(&s_pid[idx]);
        motor_set_direction(id, 0U);        /* 方向脚清零 = 停 */
        motor_set_duty(id, 0U);
        s_duty[idx] = 0U;
        s_active = 1U;                      /* 仍然算"本模块在管这两个轮子" */
        return;
    }

    dir = (idx == 0U) ? MS_LEFT_FWD_DIR : MS_RIGHT_FWD_DIR;
    if (mm_s > 0) {
        s_dir[idx] = +1;
    } else {
        /* 后退: motor.c 里方向只有 1 / 2 两个值, 取"另一个"就是反方向 */
        dir = (uint8_t)((dir == 1U) ? 2U : 1U);
        s_dir[idx] = -1;
        mm_s = -mm_s;
    }

    if (mm_s > (int32_t)MS_MAX_MM_S) { mm_s = (int32_t)MS_MAX_MM_S; }

    s_target[idx] = mm_s;
    motor_set_direction(id, dir);
    /* 占空比不在这里给 —— 交给 motor_speed_update() 里的 PI。
     * 但要把模块激活, 否则 update() 不会去写电机。 */
    s_active = 1U;
}

void motor_speed_stop(uint8_t id)
{
    motor_speed_set(id, 0);
}

void motor_speed_disable(void)
{
    motor_speed_stop(1U);
    motor_speed_stop(2U);
    s_active = 0U;
}

/* ============================================================================
 *  周期部分: 测速 + 速度 PI
 * ==========================================================================*/
void motor_speed_update(void)
{
    uint32_t now, dt;
    uint8_t  i;

    if (s_active == 0U) { return; }     /* ★ 没激活就绝对不碰电机 */

    now = tick_get_ms();
    if (s_last_ms == 0U) { s_last_ms = now; return; }

    dt = now - s_last_ms;
    if (dt < MS_LOOP_MS) { return; }    /* 还没到采样时刻 */
    s_last_ms = now;
    if (dt > 200U) { dt = 200U; }       /* 防卡顿造成一个离谱的 dt */

    for (i = 0U; i < 2U; i++) {
        int32_t c;
        int32_t mm_s;
        int32_t out;

        /* 计数"取走并清零"。
         * ★ 必须关一下中断: 读 + 清是两条指令, 中间被 ISR 插进来会丢计数。
         *   关得很短(几十个时钟), 不影响按键扫描和 PWM(那是硬件在跑)。 */
        __disable_irq();
        c = s_count[i];
        s_count[i] = 0;
        __enable_irq();

        /* 计数 -> mm/s:
         *     mm/s = 计数 / 每圈计数 * 轮周长(mm) * 1000 / dt(ms)
         * 用 64 位中间量, 免得 计数*周长*1000 溢出。 */
        mm_s = (int32_t)(((int64_t)c * (int64_t)MS_WHEEL_C_MM * 1000)
                         / ((int64_t)MS_COUNTS_PER_REV * (int64_t)dt));

        /* ★ 符号: 编码器只数 A 相边沿, 数不出方向, 所以用【命令方向】定符号 */
        if (s_dir[i] < 0) { mm_s = -mm_s; }
        s_meas[i] = mm_s;

        /* 速度 PI: 目标 - 实测 -> 占空比。复用 system/pid.c 的 pid_compute()。
         * ★ 输出不许为负: 方向由方向脚管, 占空比只表示"多大劲"。 */
        out = pid_compute(&s_pid[i], s_target[i], mm_s, dt);
        if (out < 0) { out = 0; }
        if (out > (int32_t)MS_MAX_DUTY) { out = (int32_t)MS_MAX_DUTY; }
        s_duty[i] = (uint16_t)out;

        motor_set_duty((uint8_t)(i + 1U), s_duty[i]);
    }
}

/* ============================================================================
 *  调试接口
 * ==========================================================================*/
int32_t  motor_speed_get(uint8_t id)        { return ((id >= 1U) && (id <= 2U)) ? s_meas[id - 1U] : 0; }
int32_t  motor_speed_get_target(uint8_t id) { return ((id >= 1U) && (id <= 2U)) ? s_target[id - 1U] : 0; }
uint16_t motor_speed_get_duty(uint8_t id)   { return ((id >= 1U) && (id <= 2U)) ? s_duty[id - 1U] : 0U; }
uint8_t  motor_speed_is_active(void)        { return s_active; }
