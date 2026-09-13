#include "sm_motor.h"
#include "ti_msp_dl_config.h"
#include "delay.h"
#include <math.h>

/* 占空比上限约定: 0~100(百分比)。若用 0~1000(千分比), 改成 1000U 即可。 */
#define SM_DUTY_MAX   100U

/* ---------------------------------------------------------------------------
 * 速度线性(恒定加速度)的周期插值
 *   period = slow*fast / (fast + (slow-fast)*n/N)   —— 频率线性, 启停平滑
 * -------------------------------------------------------------------------*/
static uint32_t sm_motor_harmonic(uint32_t slow, uint32_t fast,
                                  uint32_t n, uint32_t N)
{
    uint64_t num;
    uint64_t den;

    if (N == 0U) return slow;
    if (slow <= fast) return fast;

    num = (uint64_t)slow * fast;
    den = fast + (((uint64_t)(slow - fast)) * n) / N;
    if (den == 0U) den = 1U;
    if (den > slow) den = slow;
    return (uint32_t)(num / den);
}

/* ---------------------------------------------------------------------------
 * 每轴独立的位置控制状态(主循环与两轴中断都会访问, 故 volatile)
 *   [0]=电机1(TIMA1/CC0), [1]=电机2(TIMG12/CC1)
 * -------------------------------------------------------------------------*/
typedef struct {
    volatile uint8_t  active;       /* =1 走位中 */
    volatile uint32_t remaining;    /* 剩余步数 */
    volatile uint32_t total;        /* 总步数 */
    volatile uint32_t ramp;         /* 加速/减速各占步数 */
    volatile uint32_t period_max;   /* 起步周期(最慢) */
    volatile uint32_t period_min;   /* 巡航周期(最快) */
} MoveAxis;

static volatile MoveAxis s_axis[2];
static uint8_t s_speed_running[2];   /* 连续速度控制: 该轴是否已在转动 */

/* ===========================================================================
 *  轴1: TIMA1 / CC0  (DL_TimerA_*)
 * ========================================================================*/
static void axis1_launch(uint32_t period)
{
    DL_TimerA_stopCounter(sm_motor_ST1_INST);
    DL_TimerA_clearInterruptStatus(sm_motor_ST1_INST, DL_TIMER_IIDX_ZERO);
    DL_TimerA_disableInterrupt(sm_motor_ST1_INST, DL_TIMER_IIDX_ZERO);
    DL_TimerA_setLoadValue(sm_motor_ST1_INST, period - 1U);
    DL_TimerA_setTimerCount(sm_motor_ST1_INST, period - 1U);
    DL_TimerA_setCaptureCompareValue(sm_motor_ST1_INST, period / 2U,
                                     GPIO_sm_motor_ST1_C0_IDX);   /* 50% */
    DL_TimerA_startCounter(sm_motor_ST1_INST);
}
static void axis1_apply(uint32_t period)
{
    DL_TimerA_setLoadValue(sm_motor_ST1_INST, period - 1U);
    DL_TimerA_setCaptureCompareValue(sm_motor_ST1_INST, period / 2U,
                                     GPIO_sm_motor_ST1_C0_IDX);
}
static void axis1_enable_zero(void)
{
    DL_TimerA_clearInterruptStatus(sm_motor_ST1_INST, DL_TIMER_IIDX_ZERO);
    DL_TimerA_enableInterrupt(sm_motor_ST1_INST, DL_TIMER_IIDX_ZERO);
    NVIC_ClearPendingIRQ(sm_motor_ST1_INST_INT_IRQN);
    NVIC_EnableIRQ(sm_motor_ST1_INST_INT_IRQN);
}
static void axis1_stop(void)
{
    DL_TimerA_stopCounter(sm_motor_ST1_INST);
    DL_TimerA_disableInterrupt(sm_motor_ST1_INST, DL_TIMER_IIDX_ZERO);
}

/* ===========================================================================
 *  轴2: TIMG12 / CC1  (DL_TimerG_*)
 * ========================================================================*/
static void axis2_launch(uint32_t period)
{
    DL_TimerG_stopCounter(sm_motor_ST2_INST);
    DL_TimerG_clearInterruptStatus(sm_motor_ST2_INST, DL_TIMER_IIDX_ZERO);
    DL_TimerG_disableInterrupt(sm_motor_ST2_INST, DL_TIMER_IIDX_ZERO);
    DL_TimerG_setLoadValue(sm_motor_ST2_INST, period - 1U);
    DL_TimerG_setTimerCount(sm_motor_ST2_INST, period - 1U);
    DL_TimerG_setCaptureCompareValue(sm_motor_ST2_INST, period / 2U,
                                     GPIO_sm_motor_ST2_C1_IDX);   /* 50% */
    DL_TimerG_startCounter(sm_motor_ST2_INST);
}
static void axis2_apply(uint32_t period)
{
    DL_TimerG_setLoadValue(sm_motor_ST2_INST, period - 1U);
    DL_TimerG_setCaptureCompareValue(sm_motor_ST2_INST, period / 2U,
                                     GPIO_sm_motor_ST2_C1_IDX);
}
static void axis2_enable_zero(void)
{
    DL_TimerG_clearInterruptStatus(sm_motor_ST2_INST, DL_TIMER_IIDX_ZERO);
    DL_TimerG_enableInterrupt(sm_motor_ST2_INST, DL_TIMER_IIDX_ZERO);
    NVIC_ClearPendingIRQ(sm_motor_ST2_INST_INT_IRQN);
    NVIC_EnableIRQ(sm_motor_ST2_INST_INT_IRQN);
}
static void axis2_stop(void)
{
    DL_TimerG_stopCounter(sm_motor_ST2_INST);
    DL_TimerG_disableInterrupt(sm_motor_ST2_INST, DL_TIMER_IIDX_ZERO);
}

/* ---------------------------------------------------------------------------
 * 初始化: 使能两轴(EN 高)
 * -------------------------------------------------------------------------*/
void sm_motor_Init(void)
{
    DL_GPIO_setPins(sm_motor_EN1_PORT, sm_motor_EN1_PIN);
    DL_GPIO_setPins(sm_motor_EN2_PORT, sm_motor_EN2_PIN);
}

/* 启动两轴计数器(不带 ZERO 中断) */
void sm_motor_Start(void)
{
    DL_TimerA_startCounter(sm_motor_ST1_INST);
    DL_TimerG_startCounter(sm_motor_ST2_INST);
}

/* ---------------------------------------------------------------------------
 * 设置某轴方向
 * -------------------------------------------------------------------------*/
void sm_motor_Set_Direction(uint16_t id, uint16_t direction)
{
    if (id == 1U)
    {
        if (direction == 1U) DL_GPIO_setPins(sm_motor_DIR1_PORT, sm_motor_DIR1_PIN);
        else                 DL_GPIO_clearPins(sm_motor_DIR1_PORT, sm_motor_DIR1_PIN);
    }
    else if (id == 2U)
    {
        if (direction == 1U) DL_GPIO_setPins(sm_motor_DIR2_PORT, sm_motor_DIR2_PIN);
        else                 DL_GPIO_clearPins(sm_motor_DIR2_PORT, sm_motor_DIR2_PIN);
    }
}

/* ---------------------------------------------------------------------------
 * 设置某轴 STEP PWM 占空比(步进调速不用, 留给其它)
 * -------------------------------------------------------------------------*/
void sm_motor_Set_Duty(uint16_t id, uint16_t Duty)
{
    uint32_t load;
    uint32_t ccr;

    if (Duty > SM_DUTY_MAX) Duty = (uint16_t)SM_DUTY_MAX;

    if (id == 1U)
    {
        load = DL_TimerA_getLoadValue(sm_motor_ST1_INST);
        ccr  = ((load + 1U) * Duty) / SM_DUTY_MAX;
        DL_TimerA_setCaptureCompareValue(sm_motor_ST1_INST, ccr,
                                         GPIO_sm_motor_ST1_C0_IDX);
    }
    else if (id == 2U)
    {
        load = DL_TimerG_getLoadValue(sm_motor_ST2_INST);
        ccr  = ((load + 1U) * Duty) / SM_DUTY_MAX;
        DL_TimerG_setCaptureCompareValue(sm_motor_ST2_INST, ccr,
                                         GPIO_sm_motor_ST2_C1_IDX);
    }
}

/* ---------------------------------------------------------------------------
 * 以 rpm 持续转动某轴
 * -------------------------------------------------------------------------*/
void sm_motor_Run(uint16_t id, uint16_t rpm)
{
    uint32_t clk;
    uint32_t period;
    uint64_t steps_per_min;
    uint32_t axis;   /* 0/1 */

    if (id == 1U)      { clk = sm_motor_ST1_INST_CLK_FREQ; axis = 0U; }
    else if (id == 2U) { clk = sm_motor_ST2_INST_CLK_FREQ; axis = 1U; }
    else return;

    if (rpm == 0U) { sm_motor_Stop(id); return; }

    steps_per_min = (uint64_t)MOTOR_STEPS_PER_REV * rpm;
    period = (uint32_t)((((uint64_t)clk * 60U) + (steps_per_min / 2U))
                        / steps_per_min);
    if (period < 2U || period > 65536U) return;

    s_axis[axis].active    = 0U;   /* 连续转: 不进行走位 */
    s_axis[axis].remaining = 0U;

    if (id == 1U) axis1_launch(period);
    else          axis2_launch(period);
}

/* ---------------------------------------------------------------------------
 * 平滑转到指定角度(带加减速, 到位自动停)
 * -------------------------------------------------------------------------*/
void sm_motor_MoveTo(uint16_t id, int32_t angle_deg, uint16_t rpm)
{
    uint32_t steps;
    int64_t  steps64;
    uint64_t cruise64;
    uint32_t cruise;
    uint32_t start_period;
    uint32_t ramp;
    uint32_t clk;
    uint32_t axis;

    if (id == 1U)      axis = 0U;
    else if (id == 2U) axis = 1U;
    else return;

    if (angle_deg == 0) { sm_motor_Stop(id); return; }

    steps64 = (((int64_t)((angle_deg < 0) ? -angle_deg : angle_deg)) *
               (int64_t)MOTOR_STEPS_PER_REV) / 360L;
    steps = (uint32_t)steps64;
    if (steps == 0U) steps = 1U;

    sm_motor_Set_Direction(id, (uint16_t)((angle_deg > 0) ? 1U : 0U));

    if (rpm == 0U) { sm_motor_Stop(id); return; }

    clk = (id == 1U) ? sm_motor_ST1_INST_CLK_FREQ : sm_motor_ST2_INST_CLK_FREQ;
    cruise64 = ((uint64_t)clk * 60U) /
               (((uint64_t)MOTOR_STEPS_PER_REV) * (uint64_t)rpm);
    cruise = (uint32_t) cruise64;
    if (cruise < 2U || cruise > 65536U) { sm_motor_Stop(id); return; }

    ramp = steps / 3U;
    if (ramp < 40U) ramp = (steps < 80U) ? (steps / 2U) : 40U;
    if (ramp > (steps / 2U)) ramp = steps / 2U;
    if (ramp == 0U) ramp = 1U;

    start_period = cruise * 8U;
    if (start_period > 65536U) start_period = 65536U;
    if (start_period < cruise + 2U) start_period = cruise + 2U;

    s_axis[axis].active     = 1U;
    s_axis[axis].total      = steps;
    s_axis[axis].remaining  = steps;
    s_axis[axis].ramp       = ramp;
    s_axis[axis].period_max = start_period;
    s_axis[axis].period_min = cruise;

    if (id == 1U) { axis1_launch(start_period); axis1_enable_zero(); }
    else          { axis2_launch(start_period); axis2_enable_zero(); }
}

/* ---------------------------------------------------------------------------
 * 立即停止某轴
 * -------------------------------------------------------------------------*/
void sm_motor_Stop(uint16_t id)
{
    if (id == 1U)
    {
        s_axis[0].active = 0U;
        s_axis[0].remaining = 0U;
        s_speed_running[0] = 0U;
        axis1_stop();
    }
    else if (id == 2U)
    {
        s_axis[1].active = 0U;
        s_axis[1].remaining = 0U;
        s_speed_running[1] = 0U;
        axis2_stop();
    }
}

/* 查询某轴是否正在走位(用于结合式: 等 MoveTo 完成后才做校正) */
uint8_t sm_motor_IsMoving(uint16_t id)
{
    if (id == 1U) return s_axis[0].active;
    if (id == 2U) return s_axis[1].active;
    return 0U;
}

/* ---------------------------------------------------------------------------
 * 中断: 轴1(TIMA1) 每周期计一步, 做加减速; 走完停
 * -------------------------------------------------------------------------*/
void sm_motor_ST1_INST_IRQHandler(void)     /* = TIMA1_IRQHandler */
{
    if (DL_TimerA_getPendingInterrupt(sm_motor_ST1_INST) == DL_TIMER_IIDX_ZERO)
    {
        MoveAxis *a = (MoveAxis *)&s_axis[0];
        uint32_t done;
        uint32_t period;

        if (a->active == 0U) { return; }
        if (a->remaining <= 1U)
        {
            a->active = 0U; a->remaining = 0U;
            axis1_stop();
            return;
        }
        a->remaining--;
        done = a->total - a->remaining;

        if (done < a->ramp) {
            period = sm_motor_harmonic(a->period_max, a->period_min, done, a->ramp);
        } else if (done >= (a->total - a->ramp)) {
            uint32_t n = done - (a->total - a->ramp);
            period = sm_motor_harmonic(a->period_max, a->period_min,
                                       a->ramp - n, a->ramp);
        } else {
            period = a->period_min;
        }
        axis1_apply(period);
    }
}

/* ---------------------------------------------------------------------------
 * 中断: 轴2(TIMG12) 每周期计一步, 做加减速; 走完停
 * -------------------------------------------------------------------------*/
void sm_motor_ST2_INST_IRQHandler(void)     /* = TIMG12_IRQHandler */
{
    if (DL_TimerG_getPendingInterrupt(sm_motor_ST2_INST) == DL_TIMER_IIDX_ZERO)
    {
        MoveAxis *a = (MoveAxis *)&s_axis[1];
        uint32_t done;
        uint32_t period;

        if (a->active == 0U) { return; }
        if (a->remaining <= 1U)
        {
            a->active = 0U; a->remaining = 0U;
            axis2_stop();
            return;
        }
        a->remaining--;
        done = a->total - a->remaining;

        if (done < a->ramp) {
            period = sm_motor_harmonic(a->period_max, a->period_min, done, a->ramp);
        } else if (done >= (a->total - a->ramp)) {
            uint32_t n = done - (a->total - a->ramp);
            period = sm_motor_harmonic(a->period_max, a->period_min,
                                       a->ramp - n, a->ramp);
        } else {
            period = a->period_min;
        }
        axis2_apply(period);
    }
}

/* ===========================================================================
 *  画圆: 两轴按 sin/cos 做连续速度跟随(一直转)
 *   Pan = r*cos(phase),  Tilt = r*sin(phase)
 *  radius_x10 : 半径, 0.1 度(如 150 = 半径 15.0°)
 *  period_ms  : 转一圈的毫秒数(如 3000)
 *  说明: 两轴按正弦/余弦速度连续转动, 相机便画出圆轨迹;
 *        在速度接近 0 的"换向点"因 16 位定时器有最小步率, 会略有顿挫。
 * ========================================================================*/
static void axis1_set_period(uint32_t period)
{
    DL_TimerA_setLoadValue(sm_motor_ST1_INST, period - 1U);
    DL_TimerA_setCaptureCompareValue(sm_motor_ST1_INST, period / 2U,
                                     GPIO_sm_motor_ST1_C0_IDX);
}
static void axis2_set_period(uint32_t period)
{
    DL_TimerG_setLoadValue(sm_motor_ST2_INST, period - 1U);
    DL_TimerG_setCaptureCompareValue(sm_motor_ST2_INST, period / 2U,
                                     GPIO_sm_motor_ST2_C1_IDX);
}

/* ---------------------------------------------------------------------------
 * 持续速度控制(闭环 PID 用): 只改方向 + 周期, 不反复停启(避免每拍抖一下)
 * rpm 带符号: >0 一个方向, <0 反方向, 0=停
 * -------------------------------------------------------------------------*/
void sm_motor_SetSpeed(uint16_t id, int32_t rpm)
{
    uint32_t axis, clk, period;
    uint32_t mag;

    if (id == 1U)      { axis = 0U; clk = sm_motor_ST1_INST_CLK_FREQ; }
    else if (id == 2U) { axis = 1U; clk = sm_motor_ST2_INST_CLK_FREQ; }
    else return;

    if (rpm == 0) { sm_motor_Stop(id); s_speed_running[axis] = 0U; return; }

    if (rpm > 0) sm_motor_Set_Direction(id, 1U);
    else         sm_motor_Set_Direction(id, 0U);

    mag = (rpm < 0) ? (uint32_t)(-(int32_t)rpm) : (uint32_t)rpm;
    period = (uint32_t)(((uint64_t)clk * 60U) /
                        ((uint64_t)MOTOR_STEPS_PER_REV * mag));
    if (period < 2U) period = 2U;
    /* 轴1(16位)上限 65536; 轴2(32位)放宽到 0xFFFFFF */
    if (axis == 0U) { if (period > 65536U) period = 65536U; }
    else            { if (period > 0x00FFFFFFU) period = 0x00FFFFFFU; }

    if (s_speed_running[axis] == 0U) {       /* 第一次启动该轴 */
        if (axis == 0U) axis1_launch(period);
        else            axis2_launch(period);
        s_speed_running[axis] = 1U;
    } else {                                  /* 已在转: 只更新周期 */
        if (axis == 0U) axis1_set_period(period);
        else            axis2_set_period(period);
    }
}

void sm_motor_CircleScan(int16_t radius_x10, uint16_t period_ms)
{
    const float PI2 = 6.28318530718f;
    const float tick_ms = 5.0f;                 /* 每 5ms 更新一次速度 */
    float omega = PI2 / (float)period_ms;       /* 角速度: 弧度/ms */
    float dphase = omega * tick_ms;             /* 每 tick 推进的相位 */
    float phase = 0.0f;
    float radius = (float)radius_x10 / 10.0f;   /* 半径: 度 */
    float spd = (float)MOTOR_STEPS_PER_REV / 360.0f;  /* 步/度 */

    /* 先启动两轴(给一个初始周期) */
    axis1_launch(10000U);
    axis2_launch(10000U);

    while (1)                                   /* 一直转, 不停 */
    {
        float v_pan, v_tilt, sps;
        uint32_t p1, p2;

        /* Pan = r*cos, 速度 v_pan = -r*omega*sin (度/ms) */
        v_pan  = -radius * omega * sinf(phase);
        /* Tilt = r*sin, 速度 v_tilt =  r*omega*cos (度/ms) */
        v_tilt =  radius * omega * cosf(phase);

        /* 速度 -> 每秒步数 = |速度| * (步/度) * 1000 */
        sps = fabsf(v_pan) * spd * 1000.0f;
        if (sps < 1.0f) sps = 1.0f;
        p1 = (uint32_t)((float)sm_motor_ST1_INST_CLK_FREQ / sps);
        if (p1 < 2U) p1 = 2U; else if (p1 > 65536U) p1 = 65536U;

        sps = fabsf(v_tilt) * spd * 1000.0f;
        if (sps < 1.0f) sps = 1.0f;
        p2 = (uint32_t)((float)sm_motor_ST2_INST_CLK_FREQ / sps);
        /* 轴2 是 32 位定时器(TIMG12), 允许更大的周期值以支持很慢的步率 */
        if (p2 < 2U) p2 = 2U; else if (p2 > 0x00FFFFFFU) p2 = 0x00FFFFFFU;

        /* 只改周期(速度), 不停止计数器 */
        axis1_set_period(p1);
        axis2_set_period(p2);

        /* 按速度符号定方向 */
        sm_motor_Set_Direction(1, (v_pan >= 0.0f) ? 1U : 0U);
        sm_motor_Set_Direction(2, (v_tilt >= 0.0f) ? 1U : 0U);

        phase += dphase;
        if (phase >= PI2) phase -= PI2;

        delay_ms((uint32_t)tick_ms);
    }
}
