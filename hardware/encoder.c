/* encoder.c —— 轮速闭环(编码器测速 + 每轮速度PI) */
#include "encoder.h"
#include "ti_msp_dl_config.h"
#include "motor.h"
#include "pid.h"
#include "tick.h"

/* ============================ 参数(一行一个) ============================ */

#define MS_COUNTS_PER_REV   (260 * 2)   /* A相双沿: 13线x20减速比x2 = 520 计数/圈 */
#define MS_WHEEL_C_MM       150         /* 轮周长 mm(48mm 轮径)。★实测: 承重会压扁, 按地面推1米标 */
#define MS_KP               5           /* 速度PI的P: err=100mm/s 给 5 个占空比 */
#define MS_KI               4           /* 速度PI的I: 量程 4*60000/10000 = 24 个占空比(必须够到20, 否则顶不过死区) */
#define MS_I_CLAMP          60000       /* 积分限幅(防windup) */
#define MS_BASE_DUTY        8           /* 前馈基准: 一上来先给这么多占空比, 直接跳过死区(实测死区<=10) */
#define MS_MAX_DUTY         20          /* 占空比硬顶, 和以前开环的基础速度一致 -> 不可能跑飞 */
#define MS_MAX_MM_S         900         /* 目标速度上限(防误传) */
#define MS_LOOP_MS          20U         /* 速度环周期 ms, 太小低速档不够计数 */
#define MS_FAULT_MM_S       60          /* 目标超过它才做故障检查(太慢时读数本来就近0) */
#define MS_FAULT_MS         800U        /* 连续这么久"命令有速度但实测≈0" -> 判编码器坏 */

/* ★★ 前进方向 —— 已按"电机改到后轮"互换过 ★★
 * 上电验证: KEY2 给正速度, 车应该【前进】。反了就再把这两个换回来。 */
#define MS_LEFT_FWD_DIR     2U
#define MS_RIGHT_FWD_DIR    1U

/* ======================================================================= */

static volatile int32_t s_count[2];
static Pid      s_pid[2];
static int32_t  s_target[2];
static volatile int32_t s_meas[2];
static volatile int8_t  s_dir[2];
static uint16_t s_duty[2];
static uint32_t s_last_ms;
static uint8_t  s_active;
static uint16_t s_fault_ms[2];          /* "命令有速度但实测≈0"已经持续多久 */
static uint8_t  s_fault[2];             /* 1 = 编码器故障, 该轮已切断输出 */
static int32_t  s_raw[2];               /* 上一采样周期的原始脉冲数(诊断用) */

void motor_speed_isr(uint32_t sta_gpioa, uint32_t sta_gpiob)
{
    if ((sta_gpiob & encoder_E1A_PIN) != 0U) { s_count[0]++; }
    if ((sta_gpioa & encoder_E2A_PIN) != 0U) { s_count[1]++; }
}

void motor_speed_init(void)
{
    uint8_t i;
    for (i = 0U; i < 2U; i++) {
        s_count[i] = 0; s_target[i] = 0; s_meas[i] = 0; s_dir[i] = 0; s_duty[i] = 0U;
        s_fault_ms[i] = 0U; s_fault[i] = 0U;
        pid_init(&s_pid[i], MS_KP, MS_KI, 0, 100, MS_I_CLAMP, MS_MAX_DUTY, 0, MS_LOOP_MS);
    }
    s_last_ms = 0U;
    s_active  = 0U;
    /* 先清标志再开NVIC: syscfg 里云台编码器的引脚中断也使能着, 不清会立刻进一次中断 */
    DL_GPIO_clearInterruptStatus(GPIOA, 0xFFFFFFFFU);
    DL_GPIO_clearInterruptStatus(GPIOB, 0xFFFFFFFFU);
    NVIC_ClearPendingIRQ(ENC_GPIO_IRQN);
    NVIC_EnableIRQ(ENC_GPIO_IRQN);
}

void motor_speed_set(uint8_t id, int32_t mm_s)
{
    uint8_t idx, dir;

    if ((id < 1U) || (id > 2U)) { return; }
    idx = (uint8_t)(id - 1U);

    if (mm_s == 0) {
        s_target[idx] = 0; s_dir[idx] = 0; s_duty[idx] = 0U;
        s_fault_ms[idx] = 0U; s_fault[idx] = 0U;   /* 停了就清故障, 下次重试 */
        pid_reset(&s_pid[idx]);
        motor_set_direction(id, 0U);
        motor_set_duty(id, 0U);
        s_active = 1U;
        return;
    }

    dir = (idx == 0U) ? MS_LEFT_FWD_DIR : MS_RIGHT_FWD_DIR;
    if (mm_s > 0) {
        s_dir[idx] = +1;
    } else {
        dir = (uint8_t)((dir == 1U) ? 2U : 1U);   /* 反方向 */
        s_dir[idx] = -1;
        mm_s = -mm_s;
    }
    if (mm_s > (int32_t)MS_MAX_MM_S) { mm_s = (int32_t)MS_MAX_MM_S; }

    s_target[idx] = mm_s;
    motor_set_direction(id, dir);
    s_active = 1U;
}

void motor_speed_stop(uint8_t id) { motor_speed_set(id, 0); }

void motor_speed_disable(void)
{
    motor_speed_stop(1U);
    motor_speed_stop(2U);
    s_active = 0U;
}

void motor_speed_update(void)
{
    uint32_t now, dt;
    uint8_t  i;

    if (s_active == 0U) { return; }     /* 没激活就绝不碰电机 */

    now = tick_get_ms();
    if (s_last_ms == 0U) { s_last_ms = now; return; }
    dt = now - s_last_ms;
    if (dt < MS_LOOP_MS) { return; }
    s_last_ms = now;
    if (dt > 200U) { dt = 200U; }

    for (i = 0U; i < 2U; i++) {
        int32_t c, mm_s, out;

        __disable_irq();                /* 读+清要原子, 否则丢计数 */
        c = s_count[i];
        s_count[i] = 0;
        __enable_irq();
        s_raw[i] = c;                   /* ★ 存下来给屏幕看(诊断) */

        /* counts -> mm/s:  计数/每圈计数 * 周长 * 1000 / dt */
        mm_s = (int32_t)(((int64_t)c * (int64_t)MS_WHEEL_C_MM * 1000)
                         / ((int64_t)MS_COUNTS_PER_REV * (int64_t)dt));
        if (s_dir[i] < 0) { mm_s = -mm_s; }   /* 符号来自命令方向 */
        s_meas[i] = mm_s;

        /* ★★ 编码器故障保护 ★★
         * 命令有速度、实测却一直是 0 -> 编码器根本没在计数(线没接好/中断没来)。
         * 这时 PI 会把占空比一路顶到 20, 表现就是"一给速度就全速冲出去"。
         * 所以: 切断该轮输出 + 置故障标志, 让人一眼看出是编码器的问题。 */
        if ((s_target[i] > MS_FAULT_MM_S) && ((mm_s > -20) && (mm_s < 20))) {
            if (s_fault_ms[i] < 60000U) { s_fault_ms[i] += (uint16_t)dt; }
        } else {
            s_fault_ms[i] = 0U;
        }
        if (s_fault_ms[i] >= MS_FAULT_MS) { s_fault[i] = 1U; }

        if (s_fault[i] != 0U) {
            s_duty[i] = 0U;
            motor_set_duty((uint8_t)(i + 1U), 0U);
            continue;
        }

        /* ★ 前馈基准 + PI: 前馈直接跳过死区, PI 只补差值 —— 比纯积分快得多 */
        out = (int32_t)MS_BASE_DUTY + pid_compute(&s_pid[i], s_target[i], mm_s, dt);
        if (out < 0) { out = 0; }
        if (out > (int32_t)MS_MAX_DUTY) { out = (int32_t)MS_MAX_DUTY; }
        s_duty[i] = (uint16_t)out;

        motor_set_duty((uint8_t)(i + 1U), s_duty[i]);
    }
}

int32_t  motor_speed_get(uint8_t id)        { return ((id >= 1U) && (id <= 2U)) ? s_meas[id - 1U] : 0; }
int32_t  motor_speed_get_target(uint8_t id) { return ((id >= 1U) && (id <= 2U)) ? s_target[id - 1U] : 0; }
uint16_t motor_speed_get_duty(uint8_t id)   { return ((id >= 1U) && (id <= 2U)) ? s_duty[id - 1U] : 0U; }
uint8_t  motor_speed_is_active(void)        { return s_active; }
uint8_t  motor_speed_get_fault(uint8_t id)  { return ((id >= 1U) && (id <= 2U)) ? s_fault[id - 1U] : 0U; }
int32_t  motor_speed_get_raw(uint8_t id)    { return ((id >= 1U) && (id <= 2U)) ? s_raw[id - 1U] : 0; }
