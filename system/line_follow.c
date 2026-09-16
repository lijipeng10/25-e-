/* ============================================================================
 *  line_follow.c —— 循迹(只用灰度 + 编码器速度环, 【不接陀螺仪】)
 * ----------------------------------------------------------------------------
 *  控制律就两条, 整个文件一眼能看完:
 *
 *    1) 8 路灰度算重心偏差  error: -100(线在最左) ~ 0(正中) ~ +100(线在最右)
 *    2) 差速:  steer = LF_STEER_KP * error * LF_STEER_SIGN
 *             左轮命令 = LF_BASE_SPEED + steer
 *             右轮命令 = LF_BASE_SPEED - steer
 *       两个轮子真正转多快, 由 motor.c 的速度环(编码器 PID)去追。
 *
 *  丢线 -> 直接停车, 屏幕显示 LOST。不做"原地转向找线": 先把直线走稳,
 *          找线的花活等这一步过了再加。
 *
 *  ★ 上一版的双环(位置外环 + 航向内环) + 陀螺仪阻尼 + 原地转向状态机
 *    【已经删掉】—— 那版东西太多, 一坏就分不清是外环、内环、陀螺仪,
 *    还是原地转向在捣乱。老版本在 git 里: 1e456f5 之前的 system/line_follow.c。
 * ==========================================================================*/
#include "line_follow.h"
#include "grayscale_sensor.h"
#include "motor.h"

/* ---------- 可调参数: 全部在这里, 就 5 个 ---------- */

#define LF_LINE_LEVEL   1U      /* 灰度读到这个值算"压线"; 压线时反而是 0 就改成 0U */
#define LF_BASE_SPEED   300     /* 直行基础速度 mm/s */
#define LF_STEER_KP     3       /* 每 1 格误差给多少差速 mm/s; error 最大 ±100 */
#define LF_STEER_MAX    300     /* 差速上限 mm/s; 等于 BASE 时慢轮正好能降到 0 */
#define LF_STEER_SIGN   (+1)    /* ★★ 唯一的方向开关: 实测"越偏越远"就改成 -1 ★★ */

/* 单轮命令上限: 基础 + 满差速, 快轮最多这么快 */
#define LF_CMD_MAX      (LF_BASE_SPEED + LF_STEER_MAX)

/* ★ LF_LEFT_ID / LF_RIGHT_ID(哪个通道是物理左轮)定义在 line_follow.h 里 ——
 *   屏幕也要用它对上真实轮子, 只能有一份。 */

/* 8 路灰度的权重: 最左 -7 ... 最右 +7, 用来算线压在传感器哪一边 */
static const int8_t LF_WEIGHT[GRAYSCALE_SENSOR_CHANNELS] =
{
    -7, -5, -3, -1, +1, +3, +5, +7
};

/* ---------- 状态 ---------- */
static uint8_t  s_running;      /* 1 = 正在循迹 */
static uint8_t  s_lost;         /* 1 = 刚因为丢线停下来了 */
static uint8_t  s_bits;         /* 最近一次灰度位图, bit0 = 最左 */
static uint8_t  s_on_line;      /* 1 = 至少有一路压线 */
static int16_t  s_error;        /* 最近一次偏差 -100 ~ +100 */
static int16_t  s_steer;        /* 最近一次差速量 mm/s */
static int16_t  s_cmd_left;     /* 左轮命令速度 mm/s */
static int16_t  s_cmd_right;    /* 右轮命令速度 mm/s */
static int16_t  s_e_min;        /* 本次运行期间 error 的最小/最大值 */
static int16_t  s_e_max;

/* ---------- 内部函数 ---------- */

/* 读 8 路灰度, 返回位图(bit0 = 最左那一路)。
 * ★ Grayscale_Sensor_Read_All 内部有 ~400us 延时, 别调太快 */
static uint8_t lf_read_bits(void)
{
    uint16_t g[GRAYSCALE_SENSOR_CHANNELS];
    uint8_t  i;
    uint8_t  bits = 0U;

    Grayscale_Sensor_Read_All(g);

    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++)
    {
        if (g[i] == LF_LINE_LEVEL)
        {
            bits |= (uint8_t)(1U << i);     /* 第 i 路压线了 */
        }
    }

    return bits;
}

/* 由位图算偏差: -100(线在最左) ~ 0(正中间) ~ +100(线在最右);
 * 一路都没压线就是丢线, 通过 *on_line 返回 0 */
static int16_t lf_calc_error(uint8_t bits, uint8_t *on_line)
{
    int32_t sum = 0;                    /* 权重累加 */
    uint8_t cnt = 0U;                   /* 压线的路数 */
    uint8_t i;

    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++)
    {
        if ((bits & (uint8_t)(1U << i)) != 0U)
        {
            sum += (int32_t)LF_WEIGHT[i];
            cnt++;
        }
    }

    if (cnt == 0U)
    {
        *on_line = 0U;
        return 0;
    }

    *on_line = 1U;

    /* 先取权重平均(得到 -7~+7)再放大到 -100~+100; 写成 sum*100/(cnt*7) 是先乘后除,
     * 避免整数除法丢精度 */
    return (int16_t)((sum * 100) / ((int32_t)cnt * 7));
}

/* 把一个轮子的【命令速度】写下去: 方向脚 + 速度环目标 */
static void lf_set_wheel(uint8_t id, int32_t speed)
{
    /* ★ 编码器只数脉冲、认不出方向, 速度环拿到负目标只会把占空比压到 0。
     *   所以命令直接夹在 [0, LF_CMD_MAX]: 慢轮最多降到 0, 不倒转。 */
    if (speed < 0)
    {
        speed = 0;
    }

    if (speed > (int32_t)LF_CMD_MAX)
    {
        speed = (int32_t)LF_CMD_MAX;
    }

    motor_set_direction(id, (speed == 0) ? 0U : 1U);
    motor_pid_set(id, (float)speed);
}

/* ---------- 对外接口 ---------- */

void line_follow_init(void)
{
    s_running   = 0U;
    s_lost      = 0U;
    s_bits      = 0U;
    s_on_line   = 0U;
    s_error     = 0;
    s_steer     = 0;
    s_cmd_left  = 0;
    s_cmd_right = 0;
    s_e_min     = 0;
    s_e_max     = 0;
}

void line_follow_start(void)
{
    s_running = 1U;
    s_lost    = 0U;
    s_e_min   = 0;              /* 误差记录器清零, 只记这一次运行 */
    s_e_max   = 0;
}

void line_follow_stop(void)
{
    s_running = 0U;
    lf_set_wheel(LF_LEFT_ID, 0);
    lf_set_wheel(LF_RIGHT_ID, 0);
}

uint8_t line_follow_is_running(void)
{
    return s_running;
}

uint8_t line_follow_is_lost(void)
{
    return s_lost;
}

void line_follow_step(void)
{
    int32_t steer;

    /* ★ 第 1 步: 读灰度、算偏差 —— 不管跑不跑都要做。
     *   屏幕上的 E 和 G 全靠它; 停着不读就没法在不启动电机的情况下核对传感器。 */
    s_bits  = lf_read_bits();
    s_error = lf_calc_error(s_bits, &s_on_line);

    /* ★ 第 2 步: 算差速和两个轮子的命令 —— 也是不管跑不跑都算, 屏幕要显示。
     *   error > 0 = 线在右边 -> 要往右转 -> 左轮加速、右轮减速 */
    steer = (int32_t)LF_STEER_KP * (int32_t)s_error * LF_STEER_SIGN;

    if (steer > (int32_t)LF_STEER_MAX)
    {
        steer = (int32_t)LF_STEER_MAX;
    }

    if (steer < -(int32_t)LF_STEER_MAX)
    {
        steer = -(int32_t)LF_STEER_MAX;
    }

    s_steer     = (int16_t)steer;
    s_cmd_left  = (int16_t)(LF_BASE_SPEED + steer);
    s_cmd_right = (int16_t)(LF_BASE_SPEED - steer);

    /* 没在循迹: 上面算完够屏幕用了, 电机一下都不碰 */
    if (s_running == 0U)
    {
        return;
    }

    /* ★ 第 3 步: 丢线就停车 —— 不做原地转向找线, 先让它把直线走稳 */
    if (s_on_line == 0U)
    {
        line_follow_stop();
        s_lost = 1U;
        return;
    }

    if (s_error < s_e_min)
    {
        s_e_min = s_error;
    }

    if (s_error > s_e_max)
    {
        s_e_max = s_error;
    }

    lf_set_wheel(LF_LEFT_ID,  s_cmd_left);
    lf_set_wheel(LF_RIGHT_ID, s_cmd_right);
}

/* ---------- 给屏幕看的 ---------- */

int16_t line_follow_get_error(void)
{
    return s_error;
}

uint8_t line_follow_get_bits(void)
{
    return s_bits;
}

int16_t line_follow_get_steer(void)
{
    return s_steer;
}

int16_t line_follow_get_cmd_left(void)
{
    return s_cmd_left;
}

int16_t line_follow_get_cmd_right(void)
{
    return s_cmd_right;
}

void line_follow_get_error_range(int16_t *mn, int16_t *mx)
{
    if (mn != 0)
    {
        *mn = s_e_min;
    }

    if (mx != 0)
    {
        *mx = s_e_max;
    }
}
