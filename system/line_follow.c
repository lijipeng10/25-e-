/* ============================================================================
 *  line_follow.c —— 循迹(灰度定位置 + 陀螺仪做阻尼 + 编码器速度环)
 * ----------------------------------------------------------------------------
 *  控制律就两条, 整个文件一眼能看完:
 *
 *    1) 8 路灰度算重心偏差  error: -100(线在最左) ~ 0(正中) ~ +100(线在最右)
 *    2) 差速 = 位置项 + 阻尼项:
 *         steer = LF_STEER_KP * error * LF_STEER_SIGN      <- 位置: 线偏了才修
 *               + LF_GYRO_KD  * 角速度                     <- 阻尼: 车头正在转就先拦
 *         左轮命令 = 基础速度 + steer,  右轮命令 = 基础速度 - steer,
 *         两个轮子真正转多快, 由 motor.c 的速度环(编码器 PID)去追。
 *
 *  ★★ 为什么必须有阻尼项(实测: 只有位置项会左右摆尾, 而且越摆越大):
 *     (1) 灰度是【位置】反馈 —— 线偏出去了才知道, 事前毫无信息;
 *     (2) 内侧速度环从"目标变了"到"轮子真到位"要 0.3~0.5 秒,
 *         220mm/s 下就是 66mm, 差不多一整个传感器阵列那么宽。
 *     两条加起来 = 修正永远迟到一步 -> 越修越过 -> 画龙。
 *     陀螺仪是唯一能【提前】说出"车头正在往哪边转、转多快"的东西, 而且没有延迟。
 *     ★ 这不是回到老版的双环: 老版是"位置环 + 航向环 + 阻尼 + 原地转向"四样叠一起;
 *       现在是一条位置式 + 一个阻尼项, 还是能一眼看完。
 *
 *  丢线 -> 直接停车, 屏幕显示 LOST。弯道(COAST/TURN)在另一个分支上, 这里没接。
 * ==========================================================================*/
#include "line_follow.h"
#include "grayscale_sensor.h"
#include "motor.h"
#include "mpu6050.h"

/* ---------- 可调参数: 全部在这里, 就 5 个 ---------- */

#define LF_LINE_LEVEL   1U      /* 灰度读到这个值算"压线"; 压线时反而是 0 就改成 0U */
#define LF_BASE_SPEED   440     /* 直行基础速度 mm/s(线偏出去时还会更低, 见 LF_SLOW_KP)。
                                 * ★ 调参史: 300 会摆尾 -> 220 稳住(|E|最大只有14) -> 440 提速。
                                 * ★ 提速后如果 L/R 追不上 l/r, 说明占空比饱和了, 降回来 */
#define LF_STEER_KP     3       /* 每 1 格误差给多少差速 mm/s; error 最大 ±100 */
#define LF_STEER_MAX    300     /* 差速上限 mm/s; 等于 BASE 时慢轮正好能降到 0 */
#define LF_SLOW_KP      2       /* ★ 转弯减速: |error| 每 1 格, 基础速度降多少 mm/s。
                                 *   基础速度翻倍了, 这里也翻倍, 保持原来的刹车力度 */
#define LF_GYRO_KD      2       /* ★★ 陀螺仪阻尼: 单位 mm/s 每 (度/秒)。0 = 关掉,
                                 *    摆得更凶就改成负数(说明陀螺仪左右符号反了) */
#define LF_BASE_MIN     120     /* ★ 基础速度下限 —— 弯道再慢也不能停(停了就转不动了) */
#define LF_STEER_SIGN   (+1)    /* ★★ 唯一的方向开关: 实测"越偏越远"就改成 -1 ★★ */
#define LF_STEP_MS      10U     /* line_follow_step() 的调用周期(ms) */
#define LF_LOST_MS      200U    /* ★ 连续看不到线多久才算【真】丢线(去抖) —— 见 step() 里的长注释 */

/* 单轮命令上限: 基础 + 满差速, 快轮最多这么快(基础速度已经被 LF_BASE_MIN 限住, 这里不会超) */
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
static int16_t  s_error;        /* 控制用的偏差 -100 ~ +100(看不到线时保持上一次的值) */
static uint16_t s_lost_ms;      /* 已经连续多少毫秒没看到线 */
static int16_t  s_steer;        /* 最近一次差速量 mm/s */
static int16_t  s_cmd_left;     /* 左轮命令速度 mm/s */
static int16_t  s_cmd_right;    /* 右轮命令速度 mm/s */
static int16_t  s_e_min;        /* 本次运行期间 error 的最小/最大值 */
static int16_t  s_e_max;
static uint16_t s_run_ms;       /* 本次运行已经跑了多少毫秒(丢线后停在最后那个值) */
static uint16_t s_e_max_abs;    /* 本次运行期间 |error| 到过的最大值 */

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
    /* ★ 现在的循迹只往前走, 所以命令夹在 [0, LF_CMD_MAX]: 慢轮最多降到 0。
     *   ★ 编码器【现在能测出方向了】(见 encoder.h 的 ENCODER_x_SIGN), 以后要倒转,
     *     把负数直接传给 motor_pid_set() 就行 —— 速度环会按符号自己设方向脚。
     *     (不能再靠 motor_set_direction() 手设: 速度环每 50ms 会用符号覆盖它) */
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
    s_error     = 0;
    s_lost_ms   = 0U;
    s_steer     = 0;
    s_cmd_left  = 0;
    s_cmd_right = 0;
    s_e_min     = 0;
    s_e_max     = 0;
    s_run_ms    = 0U;
    s_e_max_abs = 0U;
}

void line_follow_start(void)
{
    s_running = 1U;
    s_lost    = 0U;
    s_lost_ms = 0U;
    s_e_min     = 0;            /* 这几项清零, 只记这一次运行 */
    s_e_max     = 0;
    s_run_ms    = 0U;
    s_e_max_abs = 0U;
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
    int32_t steer = 0;
    int32_t base;
    int16_t e_abs = 0;
    int16_t raw_err;
    int32_t rate, damp;
    uint8_t seen;

    /* ★ 第 1 步: 读灰度、算偏差 —— 不管跑不跑都要做。
     *   屏幕上的 E 和 G 全靠它; 停着不读就没法在不启动电机的情况下核对传感器。 */
    s_bits  = lf_read_bits();
    raw_err = lf_calc_error(s_bits, &seen);

    /* ★★ 丢线去抖(实测踩过, 这是真凶): 电机一转, 灰度会偶尔【整组漏读一次】——
     *    车明明压在线上, 却读到 00000000。原来单次采样就判丢线, 结果刚起步就 LOST 停车。
     *    改法: 看到线就刷新 s_error; 没看到就【保持上一次的误差】(转向不变),
     *          并开始计时, 连续 LF_LOST_MS 都看不到才算真丢线。 */
    if (seen != 0U)
    {
        s_error   = raw_err;
        s_lost_ms = 0U;
    }
    else if (s_lost_ms < 0xFFFFU)
    {
        s_lost_ms += LF_STEP_MS;
    }

    e_abs = (s_error < 0) ? -s_error : s_error;

    /* ★ 第 2 步: 算差速和两个轮子的命令 —— 也是不管跑不跑都算, 屏幕要显示。
     *   error > 0 = 线在右边 -> 要往右转 -> 左轮加速、右轮减速 */

    /* ★★ 阻尼项: 车头正在往哪边转, 就先给一个【反向】的转向量把它拦住。
     *   符号推导(和 LF_STEER_SIGN 同源, 别乱改):
     *     陀螺仪【左转为正】(实测) -> 车正在往左偏 -> 要压它往右 -> steer 取正;
     *     而 steer > 0 = 左轮快 = 往右转 ✓ 所以这里是【加】, 不是减。
     *   停着的时候 damp 强制给 0 —— 屏幕上的 l/r 才是纯位置反馈的结果,
     *   方便不启动电机就核对转向方向。 */
    if (s_running != 0U)
    {
        mpu6050_update();                           /* 只有跑起来才读, 省那 1.5ms 的 I2C */
        rate = (int32_t)mpu6050_get_rate_x10();     /* 0.1 度/秒 */
        damp = ((int32_t)LF_GYRO_KD * rate) / 10;   /* -> mm/s */
    }
    else
    {
        damp = 0;
    }

    steer = (int32_t)LF_STEER_KP * (int32_t)s_error * LF_STEER_SIGN + damp;

    if (steer > (int32_t)LF_STEER_MAX)
    {
        steer = (int32_t)LF_STEER_MAX;
    }

    if (steer < -(int32_t)LF_STEER_MAX)
    {
        steer = -(int32_t)LF_STEER_MAX;
    }

    /* ★ 转弯减速: 线偏得越多, 基础速度压得越低。两个好处:
     *   (1) 弯道转得过来 —— 同样大小的差速, 速度越慢转弯半径越小;
     *   (2) 直道上开始摆的时候速度会自动降下来, 给修正留时间, 不容易越摆越大 */
    base = (int32_t)LF_BASE_SPEED - (int32_t)LF_SLOW_KP * (int32_t)e_abs;

    if (base < (int32_t)LF_BASE_MIN)
    {
        base = (int32_t)LF_BASE_MIN;
    }

    s_steer     = (int16_t)steer;
    s_cmd_left  = (int16_t)(base + steer);
    s_cmd_right = (int16_t)(base - steer);

    /* 没在循迹: 上面算完够屏幕用了, 电机一下都不碰 */
    if (s_running == 0U)
    {
        return;
    }

    /* ★ 第 3 步: 连续丢够 LF_LOST_MS 才停车。
     *   中间的几十毫秒里车还在按【最后一次看到的线】的方向走, 不会突然回正 */
    if (s_lost_ms >= LF_LOST_MS)
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

    /* ★ 诊断用: 已经跑了多久 + |error| 最大到过多少。
     *   "瞬间丢线" 和 "跑了一段才丢" 是两种完全不同的毛病, 靠这两个数分开 */
    if (s_run_ms < 0xFFFFU)
    {
        s_run_ms += LF_STEP_MS;
    }

    if (e_abs > (int16_t)s_e_max_abs)
    {
        s_e_max_abs = (uint16_t)e_abs;
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

/* 本次运行已经跑了多少毫秒; 丢线停车后停在最后那个值 */
uint16_t line_follow_get_run_ms(void)
{
    return s_run_ms;
}

/* 本次运行期间 |error| 到过的最大值(0~100) */
int16_t line_follow_get_error_max_abs(void)
{
    return (int16_t)s_e_max_abs;
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
