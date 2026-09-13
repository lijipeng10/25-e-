/* ============================================================================
 *  line_follow.c —— 循迹(最基础版)
 * ----------------------------------------------------------------------------
 * 【循迹原理, 一句话】
 *      用 8 路灰度传感器看"黑线现在偏在哪边", 算出一个偏差值,
 *      再让左轮和右轮转得不一样快, 把车"掰"回线上。
 *
 * 【为什么差速能让车拐弯】
 *      左轮转得比右轮快  ->  车往右拐
 *      右轮转得比左轮快  ->  车往左拐
 *      所以"线偏左"就: 左轮减速、右轮加速。
 *
 * 【每一步在干什么】 (line_follow_step, 每 10ms 跑一次)
 *      第1步  读 8 路灰度     -> 拿到 8 位的"谁压线了"位图
 *      第2步  算偏差 error    -> -100(线在最左) ~ 0(正中) ~ +100(线在最右)
 *      第3步  PID 算转向量     -> 偏差越大, 转向量 steer 越大
 *      第4步  差速输出         -> 左轮 = 基础速度 + steer
 *                                右轮 = 基础速度 - steer
 *
 * 【怎么调参】先看下面"可调参数", 按注释改。
 *      车不动        -> 看 LF_BASE_DUTY 是不是太小
 *      越偏越远      -> 把 LF_STEER_SIGN 改成 -1
 *      来回画龙      -> 把 LF_KP 调小, 或 LF_MAX_STEER 调小
 *      拐不过来      -> 把 LF_KP 调大, 或 LF_MAX_STEER 调大
 *
 *  依赖: hardware/motor.c(电机) / hardware/grayscale_sensor.c(灰度)
 *        system/pid.c(PID) / system/tick.c(时基)
 * ==========================================================================*/
#include "line_follow.h"
#include "ti_msp_dl_config.h"

#include "motor.h"              /* motor_set_direction / motor_set_duty */
#include "grayscale_sensor.h"   /* Grayscale_Sensor_Read_All */
#include "pid.h"                /* 通用 PID */
#include "tick.h"               /* tick_get_ms */

/* ============================================================================
 *  可调参数 —— 要改就改这里
 * ==========================================================================*/

/* ---------- 速度 ----------
 * 【单位说明】现在这三个值就是【真正的占空比百分比】, 数值越大越快。
 *   之前 hardware/motor.c 里的换算写反了, 导致"数值越大越慢"。
 *   2025 已修 (见 motor.c 里 motor_duty_to_cmp 的说明), 现在语义正常了。
 *
 * 【本车是重车(带云台 + 相机)】所以速度一律取小:
 *   数值要"够得着"才走得动 —— 太低了电机转不起来, 车不走;
 *   但高一点点又冲得很快。所以先量出【启动死区】, 然后比它高一点点用。
 *
 * 【怎么量死区】把 LF_TEST_DUTY 依次改成 25 / 30 / 35 / 40, 按 KEY2:
 *   轮子开始能持续转动的那个值 ≈ 死区。取它 +5 左右当 LF_BASE_DUTY。 */
#define LF_BASE_DUTY        30      /* 直行基础速度。重车大约 30 左右起 */
#define LF_LOST_DUTY        25      /* 丢线时的速度(降速找线)。
                                       注意: 不能低于死区! 否则丢线时一侧轮子
                                       直接停住, 车会原地打转, 比不减速更糟。 */
#define LF_TEST_DUTY        40      /* 电机自检(KEY2)用的速度。要保证一定能转动,
                                       所以取比 LF_BASE_DUTY 高一点 */

/* ---------- 轮子映射 ---------- */
/* 电机编号: 1 = A路(PB17/PB18), 2 = B路(PB19/PB23) */
#define LF_LEFT_ID          1U      /* 左轮接在 A路 */
#define LF_RIGHT_ID         2U      /* 右轮接在 B路 */

/* "前进"对应的方向值。motor.c 里: 1 = 正转, 2 = 反转
 * 这两个值是在手动测试里标定出来的(按 KEY 让车往前跑的那个方向):
 *     A路前进 = 1, B路前进 = 2
 * 如果发现某个轮子反了, 改这里。 */
#define LF_LEFT_FWD_DIR     1U
#define LF_RIGHT_FWD_DIR    2U

/* ---------- 左右电机补偿 (trim) ----------
 * 两个电机不可能一模一样: 死区、齿轮箱阻力、轮胎摩擦都有差别。
 * 表现就是"给两个轮子同样的占空比, 车却往一边拐"。
 *
 *   LF_TRIM > 0  ->  右轮多出力、左轮少出力   => 车"往左偏"时用正数
 *   LF_TRIM < 0  ->  左轮多出力、右轮少出力   => 车"往右偏"时用负数
 *
 * 注意这里是一加一减: 左轮 -LF_TRIM, 右轮 +LF_TRIM。
 * 这样车的平均速度不变, 只改"左右谁出力多", 不会改变整体快慢。
 *
 * 【怎么调】
 *   1) 找一段长直道, 按 KEY2 (两轮同时 50%, 见 line_follow_test_wheels)。
 *      KEY2 时两个轮子本来是完全一样的指令, 所以车拐弯的唯一原因就是电机差异。
 *   2) 从 3 开始试:
 *          往左偏 -> LF_TRIM 调大 (3 -> 4 -> 5 ...)
 *          往右偏 -> LF_TRIM 调小 (3 -> 2 -> 1 -> 0 ...)
 *   3) 调到 KEY2 直行为止, 再放回循迹。
 *
 * 为什么用"加减固定值"而不是"乘一个系数":
 *   两个电机的差别主要来自死区不一样 —— 要让它们转速相同, 需要的其实是一个
 *   固定的占空比差, 和当前速度无关。所以固定加减在任何速度下都对。 */
#define LF_TRIM             0       /* ★ 用户要求先关掉补偿, 从 0 重新实测 */

/* ---------- 灰度传感器 ---------- */
/* 灰度读到哪个值算"压线"。用调试画面看: 车压黑线时对应位变 1 就对了;
 * 如果反了(压线时是 0), 把这个改成 0。 */
#define LF_LINE_LEVEL       1U

/* ---------- 转向极性(最关键的开关) ---------- */
/* +1 或 -1。判断方法: 车拿在手上按 KEY1, 把传感器在黑线上左右平移:
 *      线往左偏 -> 左轮变慢、右轮变快  = 正确(+1)
 *      线往左偏 -> 左轮反而变快        = 改成 -1
 *
 * ★★ 必须重新验证! ★★
 * 之前 hardware/motor.c 的占空比换算是反的, 那时候"谁快谁慢"和命令值是反的,
 * 所以之前验出来的极性结论【不可信】。现在换算修好了, 极性要重新测一遍。 */
#define LF_STEER_SIGN       (+1)

/* ---------- PID 参数 ----------
 *
 * 【实际算式】system/pid.c 里是:
 *      out = (KP*err + KD*(err-prev)*100/dt) / div
 * 代入 dt = LF_STEP_MS = 10ms、div = 100, 100 和 100 约掉, 就是:
 *
 *      steer = 0.40 * error  +  1.4 * LF_KD * (error 的变化量)
 *              └── 比例项 P ─┘    └──────── 微分项 D ────────┘
 *
 * 【为什么原来的 LF_KD = 12 不能要】
 * 8 路是"数字量"传感器, 算出来的 error 是【一档一档跳】的, 只可能是:
 *      0, ±14, ±28, ±43, ±57, ±71, ±85, ±100
 * (只有一路压线时 error = 权重*100/7, 权重是 1/3/5/7, 所以最小跳变 = 14)
 *
 * 于是 error 每跳一档(变化量 14):
 *      比例项 P = 0.40 * 14 = 5.6
 *      微分项 D = 1.4 * 12 * ... = 14 * 12 / 10 = 16.8   <-- 是 P 的 3 倍!
 *
 * 后果分三拍看(基础速度 40, steer 直接加在左右轮上):
 *      第1拍  error 0 -> +14 : steer = +5.6 + 16.8 = +22  (该轻修, 结果猛打)
 *      第2拍  error 还是 +14 : steer = +5.6 +  0   = +5   (又突然不管了)
 *      第3拍  error +14 -> 0 : steer =  0   - 16.8 = -16  (★ 回到中线却反着打方向!)
 * 第 3 拍是最要命的: 车已经回到线正中间了, 闭环反而把车往反方向推
 * (左轮 24% / 右轮 56%), 于是 回中线 -> 被反推出去 -> 再猛拉回来 -> 画龙。
 *
 * 【结论】
 * 对这种"一档一档跳"的误差, 微分项要么不用, 要么取很小 —— 它放大的
 * 其实是量化台阶, 不是真实趋势。让 D 不超过 P 的一半:
 *      1.4 * LF_KD <= 5.6 / 2   ->   LF_KD <= 2
 * 所以这里先给 0(纯比例)。★ 如果还是画龙, 调小 LF_KP, 不要回头去加 LF_KD。 */
#define LF_KP               40
#define LF_KI               0       /* 最基础版本先不用积分项 */
#define LF_KD               0       /* 见上面: 数字量误差上 D 只会帮倒忙 */

#define LF_MAX_STEER        30      /* 转向量上限。
                                       跟 LF_BASE_DUTY 一样大时, 极限情况下慢的
                                       一侧刚好降到 0(停住), 不会出现"反转"。
                                       ★ 改 LF_BASE_DUTY 时记得跟着一起改 */

/* ---------- 丢线保护 ---------- */
#define LF_LOST_TIMEOUT_MS  500U    /* 丢线超过这么久就停车(防止车跑飞) */

/* ---------- 控制周期 ---------- */
#define LF_STEP_MS          10U     /* line_follow_step 的调用周期(ms) */

/* ============================================================================
 *  内部状态
 * ==========================================================================*/

/* 8 路灰度传感器的权重: 最左 -7 ... 最右 +7
 * 用它算线的"重心"在哪儿: 重心偏负 = 线在左边; 偏正 = 线在右边。 */
static const int8_t LF_WEIGHT[GRAYSCALE_SENSOR_CHANNELS] = {
    -7, -5, -3, -1, +1, +3, +5, +7
};

static Pid     s_pid;               /* 转向 PID */
static uint8_t s_running;           /* 1 = 正在循迹 */
static uint8_t s_bits;              /* 最近一次灰度位图(1 = 压线, 已按 LF_LINE_LEVEL 判断) */
static uint16_t s_raw[GRAYSCALE_SENSOR_CHANNELS];  /* 最近一次灰度的原始值(0/1) */
static int16_t s_error;             /* 最近一次偏差 -100~+100 */
static uint8_t s_left_duty;         /* 最近一次左轮占空比(调试用) */
static uint8_t s_right_duty;        /* 最近一次右轮占空比(调试用) */
static int8_t  s_last_dir;          /* 丢线时用来记住"上次往哪边拐" */
static uint16_t s_lost_ms;          /* 已经连续丢线多久(ms) */

/* ============================================================================
 *  内部函数
 * ==========================================================================*/

/* ---------------------------------------------------------------------------
 *  读 8 路灰度, 返回位图: bit0 = 最左那路, 1 = 压线
 * -------------------------------------------------------------------------*/
static uint8_t lf_read_bits(void)
{
    uint16_t g[GRAYSCALE_SENSOR_CHANNELS];
    uint8_t  i;
    uint8_t  bits = 0U;

    Grayscale_Sensor_Read_All(g);       /* 这个函数内部有 ~400us 延时, 别调太快 */

    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        s_raw[i] = g[i];                    /* 原样存一份给调试显示用 */

        if (g[i] == LF_LINE_LEVEL) {
            bits |= (uint8_t)(1U << i);     /* 第 i 路压线了, 把第 i 位置 1 */
        }
    }
    return bits;
}

/* ---------------------------------------------------------------------------
 *  由位图算偏差: 返回 -100(线在最左) ~ 0(正中间) ~ +100(线在最右)
 *  一个通道都没压线(丢线)时, 通过 *on_line 返回 0
 * -------------------------------------------------------------------------*/
static int16_t lf_calc_error(uint8_t bits, uint8_t *on_line)
{
    int32_t sum = 0;                    /* 权重累加 */
    uint8_t cnt = 0U;                   /* 压线的通道数 */
    uint8_t i;

    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        if ((bits & (uint8_t)(1U << i)) != 0U) {
            sum += (int32_t)LF_WEIGHT[i];
            cnt++;
        }
    }

    if (cnt == 0U) {                    /* 丢线 */
        *on_line = 0U;
        return 0;
    }
    *on_line = 1U;

    /* 先把权重取平均(得到 -7~+7), 再放大到 -100~+100。
     * 写法 sum*100/(cnt*7) 是为了先乘后除, 避免整数除法丢精度。 */
    return (int16_t)((sum * 100) / ((int32_t)cnt * 7));
}

/* ---------------------------------------------------------------------------
 *  让一个轮子按"带符号的速度"转
 *      id      : 1 = A路, 2 = B路
 *      fwd_dir : 该轮"前进"对应的方向值(1 或 2)
 *      cmd     : 0~100 的占空比。<=0 就停下(最基础版本不允许倒转)
 * -------------------------------------------------------------------------*/
static void lf_set_wheel(uint8_t id, uint8_t fwd_dir, int32_t cmd)
{
    if (cmd <= 0) {
        motor_set_direction(id, 0U);            /* 方向脚清零 = 停 */
        motor_set_duty(id, 0U);
        return;
    }
    if (cmd > 100) { cmd = 100; }               /* 限幅, 别超过 100% */

    motor_set_direction(id, fwd_dir);           /* 前进方向 */
    motor_set_duty(id, (uint16_t)cmd);          /* 占空比 */
}

/* ---------------------------------------------------------------------------
 *  两个轮子一起停
 *  注意: 这里不用 motor_stop(), 因为 motor_stop() 内部是"两路都停",
 *        我们分开写更清楚, 也方便以后单独控制某一个轮子。
 * -------------------------------------------------------------------------*/
static void lf_stop_wheels(void)
{
    lf_set_wheel(LF_LEFT_ID,  LF_LEFT_FWD_DIR,  0);
    lf_set_wheel(LF_RIGHT_ID, LF_RIGHT_FWD_DIR, 0);
    s_left_duty  = 0U;
    s_right_duty = 0U;
}

/* ============================================================================
 *  对外接口
 * ==========================================================================*/

void line_follow_init(void)
{
    /* 初始化转向 PID:
     *   pid_init(&pid, kp, ki, kd, div, 积分限幅, 输出限幅, 死区, 最小dt)
     * 输出限幅就用 LF_MAX_STEER, 这样 PID 自己就会把转向量限制住。 */
    pid_init(&s_pid, LF_KP, LF_KI, LF_KD, 100,
             0, LF_MAX_STEER, 0, LF_STEP_MS);

    s_running    = 0U;
    s_bits       = 0U;
    s_error      = 0;
    s_left_duty  = 0U;
    s_right_duty = 0U;
    s_last_dir   = 0;
    s_lost_ms    = 0U;
}

void line_follow_start(void)
{
    pid_reset(&s_pid);          /* 清掉上次的积分/微分残留, 不然起步会猛地一拐 */
    s_lost_ms  = 0U;
    s_last_dir = 0;
    s_running  = 1U;
}

void line_follow_stop(void)
{
    s_running = 0U;
    lf_stop_wheels();           /* 立刻把两个轮子关掉 */
}

uint8_t line_follow_is_running(void)
{
    return s_running;
}

/* ---------------------------------------------------------------------------
 *  控制步进: 每 10ms 调一次
 * -------------------------------------------------------------------------*/
void line_follow_step(void)
{
    uint8_t  on_line;
    int16_t  error;
    int32_t  steer;
    int32_t  left_cmd, right_cmd;
    int32_t  base;

    /* ---------------- 第 1 步: 读灰度, 算偏差 ----------------
     * ★ 注意: 这一步无论"循迹跑不跑"都必须做!
     *   因为屏幕和串口显示的调试数据就是这里读出来的。
     *   如果停着就不读, 就没法在不启动电机的情况下标定传感器了
     *   (之前就是这里写错顺序, 导致没按 KEY1 时 S 一直是 00000000)。 */
    s_bits  = lf_read_bits();
    error   = lf_calc_error(s_bits, &on_line);
    s_error = error;

    /* 没在循迹: 数据照读照更新, 但不驱动电机 */
    if (s_running == 0U) {
        return;
    }

    /* ---------------- 第 2 步: 算转向量 steer ---------------- */
    if (on_line != 0U) {
        /* 看到线了: 正常循迹 */
        s_lost_ms = 0U;
        base      = LF_BASE_DUTY;

        /* 把"线的左右偏差"喂给 PID, 输出转向量。
         * 乘 LF_STEER_SIGN 是为了方便一键反方向。 */
        steer = pid_update(&s_pid, (int32_t)error * LF_STEER_SIGN, LF_STEP_MS);
    }
    else {
        /* 丢线了: 一个通道都没看到黑线 */
        s_lost_ms += LF_STEP_MS;

        if (s_lost_ms >= LF_LOST_TIMEOUT_MS) {
            /* 丢太久 -> 停车, 免得车乱跑。可能是车冲出去了, 或者传感器没调好 */
            line_follow_stop();
            return;
        }

        /* 短时间内: 记住上一次往哪边拐, 继续朝那边找线, 同时降低速度 */
        base  = LF_LOST_DUTY;
        steer = (int32_t)s_last_dir * LF_MAX_STEER;
    }

    /* ---------------------------------------------------------------
     *  小技巧: 修正量到底往哪边掰, 取决于"线偏的方向"和"轮子快慢"的对应关系。
     *  我们这里的约定是:
     *      error < 0 (线在左边)  ->  steer < 0  ->  左轮变慢、右轮变快  ->  车往左拐  ✓
     *      error > 0 (线在右边)  ->  steer > 0  ->  左轮变快、右轮变慢  ->  车往右拐  ✓
     *  如果实测反了(越偏越远), 把上面的 LF_STEER_SIGN 改成 -1 即可。
     * --------------------------------------------------------------- */

    /* 记住这次拐的方向, 丢线时要用 */
    if (steer > 0)      { s_last_dir = +1; }
    else if (steer < 0) { s_last_dir = -1; }

    /* ---------------- 第 3 步: 差速输出 ---------------- */
    left_cmd  = base + steer;       /* 线偏左时 steer<0 -> 左轮慢 */
    right_cmd = base - steer;       /*                    右轮快 */

    /* 再补上"两个电机本身不一样"这一步。见 LF_TRIM 的说明。
     * 一加一减, 平均速度不变, 只调左右出力的分配。 */
    left_cmd  -= LF_TRIM;
    right_cmd += LF_TRIM;

    /* 限幅: 两边都不许倒转, 慢的一侧最多降到 0。
     * (最基础版本先这么保守, 稳住不跑飞比较重要) */
    if (left_cmd  < 0) { left_cmd  = 0; }
    if (right_cmd < 0) { right_cmd = 0; }

    lf_set_wheel(LF_LEFT_ID,  LF_LEFT_FWD_DIR,  left_cmd);
    lf_set_wheel(LF_RIGHT_ID, LF_RIGHT_FWD_DIR, right_cmd);

    s_left_duty  = (uint8_t)left_cmd;
    s_right_duty = (uint8_t)right_cmd;
}

/* ============================================================================
 *  调试接口
 * ==========================================================================*/

void line_follow_get_raw(uint16_t *out)
{
    uint8_t i;
    if (out == 0) { return; }
    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        out[i] = s_raw[i];
    }
}

void line_follow_test_wheels(uint8_t on)
{
    int32_t l, r;

    if (on != 0U) {
        /* ★ 自检故意也带上 LF_TRIM:
         *   两个轮子本来发的是完全相同的指令, 所以车拐弯的唯一原因就是
         *   电机本身的差异。于是 KEY2 就成了调 LF_TRIM 最快的工具 ——
         *   看车直不直, 直接改 LF_TRIM, 不用反复进循迹模式试。 */
        l = (int32_t)LF_TEST_DUTY - LF_TRIM;
        r = (int32_t)LF_TEST_DUTY + LF_TRIM;
        if (l < 0) { l = 0; }
        if (r < 0) { r = 0; }

        lf_set_wheel(LF_LEFT_ID,  LF_LEFT_FWD_DIR,  l);
        lf_set_wheel(LF_RIGHT_ID, LF_RIGHT_FWD_DIR, r);

        s_left_duty  = (uint8_t)l;
        s_right_duty = (uint8_t)r;
    } else {
        lf_stop_wheels();
    }
}

int16_t line_follow_get_error(void)      { return s_error; }
uint8_t line_follow_get_bits(void)       { return s_bits; }
uint8_t line_follow_get_left_duty(void)  { return s_left_duty; }
uint8_t line_follow_get_right_duty(void) { return s_right_duty; }
