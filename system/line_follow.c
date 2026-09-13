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
 * 【单位说明】下面这些数就是【真正的占空比百分比】, 数值越大越快。
 *   之前 hardware/motor.c 里的换算写反了, 导致"数值越大越慢"。
 *   已修 (见 motor.c 里 motor_duty_to_cmp 的说明), 现在语义正常了。
 *
 * 【本车是重车(带云台 + 相机)】所以整体限速。
 *   太低了电机转不起来车不走; 高一点点又冲得很快。
 *   先量出【启动死区】, 再取比它高一点点的值。
 *
 * 【怎么量死区】把 LF_TEST_DUTY 依次改成 10 / 15 / 20, 按 KEY2 看轮子:
 *   能持续转起来的那个值 ≈ 死区。取它 +2~3 当 LF_BASE_DUTY。 */

/* ★ 最高速度(硬顶): 任何一轮的占空比都不许超过这个值。
 *   不管 LF_BASE_DUTY / LF_MAX_STEER 怎么配, lf_set_wheel() 最后都会把
 *   结果卡在这里, 所以车不可能跑得比它更快。
 *   下面还有一条编译期检查, 配置超了会直接编译报错。 */
#define LF_MAX_DUTY         20

/* ★★ 基础速度: 必须【明显高于】电机启动死区, 否则会左右摆 ★★
 *
 * 踩过的坑: 把 LF_BASE_DUTY 调到 14 想让车慢一点, 结果左右摇摆反而更严重。
 * 原因 —— 基础速度离死区太近时:
 *      1) 两个轮子本身都在死区边上, 转速不稳
 *      2) 一个很小的转向命令(比如 steer = ±2)就会把某一侧推到死区【以下】
 *         -> 那个轮子【直接不转】-> 车猛地往一边窜
 *      3) 窜过头之后误差反号, 于是另一侧不转 -> 再窜回来
 *      结果就是来回摆。速度越低越严重, 这跟直觉是反的。
 *
 * 所以要留出余量: 下面的差速摆动(±几)不能让任何一侧掉到死区以下。
 * 本车(带云台+相机)实测大约 16 以上才稳, 所以取 18。 */
#define LF_BASE_DUTY        18      /* 直行基础速度 */
#define LF_LOST_DUTY        16      /* 丢线时的速度(降速找线)。
                                       同样不能低于死区! 否则丢线时一侧轮子
                                       直接停住, 车会原地打转, 比不减速更糟。 */
#define LF_TEST_DUTY        20      /* 电机自检(KEY2)用的速度 */

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
/* ★ LF_KP 必须和 LF_MAX_STEER 配套改!
 *   输出公式是 steer = 0.40*... 不, 是 steer = LF_KP * error / 100。
 *   error 的最小跳变是 14(见上), 所以:
 *       LF_KP = 40 时, 误差刚跳一档 -> steer = 40*14/100 = 5.6
 *   以前 LF_MAX_STEER = 30, 5.6 只用了 1/5 的量程, 还行;
 *   现在限速后 LF_MAX_STEER 只有 6, 5.6 直接就顶到上限了 ——
 *   变成"要么不转、要么打死"的开关式控制, 车会在线上左右打摆。
 *   所以这里同步降到 20:
 *       误差 14 -> steer 2      误差 28 -> steer 5
 *       误差 43 -> steer 8, 被限幅到 6 (从此饱和)
 *   即前两档还有渐变, 大偏差才打死。 */
#define LF_KP               20
#define LF_KI               0       /* 最基础版本先不用积分项 */
#define LF_KD               0       /* 见上面: 数字量误差上 D 只会帮倒忙 */

/* ★ 死区: |error| 小于它就当作"已经在线中间了", 输出 0, 不做任何修正。
 *
 * 为什么必须有它 —— 数字量传感器的误差是【一档 14】跳变的:
 *   线正好压在中间两路 -> error = 0   -> steer 0
 *   线偏过去压住一路   -> error = ±14 -> steer ±2
 * 问题在于【车没法停在 ±14 上】: 它一动就跳到另一侧, 于是 steer 在
 * +2 / 0 / -2 之间来回跳, 车就在线中间左右摆 —— 这就是"摇摆"的来源,
 * 跟电机死区无关, 纯粹是量化台阶造成的。
 *
 * 取 15: 正好把 |error| = 14 这一档(只差一路)压掉, 只在线偏到 2 路以上时才修。
 * 代价是车会在线中间 ±1 路以内自由浮动, 换来的是不摆。
 * 如果觉得"偏了一路也不修"太松, 往下调到 10(但摆动会回来一点)。 */
#define LF_DEADBAND         15

/* ★★ 转向量上限 —— 这个值直接决定"转得过弯还是冲过弯" ★★
 *
 * 差速输出是:
 *      慢的一侧 = LF_BASE_DUTY - steer
 *      快的一侧 = LF_BASE_DUTY + steer
 *
 * 【转弯力度 = 快的一侧 - 慢的一侧】, 也就是看这个差有多大。
 * 而要让这个差达到最大(= LF_MAX_DUTY, 也就是 20), 必须让【慢的一侧降到 0】。
 * 于是得到唯一正确的约束:
 *
 *      LF_MAX_STEER >= LF_BASE_DUTY
 *
 * 代入数字看(基础速度 18):
 *      LF_MAX_STEER =  6 -> 慢轮 12, 快轮 20 -> 速度差 8    <- 转不过弯
 *      LF_MAX_STEER = 18 -> 慢轮 0,  快轮 20 -> 速度差 20   <- 正确
 *
 * 为什么之前会冲过弯道: 慢的一侧最低只能降到 LF_BASE_DUTY - LF_MAX_STEER,
 * 只要它降不到 0, 速度差就上不去, 车就转不过来。
 *
 * 注意 LF_MAX_STEER 不需要大于 LF_BASE_DUTY —— 超出的部分会被 lf_set_wheel()
 * 的硬顶截掉, 白给。取相等正好。 */
#define LF_MAX_STEER        18

/* 编译期检查: 参数配错了直接编译报错, 不要等到跑车才发现。 */
#if (LF_BASE_DUTY > LF_MAX_DUTY)
#error "LF_BASE_DUTY 超过了 LF_MAX_DUTY(最高速度硬顶)"
#endif
#if (LF_LOST_DUTY > LF_MAX_DUTY)
#error "LF_LOST_DUTY 超过了 LF_MAX_DUTY(最高速度硬顶)"
#endif
#if (LF_MAX_STEER < LF_BASE_DUTY)
#error "LF_MAX_STEER 必须 >= LF_BASE_DUTY: 否则慢的一侧降不到 0, 速度差上不去, 会冲过弯道"
#endif

/* ============================================================================
 *  弯道: 停车原地转向再前进
 * ----------------------------------------------------------------------------
 *  思路: 车又重又慢, 用"差速硬拐"过急弯本来就吃力(会冲出去)。
 *        不如干脆一点 —— 到弯节点【停住】, 原地把车头【转正】, 再继续前进。
 *
 *  靠什么判断"到弯节点了": 线丢了。8 路一路都看不到黑线, 说明车已经冲过了
 *  线的尽头/拐角, 这就是节点。
 *
 *  靠什么判断"转到位了": 转向过程中传感器又会扫到那条新线, 当它出现在
 *  【中间附近】(|error| <= LF_PIVOT_OK) 时, 说明车头已经对准新方向, 停转。
 *  这样不需要陀螺仪/编码器, 只用灰度就能闭环。
 * ==========================================================================*/

#define LF_PIVOT_TRIGGER_MS 80U     /* 连续丢线超过这么久 -> 判定到弯节点, 停车转向。
                                       不能太小, 否则线上一个小缺口就会误触发;
                                       不能太大, 否则会冲过节点太远。 */
#define LF_PIVOT_DUTY       16      /* 原地转向时两个轮子的占空比(一正一反)。
                                       要能克服静摩擦把车拧动。 */
#define LF_PIVOT_OK         20      /* |error| <= 这个值就算"对准了", 结束转向。
                                       相当于线落在中间 1~2 路以内。 */
#define LF_PIVOT_TRY_MS     900U    /* ★ 一个方向最多找这么久。到了还没找到线,
                                       就【掉头往反方向找】—— 这是"估不准
                                       往哪边转"的兜底: 两个方向都扫一遍,
                                       总有一遍能扫到线, 不会一直原地转圈。
                                       两个方向合计 2*900 = 1.8 秒封顶。 */

/* 弯道参数的编译期检查。★ 注意必须放在这些宏【定义之后】——
 * 放在前面的话宏还没定义, 会被当成 0, 检查就没意义了(这里踩过一次)。 */
#if (LF_PIVOT_TRIGGER_MS >= LF_PIVOT_TRY_MS)
#error "LF_PIVOT_TRIGGER_MS 必须小于 LF_PIVOT_TRY_MS"
#endif
#if (LF_PIVOT_DUTY > LF_MAX_DUTY)
#error "LF_PIVOT_DUTY 超过了 LF_MAX_DUTY(最高速度硬顶)"
#endif

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
static int8_t  s_last_dir;          /* 瞬时"上次往哪边拐"(只在丢线那几十毫秒用) */
static uint16_t s_lost_ms;          /* 已经连续丢线多久(ms) */

/* ★★ 方向证据 —— 决定弯道往哪边转 ★★
 *
 * 踩过的坑: 原来直接用 s_last_dir 定转向方向, 结果到弯道"算不出往哪边转"。
 * 原因: s_last_dir 是【瞬时值】, 而直线上误差本来就在 ±14 之间抖, 每次小修正
 *       都会把它在 +1 / -1 之间来回翻。到丢线那一刻, 它记下的只是最后一次
 *       无意义的抖动, 跟弯的方向完全无关 —— 等于在猜。
 *
 * 改法: 累积"方向证据"。只在看到线的时候更新, 而且用泄漏积分:
 *          s_dir_ev = s_dir_ev * 7/8 + error
 *   效果: 直线上那种 ±14 的正负抖动会互相抵消(证据一直贴着 0 附近);
 *         而弯道口那条【横扫过来的横向线】会连续好几拍给出同号的大偏差,
 *         证据就会稳稳地积累到一边。这样判断的才是"趋势", 不是"噪声"。
 * 限幅是为了让方向证据能及时被新情况翻转, 不至于被历史拖住。 */
static int32_t  s_dir_ev;           /* 方向证据: 正 = 线一直在偏右, 负 = 偏左 */

static uint8_t  s_pivoting;         /* 1 = 正在原地转向(弯道模式) */
static int8_t   s_pivot_dir;        /* 本次原地转向往哪边(开始时定下, 中途不变) */
static uint8_t  s_pivot_try;        /* 0 = 第一遍(按证据方向), 1 = 第二遍(反方向) */
static uint16_t s_pivot_ms;         /* 当前这一遍已经转了多久(ms) */

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
 *  让一个轮子转
 *      id      : 1 = A路, 2 = B路
 *      fwd_dir : 该轮"前进"对应的方向值(1 或 2)
 *      cmd     : 带符号的占空比
 *                  > 0  前进, 大小 = 占空比
 *                  < 0  后退(只有"原地转向"会用到)
 *                  = 0  停
 *
 *  ★ 注意: 负数是【反转】, 不是停! 所以循迹的差速输出在传进来之前
 *    必须先自己夹到 0 以上(见 line_follow_step 里的限幅), 否则大偏差时
 *    慢的那一侧会突然倒转, 车会甩出去。
 * -------------------------------------------------------------------------*/
static void lf_set_wheel(uint8_t id, uint8_t fwd_dir, int32_t cmd)
{
    uint8_t dir;

    if (cmd == 0) {
        motor_set_direction(id, 0U);            /* 方向脚清零 = 停 */
        motor_set_duty(id, 0U);
        return;
    }

    if (cmd > 0) {
        dir = fwd_dir;                          /* 前进 */
    } else {
        /* 后退。motor.c 里方向只有 1 / 2 两个值, 取"另一个"就是反方向。
         * (A路: 1 = 正转, 2 = 反转; B路: 2 = 正转, 1 = 反转)
         * 如果以后给某一路加了别的方向定义, 这里要跟着改。 */
        dir = (uint8_t)((fwd_dir == 1U) ? 2U : 1U);
        cmd = -cmd;
    }

    /* ★ 最高速度的硬顶。所有往电机发的值都从这里过, 所以改了这里,
     *   不管上面怎么算都不可能超速 —— 这是最后一道保险。 */
    if (cmd > LF_MAX_DUTY) { cmd = LF_MAX_DUTY; }

    motor_set_direction(id, dir);
    motor_set_duty(id, (uint16_t)cmd);
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

/* ---------------------------------------------------------------------------
 *  原地转向: 两个轮子一正一反, 车绕自己中心转
 *      dir > 0 : 往右转(左轮前进, 右轮后退)
 *      dir < 0 : 往左转
 * -------------------------------------------------------------------------*/
static void lf_pivot(int8_t dir)
{
    int32_t d = (int32_t)LF_PIVOT_DUTY;

    if (dir > 0) {
        lf_set_wheel(LF_LEFT_ID,  LF_LEFT_FWD_DIR,   d);    /* 左轮前进 */
        lf_set_wheel(LF_RIGHT_ID, LF_RIGHT_FWD_DIR, -d);    /* 右轮后退 */
    } else {
        lf_set_wheel(LF_LEFT_ID,  LF_LEFT_FWD_DIR,  -d);    /* 左轮后退 */
        lf_set_wheel(LF_RIGHT_ID, LF_RIGHT_FWD_DIR,  d);    /* 右轮前进 */
    }

    s_left_duty  = (uint8_t)d;
    s_right_duty = (uint8_t)d;
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
             0, LF_MAX_STEER, LF_DEADBAND, LF_STEP_MS);   /* 死区见 LF_DEADBAND */

    s_running    = 0U;
    s_bits       = 0U;
    s_error      = 0;
    s_left_duty  = 0U;
    s_right_duty = 0U;
    s_last_dir   = 0;
    s_lost_ms    = 0U;
    s_dir_ev     = 0;
    s_pivoting   = 0U;
    s_pivot_dir  = +1;
    s_pivot_try  = 0U;
    s_pivot_ms   = 0U;
}

void line_follow_start(void)
{
    pid_reset(&s_pid);          /* 清掉上次的积分/微分残留, 不然起步会猛地一拐 */
    s_lost_ms   = 0U;
    s_last_dir  = 0;
    s_dir_ev    = 0;        /* 方向证据也清空, 从头攒 */
    s_pivoting  = 0U;
    s_pivot_dir = +1;
    s_pivot_try = 0U;
    s_pivot_ms  = 0U;
    s_running   = 1U;
}

void line_follow_stop(void)
{
    s_running  = 0U;
    s_pivoting = 0U;            /* 顺手退出弯道转向状态 */
    lf_stop_wheels();           /* 立刻把两个轮子关掉 */
}

uint8_t line_follow_is_running(void)
{
    return s_running;
}

uint8_t line_follow_is_pivoting(void)
{
    return s_pivoting;
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

    /* ================================================================
     *  状态 A: 正在原地转向(弯道模式)
     * ============================================================== */
    if (s_pivoting != 0U)
    {
        s_pivot_ms += LF_STEP_MS;

        /* 线重新出现在【中间附近】= 车头已经对准新方向 -> 停转, 回去循迹。
         * 为什么要求"在中间": 转的过程中线会从一边扫到另一边, 如果一看到线
         * 就停, 车头还是歪的。等它扫到中间, 才说明车头正对着新方向。 */
        if ((on_line != 0U) && (error <= LF_PIVOT_OK) && (error >= -LF_PIVOT_OK)) {
            pid_reset(&s_pid);      /* 清掉转向过程中残留的量, 免得接着猛拐一下 */
            s_lost_ms  = 0U;
            s_pivoting = 0U;
            s_dir_ev   = 0;         /* 这一弯的证据用完了, 清空, 下一弯重新攒 */
            /* 这里【不 return】: 直接落下去按正常循迹走一拍, 衔接更顺 */
        }
        else {
            /* ★ 兜底: 一个方向找够了还没扫到线, 就【掉头往反方向找】。
             *   这样不管方向证据猜得对不对, 两个方向都扫一遍总能扫到,
             *   车就不会一直在那儿原地转圈了。 */
            if (s_pivot_ms >= LF_PIVOT_TRY_MS) {
                if (s_pivot_try == 0U) {
                    s_pivot_try = 1U;
                    s_pivot_ms  = 0U;
                    s_pivot_dir = (int8_t)(-s_pivot_dir);   /* 掉头 */
                } else {
                    /* 两边都扫遍了还是没有线 -> 车多半已经掉出赛道, 停车 */
                    line_follow_stop();
                    return;
                }
            }

            lf_pivot(s_pivot_dir);  /* 还没对准, 继续原地转 */
            return;
        }
    }

    /* ================================================================
     *  状态 B: 正常循迹
     * ============================================================== */

    /* ---------------- 第 2 步: 算转向量 steer ---------------- */
    if (on_line != 0U) {
        /* 看到线了: 正常循迹 */
        s_lost_ms = 0U;
        base      = LF_BASE_DUTY;

        /* 累积"往哪边偏"的方向证据(见 s_dir_ev 的说明)。
         * 只在这里更新 —— 丢线时不更新, 这样弯道口攒下来的证据能保住,
         * 供原地转向决定方向用。 */
        s_dir_ev = (s_dir_ev * 7) / 8 + (int32_t)error;
        if (s_dir_ev >  400) { s_dir_ev =  400; }
        if (s_dir_ev < -400) { s_dir_ev = -400; }

        /* 把"线的左右偏差"喂给 PID, 输出转向量。
         * 乘 LF_STEER_SIGN 是为了方便一键反方向。 */
        steer = pid_update(&s_pid, (int32_t)error * LF_STEER_SIGN, LF_STEP_MS);
    }
    else {
        /* 丢线了: 一个通道都没看到黑线 */
        s_lost_ms += LF_STEP_MS;

        if (s_lost_ms >= LF_PIVOT_TRIGGER_MS) {
            /* ★ 连续丢线够久了 -> 判定"到弯节点了":
             *   先【停住】(别冲过节点), 再转到状态 A 去把车头拧正。
             *
             * 往哪边转: 看【方向证据】(见 s_dir_ev 的说明), 不看瞬时值。
             * 证据不够就退回最后一次拐弯方向, 再不够就默认往右。
             * 反正状态 A 里找不到会自动掉头扫另一边, 猜错也不会卡死。 */
            if (s_dir_ev > 0)      { s_pivot_dir = +1; }
            else if (s_dir_ev < 0) { s_pivot_dir = -1; }
            else                   { s_pivot_dir = (s_last_dir != 0) ? s_last_dir : +1; }

            s_pivoting  = 1U;
            s_pivot_try = 0U;
            s_pivot_ms  = 0U;
            lf_stop_wheels();               /* 立刻停住, 不要冲过节点 */
            return;
        }

        /* 还没到判定时间: 先按上次的方向继续找, 同时降速。
         * 这一段是为了让线上一个小缺口能直接开过去, 不误触发停车。 */
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
