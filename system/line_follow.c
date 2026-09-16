/* ---------- line_follow.c —— 循迹(双环) ---------- */
/*
 * 每 10ms 调一次 line_follow_step(): 读 8 路灰度 -> 算偏差 error(-100 线在最左 ~
 * +100 线在最右) -> 双环算转向量 steer -> 左右差速输出。外环把位置误差变成目标航向,
 * 内环追目标航向; 大角度转弯不靠它, 靠"停车原地转向"。
 * ★ 依赖: motor / grayscale_sensor / encoder / mpu6050 / pid / tick。
 * ★ 下面"可调参数"里带 SPEED 字样的单位都是 mm/s(轮速闭环), 不是占空比。
 */
#include "line_follow.h"
#include "ti_msp_dl_config.h"

#include "motor.h"              /* motor_set_direction / motor_set_duty */
#include "grayscale_sensor.h"   /* Grayscale_Sensor_Read_All */
#include "pid.h"                /* 通用 PID */
#include "tick.h"               /* tick_get_ms */
#include "mpu6050.h"            /* mpu6050_get_rate_x10: 阻尼项要用角速度 */
#include "encoder.h"            /* speed_1 / speed_2 */

/* ---------- 可调参数: 要改就改这里 ----------
 * ★ 速度类(LF_BASE_SPEED / LF_LOST_SPEED / LF_MAX_STEER / LF_PIVOT_SPEED)单位都是 mm/s:
 *   命令的是速度, 占空比由每轮的 PI 自己给(屏幕上 d1/d2 显示的是实际占空比)。 */

#define LF_BASE_SPEED       300     /* 直行基础速度 mm/s; 定太高占空比会顶在 20 饱和, 差速没余量 -> 循迹失控 */
#define LF_LOST_SPEED       250     /* 丢线时的找线速度 mm/s; 必须 <= LF_BASE_SPEED(丢线是降速找线) */

/* 电机编号: 1 = A路(PB17/PB18), 2 = B路(PB19/PB23) */
#define LF_LEFT_ID          1U      /* 左轮接在 A路 */
#define LF_RIGHT_ID         2U      /* 右轮接在 B路 */

/* ★ "哪个方向是前进"只由 hardware/encoder.c 的 MS_LEFT_FWD_DIR / MS_RIGHT_FWD_DIR 管,
 *   轮子转反了就改那里(全工程只此一处), 这里不再有 LF_*_FWD_DIR */
#define LF_TRIM             0       /* 左右电机补偿; 速度环已自动拉平两个电机, 所以给 0 */

#define LF_LINE_LEVEL       1U      /* 灰度读到这个值算"压线"; 压线时对应位反而是 0 就改成 0U */
#define LF_STEER_SIGN       (+1)    /* 转向极性 +1 / -1; 实测"越偏越远"就改成 -1 */

/* ---------- 双环增益: 调参主要就是调这两个 ---------- */
#define LF_POS_KP           2       /* 外环比例: 位置误差 -> 目标航向(0.1度/格); 太大会画龙, 太小会斜着贴线跑 */
#define LF_PSI_MAX          600     /* 目标航向限幅(0.1度, 即 ±60 度); error 卡在 ±100, 它是给 KP 调大时兜底的 */
#define LF_HEAD_KP          100     /* 内环比例: 每 10 度航向差给多少 mm/s(取 100 -> 差 30 度就顶到上限); 小=转不动, 大=抖 */
#define LF_HEAD_KI          0       /* 内环积分: 先不用; 调稳之后它可以补掉稳态航向差 */
#define LF_HEAD_KD          0       /* 内环微分: 灰度误差是台阶信号, 开了只会放大台阶; 要阻尼请用 LF_GYRO_KD */

/* 双环增益的编译期检查: 这两个取 0 都不是"关掉", 而是【车会失控】。 */
#if (LF_POS_KP <= 0)
#error "LF_POS_KP 必须 > 0: 外环取 0 则 psi_ref 恒为 0, 内环会死保起步航向, 车直着冲出去"
#endif
#if (LF_HEAD_KP <= 0)
#error "LF_HEAD_KP 必须 > 0: 内环取 0 则转向量恒为 0(只剩阻尼), 车根本不会拐弯"
#endif
#if (LF_PSI_MAX <= 0)
#error "LF_PSI_MAX 必须 > 0"
#endif

#define LF_GYRO_KD          150     /* 陀螺仪阻尼, 单位 mm/s per (度/秒), 取正号(实测左转为正); 角速度 100 度/秒 给 150 mm/s; ★ 太大会在弯道上把转向吃掉 -> 转反丢线 */
#define LF_DEADBAND         0       /* 误差死区: |error| 不超过它就不修正; ★ 保持 0, 死区会把 ±14 抹平, 台阶反而更摆 */
#define LF_MAX_STEER        300     /* 转向量上限 mm/s; ★ 必须 >= LF_BASE_SPEED, 取相等正好(慢轮能降到 0, 转弯力度最大) */

/* 编译期检查: 参数配错了直接编译报错, 不要等到跑车才发现。 */
#if (LF_LOST_SPEED > LF_BASE_SPEED)
#error "LF_LOST_SPEED 不该大于 LF_BASE_SPEED: 丢线时是【降速】找线"
#endif
#if (LF_MAX_STEER < LF_BASE_SPEED)
#error "LF_MAX_STEER 必须 >= LF_BASE_SPEED: 否则慢的一侧降不到 0, 速度差上不去, 会冲过弯道"
#endif

/* ---------- 弯道: 停车原地转向再前进 ----------
 * 判"到弯节点": 线丢了(8 路都看不见), 或线甩到传感器边上并持续一小会儿(急弯提前判定)。
 * 判"转到位": 转向中重新扫到线且 |error| <= LF_PIVOT_OK。只用灰度闭环, 不靠陀螺仪/编码器。 */

#define LF_CORNER_ERR       71      /* |error| 到 71 = 线已经甩到传感器边上了, 急弯提前判定的阈值 */
#define LF_CORNER_MS        60U     /* 上面的条件要连续满足这么久才算急弯(防止直道上误触发) */
#define LF_PIVOT_TRIGGER_MS 150U    /* 兜底判据: 连续丢线这么久就停车转向; 太短线上一个小缺口就误触发, 太长过弯冲过头 */
#define LF_PIVOT_SPEED      300     /* 原地转向时两个轮子的速度 mm/s(一正一反); 要克服静摩擦, 不该小于 LF_BASE_SPEED */
#define LF_PIVOT_OK         43      /* |error| <= 它就算"对准了", 结束转向; 窗口太小(20)时原地转得太快, 线会整段扫过去抓不到 */
#define LF_PIVOT_MAX_DEG    1100    /* 0.1度: 朝一个方向最多转 110 度, 到了还没对上就掉头; 同时也是防"看见来路"的安全线 */
#define LF_PIVOT_TRY_MS     2500U   /* 兜底超时(ms); 只有没接陀螺仪(yaw 恒为 0)时才用得到 */

/* 弯道参数的编译期检查。★ 必须放在这些宏【定义之后】——
 * 放在前面的话宏还没定义, 会被当成 0, 检查就没意义了。 */
#if (LF_PIVOT_TRIGGER_MS >= LF_PIVOT_TRY_MS)
#error "LF_PIVOT_TRIGGER_MS 必须小于 LF_PIVOT_TRY_MS"
#endif
#if (LF_PIVOT_SPEED < LF_BASE_SPEED)
#error "LF_PIVOT_SPEED 不该小于 LF_BASE_SPEED: 原地转向要克服静摩擦, 得比行进时更有劲"
#endif
#if (LF_PIVOT_OK >= LF_CORNER_ERR)
#error "LF_PIVOT_OK 必须小于 LF_CORNER_ERR: 否则\"还没判定成弯\"就已经算\"对准了\", 判据自相矛盾"
#endif

/* ---------- 控制周期 ---------- */
#define LF_STEP_MS          10U     /* line_follow_step 的调用周期(ms) */

/* ---------- 内部状态 ---------- */

/* 8 路灰度的权重: 最左 -7 ... 最右 +7, 用来算线的重心在哪边 */
static const int8_t LF_WEIGHT[GRAYSCALE_SENSOR_CHANNELS] =
{
    -7, -5, -3, -1, +1, +3, +5, +7
};

static Pid     s_pid;               /* 转向 PID */
static uint8_t s_running;           /* 1 = 正在循迹 */
static uint8_t s_bits;              /* 最近一次灰度位图(1 = 压线, 已按 LF_LINE_LEVEL 判断) */
static uint16_t s_raw[GRAYSCALE_SENSOR_CHANNELS];  /* 最近一次灰度的原始值(0/1) */
static int16_t s_error;             /* 最近一次偏差 -100~+100 */
static int16_t s_psi_ref;           /* 外环算出的目标航向(0.1度, 左转为正), 调试用 */
static int8_t  s_last_dir;          /* 瞬时"上次往哪边拐"(只在丢线那几十毫秒用) */
static uint16_t s_lost_ms;          /* 已经连续丢线多久(ms) */

/* 误差记录器: 记本次运行期间 error 到过的最小/最大值, 每次按 KEY1 启动时清零。
 * 车在跑时没法盯屏幕, 停下车再看: ±30 以内 = 贴线, ±70 以上 = 已经甩到线的边上 */
static int16_t  s_e_min;
static int16_t  s_e_max;

/* 摆动计数器: 数 error 符号翻了几次(必须越过一整档 ±14 才算一次, 免得被 0 附近抖动刷爆)。
 * 摆得快而幅度小 -> 控制器太灵敏, 降 LF_POS_KP; 摆得慢而幅度大 -> 太弱, 加 LF_POS_KP */
static int16_t  s_prev_err;
static uint16_t s_e_flips;

/* 方向证据 = 泄漏积分 s_dir_ev = s_dir_ev*7/8 + error, 只在看到线的时候更新。
 * ★ 弯道方向不能用 s_last_dir(瞬时值): 直道上的抖动会把它翻来翻去, 到弯道跟弯的方向无关 */
static int32_t  s_dir_ev;

static uint8_t  s_pivoting;         /* 1 = 正在原地转向(弯道模式) */
static int8_t   s_pivot_dir;        /* 本次原地转向往哪边(开始时定下, 中途不变) */
static uint8_t  s_pivot_try;        /* 0 = 第一遍(按证据方向), 1 = 第二遍(反方向) */
static uint16_t s_pivot_ms;         /* 当前这一遍已经转了多久(ms) */
static uint16_t s_corner_ms;        /* |error| 已经连续超阈值多久(ms), 急弯判据用 */

/* ---------- 内部函数 ---------- */

/* 读 8 路灰度, 返回位图: bit0 = 最左那路, 1 = 压线(原始值另存在 s_raw 里给调试看)。
 * ★ Grayscale_Sensor_Read_All 内部有 ~400us 延时, 别调太快 */
static uint8_t lf_read_bits(void)
{
    uint16_t g[GRAYSCALE_SENSOR_CHANNELS];
    uint8_t  i;
    uint8_t  bits = 0U;

    Grayscale_Sensor_Read_All(g);       /* 这个函数内部有 ~400us 延时, 别调太快 */

    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++)
    {
        s_raw[i] = g[i];                    /* 原样存一份给调试显示用 */

        if (g[i] == LF_LINE_LEVEL)
        {
            bits |= (uint8_t)(1U << i);     /* 第 i 路压线了, 把第 i 位置 1 */
        }
    }

    return bits;
}

/* 由位图算偏差: 返回 -100(线在最左) ~ 0(正中间) ~ +100(线在最右);
 * 一个通道都没压线(丢线)时, 通过 *on_line 返回 0 */
static int16_t lf_calc_error(uint8_t bits, uint8_t *on_line)
{
    int32_t sum = 0;                    /* 权重累加 */
    uint8_t cnt = 0U;                   /* 压线的通道数 */
    uint8_t i;

    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++)
    {
        if ((bits & (uint8_t)(1U << i)) != 0U)
        {
            sum += (int32_t)LF_WEIGHT[i];
            cnt++;
        }
    }

    if (cnt == 0U)                      /* 丢线 */
    {
        *on_line = 0U;
        return 0;
    }

    *on_line = 1U;

    /* 先取权重平均(得到 -7~+7)再放大到 -100~+100; 写成 sum*100/(cnt*7) 是先乘后除,
     * 避免整数除法丢精度 */
    return (int16_t)((sum * 100) / ((int32_t)cnt * 7));
}

/* 让一个轮子按【速度】转: id = 1(A路) / 2(B路), speed 单位 mm/s。
 * ★ speed 带符号: > 0 前进, < 0 后退, = 0 停 —— 负数是【反转】, 不是停。
 * 方向脚在这里设, 占空比由速度 PI 决定(所以"哪个方向是前进"的定义在 encoder.c) */
static void lf_set_wheel(uint8_t id, int32_t speed)
{
    /* 最后一道保险: 不管上面怎么算, 这里卡住上下限 */
    if (speed > (int32_t)LF_MAX_STEER)
    {
        speed = (int32_t)LF_MAX_STEER;
    }

    if (speed < -(int32_t)LF_MAX_STEER)
    {
        speed = -(int32_t)LF_MAX_STEER;
    }

    /* 方向脚要自己设: motor_pid_set() 只管速度 */
    if (speed >= 0)
    {
        motor_set_direction(id, 1U);
    }
    else
    {
        motor_set_direction(id, 2U);
    }

    motor_pid_set(id, (float)speed);
}

/* 两个轮子一起停(不用 motor_stop(), 分开写更清楚, 也方便以后单独控制某一个轮子) */
static void lf_stop_wheels(void)
{
    lf_set_wheel(LF_LEFT_ID,  0);
    lf_set_wheel(LF_RIGHT_ID, 0);
}

/* 原地转向: 两个轮子一正一反, 车绕自己中心转。dir > 0 = 往右转, dir < 0 = 往左转。
 * ★ 速度环保证真的是 ±LF_PIVOT_SPEED, 所以角速度可重复(LF_PIVOT_MAX_DEG 的角度判据靠这个) */
static void lf_pivot(int8_t dir)
{
    int32_t d = (int32_t)LF_PIVOT_SPEED;

    if (dir > 0)
    {
        lf_set_wheel(LF_LEFT_ID,   d);    /* 左轮前进 */
        lf_set_wheel(LF_RIGHT_ID, -d);    /* 右轮后退 */
    }
    else
    {
        lf_set_wheel(LF_LEFT_ID,  -d);    /* 左轮后退 */
        lf_set_wheel(LF_RIGHT_ID,  d);    /* 右轮前进 */
    }
}

/* ---------- 对外接口 ---------- */

void line_follow_init(void)
{
    /* 初始化【内环(航向环)】PID: div = 100 -> 输出正好是 LF_HEAD_KP*e_psi/100;
     * 输出限幅 = LF_MAX_STEER(内环直接出转向量), 死区给 0(航向是连续量) */
    pid_init(&s_pid, LF_HEAD_KP, LF_HEAD_KI, LF_HEAD_KD, 100,
             0, LF_MAX_STEER, 0, LF_STEP_MS);

    s_running    = 0U;
    s_bits       = 0U;
    s_error      = 0;
    s_last_dir   = 0;
    s_lost_ms    = 0U;
    s_dir_ev     = 0;
    s_pivoting   = 0U;
    s_pivot_dir  = +1;
    s_pivot_try  = 0U;
    s_pivot_ms   = 0U;
    s_corner_ms  = 0U;
    s_e_min      = 0;
    s_e_max      = 0;
    s_prev_err   = 0;
    s_e_flips    = 0U;
}

void line_follow_start(void)
{
    pid_reset(&s_pid);          /* 清掉上次的积分/微分残留, 不然起步会猛地一拐 */

    /* ★★ 航向零点(第一个清零点) —— 按 KEY1 启动的这一瞬间, 车【必须摆正、对着线的方向】,
     * 这一瞬间的车头方向就是整段路的 0 度。起步时歪着, 车就会一路带着这个初始偏差跑。
     * 另一个清零点在弯道原地转向结束时(那里车头刚对准新线, 同样该是 0 度)。 */
    mpu6050_zero_yaw();

    s_lost_ms   = 0U;
    s_last_dir  = 0;
    s_dir_ev    = 0;        /* 方向证据也清空, 从头攒 */
    s_pivoting  = 0U;
    s_pivot_dir = +1;
    s_pivot_try = 0U;
    s_pivot_ms  = 0U;
    s_corner_ms = 0U;
    s_e_min     = 0;        /* 误差记录器清零, 只记这一次运行 */
    s_e_max     = 0;
    s_prev_err  = 0;
    s_e_flips   = 0U;
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

/* 判断"该往哪边转": 优先用方向证据, 不够再退回瞬时方向, 再不够默认往右。
 * ★ 不能直接用 s_last_dir: 它只是线上最后一次小抖动的方向, 到弯道那一刻跟弯的方向无关 */
static int8_t lf_best_dir(void)
{
    if (s_dir_ev > 0)
    {
        return +1;
    }

    if (s_dir_ev < 0)
    {
        return -1;
    }

    if (s_last_dir != 0)
    {
        return s_last_dir;
    }

    return +1;                          /* 什么线索都没有, 默认往右找 */
}

/* 进入"原地转向"状态: 定好方向、清零计时, 并且【立刻把轮子停住】(为了不冲过弯节点) */
static void lf_enter_pivot(void)
{
    s_pivot_dir = lf_best_dir();
    s_pivoting  = 1U;
    s_pivot_try = 0U;
    s_pivot_ms  = 0U;
    s_corner_ms = 0U;

    /* ★ 航向清零(第三个清零点): 从这里开始, |yaw| 就是"这一遍原地转向已经转过了多少度" */
    mpu6050_zero_yaw();

    lf_stop_wheels();
}

/* 控制步进: 每 10ms 调一次 */
void line_follow_step(void)
{
    uint8_t  on_line;
    int16_t  error;
    int32_t  steer;
    int32_t  left_cmd, right_cmd;
    int32_t  base;
    int32_t  psi_ref;           /* 外环输出: 目标航向(0.1度, 左转为正) */
    int32_t  e_psi;             /* 内环输入: 目标航向 - 实际航向 */
    int32_t  damp;              /* 陀螺仪阻尼量 */
    int32_t  turned;            /* 原地转向【沿当前方向】已转过的角度(0.1度, 带符号) */

    /* ---------------- 第 1 步: 读灰度, 算偏差 ----------------
     * ★ 无论循迹跑不跑, 这一步都必须做! 屏幕和串口显示的调试数据就是这里读出来的,
     *   停着不读就没法在不启动电机的情况下标定传感器。 */
    s_bits  = lf_read_bits();
    error   = lf_calc_error(s_bits, &on_line);
    s_error = error;

    /* 没在循迹: 数据照读照更新, 但不驱动电机 */
    if (s_running == 0U)
    {
        return;
    }

    /* 记录本次运行的误差摆幅(见 s_e_min / s_e_max 的说明) */
    if (error < s_e_min)
    {
        s_e_min = error;
    }

    if (error > s_e_max)
    {
        s_e_max = error;
    }

    /* 数摆动次数(见 s_e_flips 的说明): 必须越过一整档 14 才算翻了一次 */
    if (((error >= 14) && (s_prev_err <= -14)) ||
        ((error <= -14) && (s_prev_err >= 14)))
    {
        s_e_flips++;
    }

    s_prev_err = error;

    /* ---------------- 状态 A: 正在原地转向(弯道模式) ---------------- */
    if (s_pivoting != 0U)
    {
        s_pivot_ms += LF_STEP_MS;

        /* ★ 先算"沿当前方向已经转过了多少度" —— 用陀螺仪, 不靠计时。
         * ★★ 符号坑: lf_pivot(dir>0) 是往【右】转, 而 yaw>0 是往【左】转, 两个约定是反的,
         *    所以必须写成 -yaw*dir; 写成 +yaw*dir 的话两个角度判据永远不满足, 车会一路转到底 */
        turned = -(int32_t)mpu6050_get_yaw_x10() * (int32_t)s_pivot_dir;

        /* 判"对准了": 线回到中间附近就停。
         * ★ 这里【故意不加】"至少转过多少度"的门槛 —— 直道上误进原地转向时正确的线就在
         *   原地(yaw≈0), 有门槛车就永远认不回来, 表现为"直线都会掉头" */
        if ((on_line != 0U) && (error <= LF_PIVOT_OK) && (error >= -LF_PIVOT_OK))
        {
            pid_reset(&s_pid);      /* 清掉转向过程中残留的量, 免得接着猛拐一下 */

            /* ★ 航向零点(第二个清零点见文件上面): 能走到这里说明线已经在中间附近, 车头刚
             * 对准新的这段线, 现在这个方向就该是新的 0 度。不清零的话转完弯内环会拼命把车头
             * 扭回【转弯前】的航向 —— 那就直接冲出去了。 */
            mpu6050_zero_yaw();

            s_lost_ms  = 0U;
            s_pivoting = 0U;
            s_dir_ev   = 0;         /* 这一弯的证据用完了, 清空, 下一弯重新攒 */
            /* 这里【不 return】: 直接落下去按正常循迹走一拍, 衔接更顺 */
        }
        else
        {
            /* ★ 一个方向【最多转 LF_PIVOT_MAX_DEG】, 到了还没对上就掉头往反方向找。
             *   没有它车会一直转下去, 转过 180 度后从背后重新看到【来路】那条线, 反着走回去。
             *   (LF_PIVOT_TRY_MS 只当兜底超时: 没接陀螺仪时 yaw 恒为 0, 只能靠它) */
            if ((turned >= (int32_t)LF_PIVOT_MAX_DEG) || (s_pivot_ms >= LF_PIVOT_TRY_MS))
            {
                if (s_pivot_try == 0U)
                {
                    s_pivot_try = 1U;
                    s_pivot_ms  = 0U;
                    s_pivot_dir = (int8_t)(-s_pivot_dir);   /* 掉头 */
                    /* ★ 这里【故意不清零 yaw】: turned = -yaw*dir 是带符号的, 掉头之后 dir 反了,
                     *   同一个 yaw 算出来的 turned 立刻变成负的 —— 于是车先【原路转回去】, 再继续
                     *   往另一头找, 而 |yaw| 全程被 MAX 卡住, 转不到"背对来路"的 180 度去。 */
                }
                else
                {
                    /* 两边都扫遍了还是没有线 -> 车多半已经掉出赛道, 停车 */
                    line_follow_stop();
                    return;
                }
            }

            lf_pivot(s_pivot_dir);  /* 还没对准, 继续原地转 */
            return;
        }
    }

    /* ---------------- 状态 B: 正常循迹 ---------------- */

    /* ---------------- 第 2 步: 算转向量 steer ---------------- */
    if (on_line != 0U)
    {
        /* 看到线了: 正常循迹 */
        s_lost_ms = 0U;

        /* 累积"往哪边偏"的方向证据(见 s_dir_ev 的说明)。只在这里更新 —— 丢线时不更新,
         * 这样弯道口攒下来的证据能保住, 供原地转向决定方向用。 */
        s_dir_ev = (s_dir_ev * 7) / 8 + (int32_t)error;

        if (s_dir_ev >  400)
        {
            s_dir_ev =  400;
        }

        if (s_dir_ev < -400)
        {
            s_dir_ev = -400;
        }

        /* ★ 急弯提前判定(见 LF_CORNER_ERR 的说明): 线已经甩到传感器边上并且持续了一小会儿,
         *   就不等丢线、立刻停车转向 —— 这是"过弯冲过头"的关键补丁。 */
        if ((error >= LF_CORNER_ERR) || (error <= -LF_CORNER_ERR))
        {
            s_corner_ms += LF_STEP_MS;

            if (s_corner_ms >= LF_CORNER_MS)
            {
                lf_enter_pivot();
                return;
            }
        }
        else
        {
            s_corner_ms = 0U;
        }

        base = LF_BASE_SPEED;           /* ★ mm/s */

        /* ---------- 外环: 位置误差 -> 目标航向 ----------
         * 乘 LF_STEER_SIGN 是为了方便一键反方向; 最外层那个负号的来历见文件上面的符号说明。
         * ★ 死区作用在外环上: |error| 不超过 LF_DEADBAND 时目标航向给 0, 默认 0 等于不生效。 */
        psi_ref = -((int32_t)LF_POS_KP * (int32_t)error * LF_STEER_SIGN);

        if ((error <= LF_DEADBAND) && (error >= -LF_DEADBAND))
        {
            psi_ref = 0;
        }

        if (psi_ref >  LF_PSI_MAX)
        {
            psi_ref =  LF_PSI_MAX;
        }

        if (psi_ref < -LF_PSI_MAX)
        {
            psi_ref = -LF_PSI_MAX;
        }

        s_psi_ref = (int16_t)psi_ref;

        /* ---------- 内环: 让车头去追目标航向 ----------
         * ★ e_psi > 0 = "还要往左转", 而往左转要给【负】的 steer -> 取负号。
         *   内环只吃航向差, 不吃位置误差 —— 位置信息已经在 psi_ref 里了。 */
        e_psi = psi_ref - mpu6050_get_yaw_x10();
        steer = -pid_update(&s_pid, e_psi, LF_STEP_MS);

        /* ★ 记"这一次想往哪边拐"必须取【加阻尼之前】的符号: 阻尼在急弯上可能大到把 steer
         *   压成反号, 那样记下来的方向就是反的 -> 丢线时把车往错的方向带。 */
        if (steer > 0)
        {
            s_last_dir = +1;
        }
        else if (steer < 0)
        {
            s_last_dir = -1;
        }

        /* ★ 加上陀螺仪阻尼(见 LF_GYRO_KD 的说明): 角速度(0.1度/秒)/1000 正好是"度/秒 /100"的量纲;
         *   传感器没接时 rate 恒为 0, 这一项自动失效。 */
        damp = (int32_t)LF_GYRO_KD * (int32_t)mpu6050_get_rate_x10() / 1000;
        steer += damp;
    }
    else
    {
        /* 丢线了: 一个通道都没看到黑线 */
        s_corner_ms = 0U;
        s_lost_ms  += LF_STEP_MS;

        /* 兜底判据: 连续丢线够久 -> 判定到弯节点, 停车转向。
         * (正常急弯会先被上面的 LF_CORNER_ERR 抓住, 走到这里的机会不多) */
        if (s_lost_ms >= LF_PIVOT_TRIGGER_MS)
        {
            lf_enter_pivot();
            return;
        }

        /* 还没到判定时间: 继续一边找线一边降速, 好让线上一个小缺口能直接开过去, 不误触发停车。
         * ★ 方向必须用【方向证据 s_dir_ev】, 不能用瞬时方向 —— 后者跟弯的方向没关系, 会拐反。
         * ★★ 修正量必须【和证据强度成正比】, 不能一上来就给满舵: 直道上一道小缺口、一束反光
         *   也会走到这里, 那时证据只有一点点, 满舵会把车猛地甩出去 -> 真的丢线 -> 误进原地转向。 */
        base  = LF_LOST_SPEED;          /* ★ mm/s */
        steer = (int32_t)s_dir_ev / 20;

        if (steer >  LF_MAX_STEER)
        {
            steer =  LF_MAX_STEER;
        }

        if (steer < -LF_MAX_STEER)
        {
            steer = -LF_MAX_STEER;
        }
    }

    /* 修正量往哪边掰, 取决于"线偏的方向"和"轮子快慢"的对应关系:
     *   error < 0 (线在左边) -> steer < 0 -> 左轮变慢、右轮变快 -> 车往左拐  ✓
     * 如果实测反了(越偏越远), 把上面的 LF_STEER_SIGN 改成 -1 即可。 */

    /* 记住这次拐的方向(丢线时要用)。
     * ★ 看线时的那一次已经在上面【加阻尼之前】记过了, 不能在这里再记一遍 ——
     *   这里的 steer 已经带上阻尼, 急弯上可能是反的。 */
    if (on_line == 0U)
    {
        if (steer > 0)
        {
            s_last_dir = +1;
        }
        else if (steer < 0)
        {
            s_last_dir = -1;
        }
    }

    /* ---------------- 第 3 步: 差速输出(★ 单位是 mm/s) ----------------
     *   左 = 基础 + steer, 右 = 基础 - steer
     * ★ steer 就是【速度差的一半】: Δv = 左 - 右 = 2*steer, 而 ω = Δv / W(W = 0.17m)。 */
    left_cmd  = base + steer;       /* 线偏左时 steer<0 -> 左轮慢 */
    right_cmd = base - steer;       /*                    右轮快 */

    /* ★ LF_TRIM 保留着但是 0: 两个电机的差异现在由速度环自动拉平。这里一加一减,
     *   平均速度不变, 只改左右出力的分配。 */
    left_cmd  -= LF_TRIM;
    right_cmd += LF_TRIM;

    /* ★ 限幅: 两边都不许倒转, 慢的一侧最多降到 0。
     *   行进中让慢的那一侧反转, 车会猛地甩出去 —— 倒转是【原地转向】才该干的事。 */
    if (left_cmd  < 0)
    {
        left_cmd  = 0;
    }

    if (right_cmd < 0)
    {
        right_cmd = 0;
    }

    lf_set_wheel(LF_LEFT_ID,  left_cmd);
    lf_set_wheel(LF_RIGHT_ID, right_cmd);
}

/* ---------- 调试接口 ---------- */

void line_follow_get_raw(uint16_t *out)
{
    uint8_t i;

    if (out == 0)
    {
        return;
    }

    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++)
    {
        out[i] = s_raw[i];
    }
}

/* ★ line_follow_test_wheels() 已经【删除】—— 现在测电机用 KEY2 直接命令速度, 走速度环本身 */

int16_t line_follow_get_error(void)
{
    return s_error;
}

uint8_t line_follow_get_bits(void)
{
    return s_bits;
}

/* ★ 这两个返回的是【速度环实际给出的占空比】, 不再是"我们命令了多少"。
 *   它是一把尺子: 一直顶在 20 = LF_BASE_SPEED 定高了、占空比饱和、差速没余量; 稳在 12~17 = 健康 */
uint8_t line_follow_get_left_duty(void)
{
    return 0U;
}

uint8_t line_follow_get_right_duty(void)
{
    return 0U;
}

/* 本次运行期间 error 到过的最小 / 最大值(没启动循迹时都是 0) */
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

/* 本次运行期间 error 符号翻了几次(= 来回摆了几次) */
uint16_t line_follow_get_error_flips(void)
{
    return s_e_flips;
}

/* 外环算出的目标航向(0.1度, 左转为正)。调试用: 和 mpu6050_get_yaw_x10()(实际航向) 对着看,
 * 两个数差得多 = 内环跟不上(该加 LF_HEAD_KP), 它自己跳得厉害 = 外环太猛(该降 LF_POS_KP) */
int16_t line_follow_get_psi_ref(void)
{
    return s_psi_ref;
}

/* 把当前所有可调参数一次性导出来给屏幕显示用(索引见 line_follow.h 的 LF_P_*)。
 * ★ 以后加了新的可调宏, 记得在 .h 里加一个 LF_P_xxx 序号, 并在这里补一行 */
void line_follow_get_params(uint16_t *out)
{
    if (out == 0)
    {
        return;
    }

    /* ★ 单位: 下面带 SPEED 字样的都是 mm/s; LF_P_MAX 导出的也是转向量上限(mm/s) */
    out[LF_P_BASE]     = (uint16_t)LF_BASE_SPEED;
    out[LF_P_STEER]    = (uint16_t)LF_MAX_STEER;
    out[LF_P_HEAD]     = (uint16_t)LF_HEAD_KP;      /* 内环: 每 10 度航向差给多少 mm/s */
    out[LF_P_POS]      = (uint16_t)LF_POS_KP;       /* 外环: 位置误差 -> 目标航向 */
    out[LF_P_DEADBAND] = (uint16_t)LF_DEADBAND;
    out[LF_P_TRIM]     = (uint16_t)((int16_t)LF_TRIM);      /* 可能为负, 调用者按符号解释 */
    out[LF_P_LOST]     = (uint16_t)LF_LOST_SPEED;
    out[LF_P_MAX]      = (uint16_t)LF_MAX_STEER;    /* 已改为"转向量上限 mm/s" */
    out[LF_P_PIV_TRIG] = (uint16_t)LF_PIVOT_TRIGGER_MS;
    out[LF_P_PIV_DUTY] = (uint16_t)LF_PIVOT_SPEED;  /* 已改为"原地转向速度 mm/s" */
    out[LF_P_PIV_OK]   = (uint16_t)LF_PIVOT_OK;
    out[LF_P_CORNER]   = (uint16_t)LF_CORNER_ERR;
    out[LF_P_GYRO]     = (uint16_t)((int16_t)LF_GYRO_KD);   /* 可能为负 */
    out[LF_P_PSI_MAX]  = (uint16_t)LF_PSI_MAX;
}
