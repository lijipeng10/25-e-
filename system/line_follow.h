#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdint.h>

/* line_follow.h —— 循迹(双环: 外环把位置误差变成目标航向, 内环追目标航向)。
 * 用法: 开机 line_follow_init() 一次; KEY1 按一下开、再按一下停(start/stop);
 * 主循环里用 tick_get_ms 分频, 每 10ms 调一次 line_follow_step()(没在跑时内部直接返回)。
 * ★ 调参入口全在 line_follow.c 顶部的"可调参数"区, 带 SPEED 字样的单位都是 mm/s。 */

/* 初始化(上电调一次, 不会让车动) */
void line_follow_init(void);

/* 开始循迹 / 停止循迹(停止会把两个轮子关掉) */
void line_follow_start(void);
void line_follow_stop(void);

/* 当前是否在跑: 1 = 在循迹, 0 = 停着 */
uint8_t line_follow_is_running(void);

/* 当前是否在"弯道原地转向"过程中: 1 = 正在原地转, 0 = 不在(屏幕上显示 LF:PIVOT 就是它) */
uint8_t line_follow_is_pivoting(void);

/* 控制步进: 建议每 10ms 调用一次 */
void line_follow_step(void);

/* ---------- 下面几个是给 OLED/串口 调试看的, 不调也行 ---------- */

/* 最近一次的线偏差: -100(线在最左) ~ 0(正中) ~ +100(线在最右) */
int16_t line_follow_get_error(void);

/* 最近一次 8 路灰度的位图: bit0 = 最左那路, 1 = 压线(已经和 LF_LINE_LEVEL 比过了) */
uint8_t line_follow_get_bits(void);

/* 最近一次 8 路灰度的"原始值"(每路 0 或 1, 没做任何极性判断)。
 * 标定传感器看这个: 车压线时对应位变不变, 一眼就能判断极性对不对 */
void line_follow_get_raw(uint16_t *out);

/* ★ line_follow_test_wheels() 已删除 —— 现在测电机用 KEY2(直接命令速度, 走速度环)。 */

/* 速度环【实际给出】的占空比(0~100), 不再是"我们命令了多少"。
 * 它是一把尺子: 一直顶在 20 = LF_BASE_SPEED 定高了、占空比饱和、差速没余量; 稳在 12~17 = 健康 */
uint8_t line_follow_get_left_duty(void);
uint8_t line_follow_get_right_duty(void);

/* 把当前所有可调参数导出来给屏幕显示用:
 *     uint16_t p[LF_P_COUNT];
 *     line_follow_get_params(p);
 *     p[LF_P_BASE] 就是 LF_BASE_SPEED 当前的值 ... */
/* ★ 速度类的参数(下面带 SPEED 字样的)单位都是 mm/s, 不是占空比 */
#define LF_P_BASE        0      /* 基础速度 LF_BASE_SPEED (mm/s) */
#define LF_P_STEER       1      /* 转向量上限 LF_MAX_STEER (= 转弯力度) */
#define LF_P_HEAD        2      /* 内环比例 LF_HEAD_KP (航向差 -> 转向量) */
#define LF_P_DEADBAND    3      /* 误差死区 LF_DEADBAND */
#define LF_P_TRIM        4      /* 左右电机补偿 LF_TRIM (带符号) */
#define LF_P_LOST        5      /* 丢线找线速度 LF_LOST_SPEED (mm/s) */
#define LF_P_MAX         6      /* 转向量上限 LF_MAX_STEER (mm/s) */
#define LF_P_PIV_TRIG    7      /* 丢线多久判定到弯节点 LF_PIVOT_TRIGGER_MS */
#define LF_P_PIV_DUTY    8      /* 原地转向速度 LF_PIVOT_SPEED (mm/s) */
#define LF_P_PIV_OK      9      /* 对准判据 LF_PIVOT_OK */
#define LF_P_CORNER     10      /* 急弯判据 LF_CORNER_ERR (= 过弯冲过头的关键参数) */
#define LF_P_GYRO       11      /* 陀螺仪阻尼 LF_GYRO_KD (带符号, 治左右摆尾) */
#define LF_P_POS        12      /* 外环比例 LF_POS_KP (位置误差 -> 目标航向) */
#define LF_P_PSI_MAX    13      /* 目标航向限幅 LF_PSI_MAX (0.1度) */
#define LF_P_COUNT      14
void line_follow_get_params(uint16_t *out);

/* 本次运行期间 error 到过的最小 / 最大值(按 KEY1 启动时清零);
 * 车在跑的时候没法盯屏幕, 这个用来"停下车再回头看摆得多厉害" */
void line_follow_get_error_range(int16_t *mn, int16_t *mx);

/* 本次运行期间 error 符号翻了几次(= 来回摆了几次)。
 * 摆得快(次数多但幅度小) -> 控制器太灵敏, 降 LF_POS_KP; 摆得慢(次数少但幅度大) -> 太弱, 加 LF_POS_KP */
uint16_t line_follow_get_error_flips(void);

/* 外环算出的目标航向(0.1度, 左转为正)。双环调参主要看它: 和 mpu6050_get_yaw_x10()(实际航向) 一起看,
 * 两个数差得多 = 内环跟不上(加 LF_HEAD_KP); 它自己跳得厉害 = 外环太猛(降 LF_POS_KP) */
int16_t line_follow_get_psi_ref(void);

#endif /* LINE_FOLLOW_H */