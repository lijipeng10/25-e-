#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdint.h>

/* ============================================================================
 *  line_follow.h —— 循迹(最基础版)
 * ----------------------------------------------------------------------------
 *  用法(在主循环里):
 *
 *      line_follow_init();          // 开机初始化一次
 *
 *      while (1) {
 *          uint8_t code = key_getnum();
 *          if (code == 1U) {        // KEY1: 按一下开, 再按一下停
 *              if (line_follow_is_running()) line_follow_stop();
 *              else                          line_follow_start();
 *          }
 *
 *          // 每 10ms 跑一次控制(用 tick_get_ms 分频)
 *          if (tick_get_ms() - last >= 10U) {
 *              last = tick_get_ms();
 *              line_follow_step();
 *          }
 *      }
 *
 *  调参入口全在 line_follow.c 顶部的"可调参数"区, 看那里的注释。
 * ==========================================================================*/

/* 初始化(上电调一次, 不会让车动) */
void line_follow_init(void);

/* 开始循迹 / 停止循迹(停止会把两个轮子关掉) */
void line_follow_start(void);
void line_follow_stop(void);

/* 当前是否在跑: 1 = 在循迹, 0 = 停着 */
uint8_t line_follow_is_running(void);

/* 当前是否在"弯道原地转向"过程中: 1 = 正在原地转, 0 = 不在
 * (屏幕上显示 LF:PIVOT 就是它) */
uint8_t line_follow_is_pivoting(void);

/* 控制步进: 建议每 10ms 调用一次(没在跑时内部直接返回, 调了也没关系) */
void line_follow_step(void);

/* ---------- 下面几个是给 OLED/串口 调试看的, 不调也行 ---------- */

/* 最近一次的线偏差: -100(线在最左) ~ 0(正中) ~ +100(线在最右) */
int16_t line_follow_get_error(void);

/* 最近一次 8 路灰度的位图: bit0 = 最左那路, 1 = 压线(已经和 LF_LINE_LEVEL 比过了) */
uint8_t line_follow_get_bits(void);

/* 最近一次 8 路灰度的"原始值"(每路 0 或 1, 没做任何极性判断)
 * 标定传感器时看这个: 车压线时对应位变不变, 一眼就能判断极性对不对 */
void line_follow_get_raw(uint16_t *out);

/* 电机自检: duty = 占空比 0~100, 传 0 = 停。
 * 两个轮子用【相同的占空比 + 相同的前进方向】输出, 所以车如果拐弯,
 * 唯一原因就是两个电机本身的差异 —— 用它量「电机启动死区」和调 LF_TRIM 都很方便。
 * (empty.c 里 KEY2 会一档一档往上加占空比) */
void line_follow_test_wheels(uint8_t duty);

/* 最近一次左右轮实际输出了多少占空比(0~100) */
uint8_t line_follow_get_left_duty(void);
uint8_t line_follow_get_right_duty(void);

/* ---------- 把当前所有可调参数导出来, 给屏幕显示用 ----------
 * 用法:
 *      uint16_t p[LF_P_COUNT];
 *      line_follow_get_params(p);
 *      p[LF_P_BASE] 就是 LF_BASE_DUTY 当前的值 ...
 *
 * 目的: 屏幕上能直接看到「这版固件到底是按什么参数在跑」,
 *       不用去翻源码、也不依赖串口。调参反复改值烧录时特别有用。 */
#define LF_P_BASE        0      /* 基础速度 LF_BASE_DUTY */
#define LF_P_STEER       1      /* 转向量上限 LF_MAX_STEER (= 转弯力度) */
#define LF_P_KP          2      /* 转向比例 LF_KP */
#define LF_P_DEADBAND    3      /* 误差死区 LF_DEADBAND */
#define LF_P_TRIM        4      /* 左右电机补偿 LF_TRIM (带符号) */
#define LF_P_LOST        5      /* 丢线找线速度 LF_LOST_DUTY */
#define LF_P_MAX         6      /* 最高速度硬顶 LF_MAX_DUTY */
#define LF_P_PIV_TRIG    7      /* 丢线多久判定到弯节点 LF_PIVOT_TRIGGER_MS */
#define LF_P_PIV_DUTY    8      /* 原地转向占空比 LF_PIVOT_DUTY */
#define LF_P_PIV_OK      9      /* 对准判据 LF_PIVOT_OK */
#define LF_P_CORNER     10      /* 急弯判据 LF_CORNER_ERR (= 过弯冲过头的关键参数) */
#define LF_P_COUNT      11
void line_follow_get_params(uint16_t *out);

/* 本次运行期间 error 到过的最小 / 最大值(按 KEY1 启动时清零)。
 * 车在跑的时候没法盯屏幕, 这个就是用来"停下车再回头看摆得多厉害"的。 */
void line_follow_get_error_range(int16_t *mn, int16_t *mx);

/* 本次运行期间 error 符号翻了几次(= 来回摆了几次)。
 * 摆得快(次数多但幅度小) -> 控制器太灵敏, 降 LF_KP;
 * 摆得慢(次数少但幅度大) -> 控制器太弱,   加 LF_KP。 */
uint16_t line_follow_get_error_flips(void);

#endif /* LINE_FOLLOW_H */
