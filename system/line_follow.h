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

/* 电机自检: on=1 -> 两个轮子都按"前进"方向转 50%; on=0 -> 停
 * 用来单独验证电机和接线(不经过循迹逻辑), 排查"某个轮子不转"时很好用 */
void line_follow_test_wheels(uint8_t on);

/* 最近一次左右轮实际输出了多少占空比(0~100) */
uint8_t line_follow_get_left_duty(void);
uint8_t line_follow_get_right_duty(void);

#endif /* LINE_FOLLOW_H */
