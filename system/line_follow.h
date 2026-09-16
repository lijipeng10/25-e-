#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdint.h>

/* line_follow.h —— 循迹(只用灰度 + 编码器速度环, 【不接陀螺仪】)。
 * 用法: 开机 line_follow_init() 一次; KEY2 开/关(line_follow_start/stop);
 *       主循环里每 10ms 调一次 line_follow_step()。
 * ★ 调参入口全在 line_follow.c 顶部的宏, 一共 5 个。 */

/* 初始化(上电调一次, 不会让车动) */
void line_follow_init(void);

/* 开始 / 停止循迹(停止会把两个轮子关掉) */
void line_follow_start(void);
void line_follow_stop(void);

/* 1 = 正在循迹 */
uint8_t line_follow_is_running(void);

/* 1 = 刚因为【丢线】停下来(屏幕显示 LOST); 重新按 KEY2 才继续 */
uint8_t line_follow_is_lost(void);

/* 控制步进: 建议每 10ms 调用一次。★ 没在循迹时也会读灰度、算偏差和轮速命令,
 * 只是不写电机 —— 所以车停着就能在屏幕上核对传感器和转向方向。 */
void line_follow_step(void);

/* ---------- 下面几个是给 OLED 看的, 不调也行 ---------- */

/* 最近一次线偏差: -100(线在最左) ~ 0(正中) ~ +100(线在最右) */
int16_t line_follow_get_error(void);

/* 最近一次 8 路灰度位图: bit0 = 最左那路, 1 = 压线 */
uint8_t line_follow_get_bits(void);

/* 最近一次算出来的差速量(mm/s, 带符号) */
int16_t line_follow_get_steer(void);

/* ★ 两个轮子的【命令】速度(mm/s)。车停着也会更新:
 *   把车压在线上左右挪, 看是不是"线偏右 -> 左轮大、右轮小", 转向方向一测就知道 */
int16_t line_follow_get_cmd_left(void);
int16_t line_follow_get_cmd_right(void);

/* 本次运行期间 error 到过的最小 / 最大值(按 KEY2 启动时清零) */
void line_follow_get_error_range(int16_t *mn, int16_t *mx);

#endif /* LINE_FOLLOW_H */
