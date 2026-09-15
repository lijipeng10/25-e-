#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* ============================================================================
 *  encoder.h —— 轮速闭环(编码器测速 + 每轮一个速度 PI)
 * ----------------------------------------------------------------------------
 *  接线: E1A = PB20(GPIOB), E2A = PA25(GPIOA) —— 只数 A 相, 双沿。
 *        ★ 这两路的 GPIO 中断必须在 empty.syscfg 里打开(interruptEn),
 *          并且【必须在 GROUP1_IRQHandler 里把标志清掉】, 否则中断反复重进 -> 死机。
 *
 *  【为什么要它 —— 这是循迹稳不下来的一个结构性原因】
 *
 *  原来电机是【纯开环】的: 代码给"占空比 20%", 到底转多快【没人知道】。
 *  而"占空比 -> 转速"这条关系很脏:
 *
 *    1) ★ 死区: 实测电机启动死区 <= 10(占空比)。
 *       循迹的差速是  左 = 基础 + steer, 右 = 基础 - steer。
 *       基础取 20 时:
 *            steer =  0 -> 慢轮 20   正常
 *            steer = 10 -> 慢轮 10   正好踩在死区边界上
 *            steer = 15 -> 慢轮  5   低于死区 -> 【这个轮子根本不转】
 *       也就是说: 误差一大(弯道口), 那条"平滑的修正曲线"在真实世界里
 *       其实是【阶跃】—— 慢轮直接失速。这不是调参能解决的。
 *
 *    2) 电池掉压: 同样的占空比, 跑半小时之后转得更慢。
 *       于是"今天调好的参数明天不准"。
 *
 *    3) 两个电机不一样: 现在靠 LF_TRIM 一个手调常数硬凑。
 *
 *  闭环之后这三条一起解决: 命令的是【速度(mm/s)】, PI 自己把占空比顶到
 *  需要的值(包括顶过死区), 电压掉了自动补, 两个轮子自动拉平。
 *
 *  【怎么用】
 *      motor_speed_init();          // 开机一次(SYSCFG_DL_init 之后)
 *      主循环里每 10ms 调一次:
 *          motor_speed_update();    // 内部按 MS_LOOP_MS 分频, 20ms 跑一拍
 *      要动轮子:
 *          motor_speed_set(1,  300);   // 1号轮 前进 300 mm/s
 *          motor_speed_set(2, -300);   // 2号轮  后退 300 mm/s
 *          motor_speed_set(1,    0);   // 停
 *
 *  ★★ 和 line_follow 的互斥 ★★
 *      本模块【只有在被 motor_speed_set() 激活之后才会去写电机】。
 *      没激活时 motor_speed_update() 什么都不做, 不会和
 *      line_follow 里直接调 motor_set_duty() 打架。
 *      想交还控制权(比如要开始循迹): 调 motor_speed_disable()。
 *
 *  ★ 已知限制: 编码器只数 A 相的边沿, 【数不出方向】, 所以实测速度的符号
 *    是按【命令方向】给的。循迹时轮子只会按命令方向转, 够用;
 *    真被外力倒拖时读数会不准(那种情况本来也该停车了)。
 *
 *  ★ 本文件原来那两个函数(wheel_encoder_init / wheel_encoder_get_speed)和
 *    speed_1 / speed_2 已经删除 —— 它们从来没被调用过, 而且换算有两处错误
 *    (多乘了 20、也没除以采样时间, 算出来不是速度)。全工程没有任何地方
 *    引用它们, 所以删掉是安全的。
 * ==========================================================================*/

/* ★ GPIOA/GPIOB 的中断共用 IRQ 1。SysConfig 生成的那个名字会随
 *   "哪些模块开了中断"而改变(sm_motor_encoder_* -> GPIO_MULTIPLE_*),
 *   所以这里直接用芯片头文件里的 IRQ 号, 不绑生成名。
 *   (实测: 给 E1A/E2A 开中断时, 名字一变, sm_encoder.c 就编译不过了) */
#define ENC_GPIO_IRQN       GPIOA_INT_IRQn

/* 初始化: 清计数、建 PID、清中断标志、开 NVIC。不会让轮子动。 */
void motor_speed_init(void);

/* ★ 中断服务里调用 —— 由 GROUP1_IRQHandler 传入两个端口的【全部】中断状态。
 *  (必须是全部: 见 sm_encoder.c 里 GROUP1_IRQHandler 的说明, 漏清标志会死机) */
void motor_speed_isr(uint32_t sta_gpioa, uint32_t sta_gpiob);

/* 周期调用(建议 10ms; 内部按 MS_LOOP_MS 分频跑速度环)。没激活时什么都不做。 */
void motor_speed_update(void);

/* 命令某个轮子的速度, 单位 mm/s:
 *      > 0  前进      < 0  后退      0  停(并且把 PI 清干净)
 * 会顺带【激活】本模块(此后它才会去写电机)。*/
void motor_speed_set(uint8_t id, int32_t mm_s);

/* 停一个轮子(等价于 set(id, 0)) */
void motor_speed_stop(uint8_t id);

/* 交还控制权: 停两个轮子 + 关掉本模块的输出, 之后 line_follow 可以自己写电机。
 * ★ 按 KEY1 开始循迹之前必须调它, 否则两边会抢同一个电机。 */
void motor_speed_disable(void);

/* ---------- 调试用 ---------- */
int32_t  motor_speed_get(uint8_t id);         /* 实测速度 mm/s, 带符号 */
int32_t  motor_speed_get_target(uint8_t id);  /* 当前目标速度 mm/s */
uint16_t motor_speed_get_duty(uint8_t id);    /* PI 当前给出的占空比 */
uint8_t  motor_speed_is_active(void);         /* 1 = 本模块正在驱动电机 */

#endif /* ENCODER_H */
