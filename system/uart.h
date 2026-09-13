#ifndef UART_H
#define UART_H

#include <stdint.h>

/* ============================================================================
 *  uart.h —— 通用环形缓冲 UART 驱动(可复用)
 * ---------------------------------------------------------------------------
 *  说明:
 *   - 接收: 用 UART 中断 + 环形缓冲, 中断到就把字节塞进缓冲区, 主程序随时取。
 *   - 发送: 非阻塞, 先写进发送环形缓冲, 由中断逐字节发出。
 *   - 依赖 SysConfig 生成的 UART 实例宏(UART_0_INST / UART_0_INT_IRQn)。
 *
 *  使用顺序:
 *     SYSCFG_DL_init();   // SysConfig 已配置波特率/引脚/使能
 *     uart_init();        // 开启接收中断 + 清缓冲
 *     ...
 *     uart_send_str("OK\r\n");
 *     uart_get_line(buf, len);   // 取一行
 * ==========================================================================*/

/* 初始化: 清空环形缓冲, 开接收中断 */
void uart_init(void);

/* ---- 发送(非阻塞) ---- */
void uart_send_byte(uint8_t ch);
void uart_send_bytes(const uint8_t *buf, uint16_t len);
void uart_send_str(const char *s);

/* ---- 接收 ---- */
/* 返回接收缓冲里当前有多少字节待读 */
uint16_t uart_available(void);
/* 读一个字节; 无数据返回 0, 成功返回 1 */
int      uart_read_byte(uint8_t *ch);
/* 取一行(以 '\n' 结束, 自动去掉末尾 '\r')。
 * 成功返回 1 并把内容写入 line; 尚未收到完整行返回 0。 */
int      uart_get_line(char *line, uint16_t maxlen);

/* 已接收字节总数(诊断: 看数据是否真的到达视觉串口) */
uint32_t uart_get_rx_total(void);

#endif /* UART_H */
