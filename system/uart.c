/* ============================================================================
 *  uart.c —— 通用环形缓冲 UART 驱动
 * ----------------------------------------------------------------------------
 *  本文件绑定 SysConfig 生成的 UART 实例 "smotor_vision"(UART1):
 *    smotor_vision_INST          : 寄存器基址 (UART1)
 *    smotor_vision_INST_INT_IRQN : 中断号 (UART1_INT_IRQn)
 *    中断入口名 smotor_vision_INST_IRQHandler (= UART1_IRQHandler),
 *    由本文件提供并覆盖启动文件的弱默认实现。
 *  引脚: TX=PB4, RX=PB5 (见 SysConfig), 波特率 115200, 8N1。
 *
 *  实现:
 *    - 发送: 阻塞式(等 UART 空闲再发), 简单可靠, 适合当前测试/低速率场合。
 *    - 接收: 中断 + 环形缓冲, 主程序用 uart_available/uart_read_byte 取,
 *             uart_get_line 按行取, 便于解析协议。
 *  用法: SYSCFG_DL_init(); uart_init(); 然后可用 uart_send_uart_get_line。
 * ==========================================================================*/
#include "uart.h"
#include "ti_msp_dl_config.h"

/* 接收环形缓冲大小(必须为 2 的幂, 用 &(size-1) 求余) */
#define RX_BUF_SIZE   256U

static volatile uint8_t  s_rxbuf[RX_BUF_SIZE];
static volatile uint16_t s_rx_head;   /* 写指针(中断往里写) */
static volatile uint16_t s_rx_tail;   /* 读指针(主程序往里读) */
static volatile uint32_t s_rx_total;  /* 已接收字节总数(诊断) */

/* ---------------------------------------------------------------------------
 * 接收环形缓冲: 当前已存字节数 (head - tail) & (size-1)
 * -------------------------------------------------------------------------*/
static uint16_t uart_ring_count(volatile uint16_t head,
                                volatile uint16_t tail, uint16_t size)
{
    return (uint16_t)((head - tail) & (size - 1U));
}

/* ---------------------------------------------------------------------------
 * 初始化: 清缓冲, 设置 RX 一级触发, 开 RX 中断
 * -------------------------------------------------------------------------*/
void uart_init(void)
{
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_total = 0U;

    /* 每收到一个字节就触发 RX 中断(阈值=1) */
    DL_UART_setRXFIFOThreshold(smotor_vision_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);

    /* 清挂起、使能 RX 中断 */
    DL_UART_clearInterruptStatus(smotor_vision_INST, DL_UART_INTERRUPT_RX);
    DL_UART_enableInterrupt(smotor_vision_INST, DL_UART_INTERRUPT_RX);

    NVIC_ClearPendingIRQ(smotor_vision_INST_INT_IRQN);
    NVIC_EnableIRQ(smotor_vision_INST_INT_IRQN);
}

/* ---------------------------------------------------------------------------
 * 发送(阻塞式): 等 UART 空闲再发一个字节
 * -------------------------------------------------------------------------*/
void uart_send_byte(uint8_t ch)
{
    /* 等 UART 不忙, 防止 FIFO 溢出/丢字节 */
    while (DL_UART_isBusy(smotor_vision_INST) != false) { }
    DL_UART_transmitData(smotor_vision_INST, ch);
}

void uart_send_bytes(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    for (i = 0U; i < len; i++) {
        uart_send_byte(buf[i]);
    }
}

void uart_send_str(const char *s)
{
    while (*s != '\0') {
        uart_send_byte((uint8_t)*s);
        s++;
    }
}

/* ---------------------------------------------------------------------------
 * 接收接口
 * -------------------------------------------------------------------------*/
uint16_t uart_available(void)
{
    uint16_t n;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    n = uart_ring_count(s_rx_head, s_rx_tail, RX_BUF_SIZE);
    if (primask == 0U) __enable_irq();
    return n;
}

int uart_read_byte(uint8_t *ch)
{
    if (s_rx_tail == s_rx_head) {     /* 空 */
        return 0;
    }
    *ch = s_rxbuf[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) & (RX_BUF_SIZE - 1U));
    return 1;
}

/* 取一行(以 '\n' 结束, 去掉末尾 '\r')。成功返回 1。 */
int uart_get_line(char *line, uint16_t maxlen)
{
    static char s_line[128];
    static uint16_t s_len = 0U;
    uint8_t ch;
    uint16_t i;

    while (uart_read_byte(&ch) != 0) {
        if (ch == '\n') {
            if (s_len > 0U && s_line[s_len - 1U] == '\r') {
                s_len--;
            }
            if (maxlen == 0U) { s_len = 0U; return 1; }
            for (i = 0U; (i < s_len) && (i < (maxlen - 1U)); i++) {
                line[i] = s_line[i];
            }
            line[i] = '\0';
            s_len = 0U;
            return 1;
        } else if ((ch >= ' ') && (ch < 127)) {   /* 只保存可见字符 */
            if (s_len < (sizeof(s_line) - 1U)) {
                s_line[s_len++] = (char)ch;
            }
        }
    }
    return 0;
}

uint32_t uart_get_rx_total(void)
{
    return s_rx_total;
}

/* ---------------------------------------------------------------------------
 * UART 接收中断: 收到字节就塞进接收环形缓冲
 * 我们只用了 RX 中断(发送是阻塞式), 所以这里只处理 RX。
 * -------------------------------------------------------------------------*/
void smotor_vision_INST_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(smotor_vision_INST) == DL_UART_IIDX_RX)
    {
        /* 把 FIFO 里所有字节取到接收环形缓冲 */
        while (!DL_UART_isRXFIFOEmpty(smotor_vision_INST))
        {
            uint8_t ch = DL_UART_receiveData(smotor_vision_INST);
            uint16_t next = (uint16_t)((s_rx_head + 1U) & (RX_BUF_SIZE - 1U));
            if (next != s_rx_tail) {         /* 没满才存 */
                s_rxbuf[s_rx_head] = ch;
                s_rx_head = next;
                s_rx_total++;
            }
        }
    }
}
