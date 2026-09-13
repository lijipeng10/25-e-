/* ============================================================================
 *  vision.c —— 视觉串口接口 + 视觉伺服(开环: 图像误差 -> 速度 PID)
 * ----------------------------------------------------------------------------
 *  从视觉串口(UART1/smotor_vision) 收二进制帧:
 *      [0xAA][0x55][pan_lo][pan_hi][tilt_lo][tilt_hi][checksum]
 *     pan / tilt : int16 小端, 单位 0.1°(带符号), 是"黑框中心相对激光点"的偏差
 *     checksum   : pan_lo ^ pan_hi ^ tilt_lo ^ tilt_hi  (XOR)
 *  解析后各轴用通用 PID 模块(pid.c)做速度输出 -> sm_motor_SetSpeed 驱动云台回中。
 *  *注意*: 这是开环(无编码器反馈); 若需角度闭环, 把 pid_update 的输入换成
 *   (目标角度 - encoder_get_angle_x10) 即可。
 *  依赖: uart.c(字节收) + sm_motor.c(SetSpeed) + tick.c(毫秒时基) + pid.c(通用PID)。
 * ==========================================================================*/
#include "vision.h"
#include "ti_msp_dl_config.h"
#include "uart.h"       /* uart_init / uart_read_byte */
#include "sm_motor.h"   /* sm_motor_SetSpeed / Stop */
#include "tick.h"       /* tick_get_ms */
#include "pid.h"        /* 通用 PID 模块 */

/* ---- 协议帧常量(与视觉端 Python hardware/comm.py 保持一致) ---- */
#define FRAME_SOF0     0xAAU
#define FRAME_SOF1     0x55U
#define FRAME_PAYLOAD  4U          /* pan_lo pan_hi tilt_lo tilt_hi */

/* 命令输入串口: 1=UART1(PB5, 真视觉端), 2=UART2(PC_uart, RX=PB16, 调试用)  */
#define CMD_UART  1

/* ===========================================================================
 *  视觉伺服 PID 参数 —— 传给 pid_init, 按实际机构标定后微调
 * =========================================================================*/
#define PID_KP        5       /* 比例 */
#define PID_KI        1       /* 积分(闭环消稳态误差; 太大易震荡, 联调时按现象调) */
#define PID_KD        4       /* 微分 */
#define PID_DIV       10      /* 输出归一分母 */
#define PID_I_CLAMP   20000   /* 积分累加限幅 */
#define PID_MAX_RPM   20      /* 输出限幅(RPM) */
#define PID_DEADBAND  2       /* 死区(0.1°) */
#define PID_DT_MIN    5       /* 最小合法 dt(ms) */
#define FRAME_TIMEOUT_MS 300  /* 无帧超时停电机(安全) */

/* 方向反了就改 1(两轴独立, 等价视觉端 config.INV) */
#define SERVO_INVERT_PAN   1
#define SERVO_INVERT_TILT  0

static int32_t  s_pan;              /* 最近 pan 偏差(0.1°) */
static int32_t  s_tilt;             /* 最近 tilt 偏差 */
static uint32_t s_frames;           /* 成功帧数 */
static uint32_t s_last_frame_ms;    /* 最近一帧时间戳 */
static uint32_t s_last_pid_ms;      /* 上次 PID 步进时间戳 */

/* 两轴独立的通用 PID 实例 [0]=Pan 轴1, [1]=Tilt 轴2 */
static Pid s_pid[2];

/* ---------------------------------------------------------------------------
 * PC 调试串口(UART2)阻塞发送工具
 * -------------------------------------------------------------------------*/
static void pc_send_char(char c)
{
    while (DL_UART_isBusy(PC_uart_INST)) { }
    DL_UART_transmitData(PC_uart_INST, (uint8_t)c);
}

static void pc_send_str(const char *s)
{
    while (*s != '\0') pc_send_char(*s++);
}

static void pc_send_i32(int32_t v)
{
    char b[12];
    int i = 11;
    b[i--] = '\0';
    if (v < 0) { pc_send_char('-'); v = -v; }
    if (v == 0) b[i--] = '0';
    while (v > 0) { b[i--] = (char)('0' + (v % 10)); v /= 10; }
    pc_send_str(&b[i + 1]);
}

/* 主程序用的 PC 串口打印(心跳/状态)。 */
void pc_uart_print_str(const char *s)
{
    pc_send_str(s);
}

/* ---------------------------------------------------------------------------
 * 二进制帧状态机: 每次喂一个字节, 凑齐一帧且校验通过返回 1。
 * -------------------------------------------------------------------------*/
static uint8_t s_sof_state;
static uint8_t s_pay_buf[FRAME_PAYLOAD];
static uint8_t s_pay_idx;

static int frame_feed(uint8_t ch)
{
    switch (s_sof_state) {
    case 0:
        if (ch == FRAME_SOF0) s_sof_state = 1;
        return 0;
    case 1:
        if (ch == FRAME_SOF1) { s_sof_state = 2; s_pay_idx = 0; }
        else if (ch != FRAME_SOF0) s_sof_state = 0;   /* 两个 SOF0 连写情况 */
        return 0;
    case 2:
        s_pay_buf[s_pay_idx++] = ch;
        if (s_pay_idx >= FRAME_PAYLOAD) s_sof_state = 3;
        return 0;
    default: {                          /* 收校验字节 */
        uint8_t cs = s_pay_buf[0] ^ s_pay_buf[1] ^ s_pay_buf[2] ^ s_pay_buf[3];
        if (ch == cs) { s_sof_state = 0; return 1; }
        /* 校验失败: 若该字节恰好是新帧 SOF0, 直接进状态1, 否则回到0 */
        s_sof_state = (ch == FRAME_SOF0) ? 1U : 0U;
        return 0;
    }
    }
}

/* ---------------------------------------------------------------------------
 * 视觉伺服: 用同一 dt 对两轴各做一次 PID(开环), 输出转速
 * -------------------------------------------------------------------------*/
static void visual_servo(int32_t pe, int32_t te, uint32_t now_ms)
{
    uint32_t dt = PID_DT_MIN;
    int32_t  sp0, sp1;

    if ((s_last_pid_ms != 0U) && (now_ms > s_last_pid_ms)) {
        dt = now_ms - s_last_pid_ms;
    }
    if (dt < PID_DT_MIN) dt = PID_DT_MIN;
    s_last_pid_ms = now_ms;

    sp0 = pid_update(&s_pid[0], pe, dt);   /* Pan */
    sp1 = pid_update(&s_pid[1], te, dt);   /* Tilt */

    if (SERVO_INVERT_PAN)  sp0 = -sp0;   /* 两轴独立反相 */
    if (SERVO_INVERT_TILT) sp1 = -sp1;

    sm_motor_SetSpeed(1U, sp0);
    sm_motor_SetSpeed(2U, sp1);
}

/* ---------------------------------------------------------------------------
 * 初始化: 清状态, 配置两轴 PID, 开视觉串口接收
 * -------------------------------------------------------------------------*/
void vision_init(void)
{
    s_pan = 0; s_tilt = 0; s_frames = 0;
    s_last_frame_ms = 0; s_last_pid_ms = 0;
    s_sof_state = 0; s_pay_idx = 0;

    pid_init(&s_pid[0], PID_KP, PID_KI, PID_KD, PID_DIV,
             PID_I_CLAMP, PID_MAX_RPM, PID_DEADBAND, PID_DT_MIN);
    pid_init(&s_pid[1], PID_KP, PID_KI, PID_KD, PID_DIV,
             PID_I_CLAMP, PID_MAX_RPM, PID_DEADBAND, PID_DT_MIN);

    uart_init();
}

/* ---------------------------------------------------------------------------
 * 主循环调用: 收字节->凑帧->解析->PID+驱动; 无帧超时则停电机
 * -------------------------------------------------------------------------*/
/* 喂一个字节进帧解析; 凑齐并校验通过就做 PID + 回显 */
static void feed_byte(uint8_t ch)
{
    if (frame_feed(ch)) {
        uint32_t now = tick_get_ms();
        int16_t pe = (int16_t)(s_pay_buf[0] | ((uint16_t)s_pay_buf[1] << 8));
        int16_t te = (int16_t)(s_pay_buf[2] | ((uint16_t)s_pay_buf[3] << 8));

        s_pan = pe; s_tilt = te; s_frames++;
        s_last_frame_ms = now;
        visual_servo((int32_t)pe, (int32_t)te, now);

        /* 回显到 PC 调试串口(UART2), 便于确认通信 */
        pc_send_str("rx: p=");
        pc_send_i32(pe);
        pc_send_str(" t=");
        pc_send_i32(te);
        pc_send_str("\r\n");
    }
}

/* ---------------------------------------------------------------------------
 * 主循环调用: 收命令字节->凑帧->解析->PID+驱动; 无帧超时则停电机
 * -------------------------------------------------------------------------*/
void vision_poll(void)
{
    uint8_t ch;

#if (CMD_UART == 2)
    /* 从 PC 调试串口(UART2/RX=PB16)收命令(轮询 FIFO) */
    while (!DL_UART_isRXFIFOEmpty(PC_uart_INST)) {
        feed_byte(DL_UART_receiveData(PC_uart_INST));
    }
#else
    /* 从视觉串口(UART1/RX=PB5)收命令(中断+环形缓冲) */
    while (uart_read_byte(&ch)) {
        feed_byte(ch);
    }
#endif

    /* 安全: 一段时间没收到帧就停电机, 防止跟踪丢失后跑飞 */
    {
        uint32_t now = tick_get_ms();
        if ((now - s_last_frame_ms) > FRAME_TIMEOUT_MS) {
            sm_motor_Stop(1U);
            sm_motor_Stop(2U);
        }
    }
}

int32_t  vision_get_pan(void)   { return s_pan; }
int32_t  vision_get_tilt(void)  { return s_tilt; }
uint32_t vision_get_frames(void){ return s_frames; }
