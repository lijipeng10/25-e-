/* ============================================================================
 *  mpu6050.c —— MPU6050 Z 轴陀螺积分求航向(MSPM0G3507, 硬件 I2C0)
 * ----------------------------------------------------------------------------
 *  SysConfig: MPU6050_INST = I2C0, SDA=PA0, SCL=PA1
 *  用法: mpu6050_init();  在固定周期(建议 5ms)里 mpu6050_update();
 *        主逻辑读 mpu6050_get_yaw_x10() (单位 0.1°, 范围 -1800~1800)。
 *  说明: 上电初始化时会做一次陀螺零偏标定(要求此时小车静止);
 *        航向纯陀螺积分, 长时间会缓慢漂移, 由循迹 PID 兜底。
 * ==========================================================================*/
#include "mpu6050.h"
#include "ti_msp_dl_config.h"
#include "tick.h"
#include "delay.h"

#define MPU_ADDR          0x68      /* AD0=0 -> 0x68; AD0=1 -> 0x69 */
#define MPU_ADDR_ALT      0x69      /* AD0=1 时的地址。ping 会两个都试 */

/* ★ 实际使用的从机地址。默认 0x68, ping 成功后可能被改成 0x69 ——
 *   很多 MPU6050 模块的 AD0 是悬空/拉高的, 地址其实是 0x69,
 *   只试 0x68 会把"接得好好的传感器"误判成没接。 */
static uint8_t s_addr = MPU_ADDR;

#define MPU_REG_CONFIG    0x1A
#define MPU_REG_GYROCFG   0x1B
#define MPU_REG_ACCEL_XOUT 0x3B     /* 从这里连续读 14 字节 = 6 轴 + 温度 */
#define MPU_REG_GYRO_Z    0x47
#define MPU_REG_PWR_MGMT  0x6B
#define MPU_REG_WHOAMI    0x75

#define GYRO_LSB_PER_DPS  65.5f     /* ±500 dps 量程 */

static int32_t  s_yaw_x10;
static uint32_t s_last_ms;
static float    s_gyro_bias;        /* 零偏(LSB) */

/* ★ 最近一次的 Z 轴角速度, 单位 0.1 度/秒(带符号)。
 * ★ 符号约定【已实测确认】: 车头往【左】转 -> 为正(左正右负)。
 *   原来注释写的"右转为正"是猜的, 写反了, 已按实测更正。
 * 和积分出来的航向角是两回事:
 *    航向角   = "现在车头朝哪"(积分值, 会漂)
 *    角速度   = "车头正在以多快的速度转"(瞬时值, 不漂)
 * 循迹的【阻尼项】要的是角速度 —— 见 line_follow.c 里 LF_GYRO_KD 的说明。 */
static int16_t  s_rate_x10;

/* ★ 6 轴原始值(指数平均后): 0~2 = AX AY AZ, 3~5 = GX GY GZ。
 *   存的是【没换算的 LSB】, 只给屏显看传感器通不通, 控制逻辑不用它。
 *   ★ 为什么平滑: 单次采样本身抖几百 LSB, 100ms 抓一张快照的话,
 *     屏幕上每次都是不同的随机值, 看着就是"乱跳", 根本读不出数。 */
static int32_t  s_raw[6];

/* ---- 底层: 写寄存器(单次 START+STOP) ---- */
static int i2c_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2];
    uint32_t guard = 200000U;

    buf[0] = reg;
    buf[1] = val;

    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_IDLE))
    {
        if (--guard == 0U)
        {
            return -1;
        }
    }

    DL_I2C_fillControllerTXFIFO(MPU6050_INST, buf, 2U);
    DL_I2C_startControllerTransfer(MPU6050_INST, s_addr,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2U);

    guard = 200000U;

    while (DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_BUSY)
    {
        if (--guard == 0U)
        {
            return -1;
        }
    }

    if (DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_ERROR)
    {
        return -1;
    }

    return 0;
}

/* ---- 底层: 读寄存器(重复起始: 先写寄存器地址不 STOP, 再读) ---- */
static int i2c_read_reg(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i;
    uint32_t guard;

    guard = 200000U;

    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_IDLE))
    {
        if (--guard == 0U)
        {
            return -1;
        }
    }

    /* 阶段1: 寄存器地址, START 但不停 */
    DL_I2C_fillControllerTXFIFO(MPU6050_INST, &reg, 1U);
    DL_I2C_startControllerTransferAdvanced(MPU6050_INST, s_addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U,
        DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_DISABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);

    guard = 200000U;

    while (DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_BUSY)
    {
        if (--guard == 0U)
        {
            return -1;
        }
    }

    if (DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_ERROR)
    {
        return -1;
    }

    /* 阶段2: 重复起始 + 读 len 字节 + STOP */
    DL_I2C_startControllerTransferAdvanced(MPU6050_INST, s_addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, len,
        DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_ENABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);

    for (i = 0U; i < len; i++)
    {
        guard = 200000U;

        while (DL_I2C_isControllerRXFIFOEmpty(MPU6050_INST))
        {
            if (--guard == 0U)
            {
                return -1;
            }
        }

        buf[i] = DL_I2C_receiveControllerData(MPU6050_INST);
    }

    guard = 200000U;

    while (DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_BUSY)
    {
        if (--guard == 0U)
        {
            return -1;
        }
    }

    if (DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_ERROR)
    {
        return -1;
    }

    return 0;
}

/* 把 0x3B 起的 14 字节解成 6 轴: 0~2 = AX AY AZ, 3~5 = GX GY GZ
 * (b[6]~b[7] 是温度, 跳过; 所以 GX 从 b[8] 开始, 不是 b[6]) */
static void decode6(const uint8_t *b, int16_t *raw)
{
    raw[0] = (int16_t)(((uint16_t)b[0] << 8) | b[1]);       /* AX */
    raw[1] = (int16_t)(((uint16_t)b[2] << 8) | b[3]);       /* AY */
    raw[2] = (int16_t)(((uint16_t)b[4] << 8) | b[5]);       /* AZ */
    raw[3] = (int16_t)(((uint16_t)b[8] << 8) | b[9]);       /* GX */
    raw[4] = (int16_t)(((uint16_t)b[10] << 8) | b[11]);     /* GY */
    raw[5] = (int16_t)(((uint16_t)b[12] << 8) | b[13]);     /* GZ */
}

void mpu6050_init(void)
{
    uint8_t b[2];
    int32_t acc = 0;
    int       n = 0;
    int       i;

    s_yaw_x10 = 0;
    s_last_ms = 0U;
    s_gyro_bias = 0.0f;

    delay_ms(50);                                   /* 上电稳定 */

    (void)i2c_write_reg(MPU_REG_PWR_MGMT, 0x01);    /* 唤醒, 用陀螺X PLL 作时钟 */
    delay_ms(10);
    (void)i2c_write_reg(MPU_REG_CONFIG, 0x03);      /* DLPF ~44Hz */
    (void)i2c_write_reg(MPU_REG_GYROCFG, 0x08);     /* 陀螺 ±500 dps */
    delay_ms(10);

    /* 零偏标定: 要求此时静止 */
    for (i = 0; i < 200; i++)
    {
        if (i2c_read_reg(MPU_REG_GYRO_Z, b, 2U) == 0)
        {
            acc += (int32_t)(int16_t)(((uint16_t)b[0] << 8) | b[1]);
            n++;
        }

        delay_ms(2);
    }

    if (n > 0)
    {
        s_gyro_bias = (float)acc / (float)n;
    }
}

void mpu6050_update(void)
{
    uint8_t b[14];
    uint32_t now, dt;
    int16_t raw[6];
    uint8_t i;
    float dps;

    /* 一次读完 6 轴(0x3B 起 14 字节, 中间 b[6]~b[7] 是温度, 跳过) */
    if (i2c_read_reg(MPU_REG_ACCEL_XOUT, b, 14U) != 0)
    {
        return;
    }

    decode6(b, raw);

    /* 指数平均: 每次挪 1/8。10ms 采一次 -> 时间常数约 80ms, 看着稳但不迟钝 */
    for (i = 0U; i < 6U; i++)
    {
        s_raw[i] += ((int32_t)raw[i] - s_raw[i]) / 8;
    }

    /* ★ 角速度用【瞬时值 raw[5]】, 不用平滑值: 循迹阻尼项要的是快, 不是稳 */
    dps = ((float)raw[5] - s_gyro_bias) / GYRO_LSB_PER_DPS;     /* 度/秒 */

    /* 记下瞬时角速度(0.1 度/秒) —— 循迹的阻尼项要用它 */
    s_rate_x10 = (int16_t)(dps * 10.0f);

    now = tick_get_ms();

    if (s_last_ms == 0U)
    {
        s_last_ms = now;
        return;
    }

    dt = now - s_last_ms;
    s_last_ms = now;

    if (dt > 100U)
    {
        dt = 100U;                                  /* 防卡顿造成大跳变 */
    }

    /* 0.1° += (度/秒) * (ms/1000) * 10 = 度/秒 * ms / 100 */
    s_yaw_x10 += (int32_t)(dps * (float)dt / 100.0f);

    while (s_yaw_x10 > 1800)
    {
        s_yaw_x10 -= 3600;
    }

    while (s_yaw_x10 < -1800)
    {
        s_yaw_x10 += 3600;
    }
}

int32_t mpu6050_get_yaw_x10(void)
{
    return s_yaw_x10;
}

void mpu6050_zero_yaw(void)
{
    s_yaw_x10 = 0;
}

int16_t mpu6050_get_rate_x10(void)
{
    return s_rate_x10;
}


void mpu6050_get_raw(int16_t *raw)
{
    uint8_t i;

    for (i = 0U; i < 6U; i++)
    {
        raw[i] = (int16_t)s_raw[i];
    }
}

/* ---------------------------------------------------------------------------
 *  探测传感器在不在。
 *
 *  为什么需要它: mpu6050_init() 里的零偏标定要做 200 次读取。如果传感器没接,
 *  每次都失败并等满超时(约 30ms), 200 次就是【十几秒】—— 开机会卡在那儿,
 *  看起来就像"死机"。先 ping 一下(一次读取, 最坏 30ms)就能避开。
 *
 *  判据: I2C 读到 WHO_AM_I 不超时(说明有器件应答), 而且值既不是全 0 也不是
 *        全 1(排除总线悬空)。不强制等于 0x68, 因为兼容芯片返回值可能不同。
 *
 *  返回: 1 = 在, 0 = 不在
 * -------------------------------------------------------------------------*/
uint8_t mpu6050_ping(void)
{
    uint8_t b = 0U;
    uint8_t a;

    /* ★ 两个地址都试一遍: 0x68(AD0=0) 和 0x69(AD0=1)。
     *   很多模块的 AD0 悬空或拉高, 实际地址就是 0x69 ——
     *   只试 0x68 会把接得好好的传感器误判成"没接"。 */
    for (a = MPU_ADDR; a <= MPU_ADDR_ALT; a++)
    {
        s_addr = a;

        if (i2c_read_reg(MPU_REG_WHOAMI, &b, 1U) == 0)
        {
            /* 应答了(有 ACK), 而且不是全 0/全 1(排除总线悬空) */
            if ((b != 0x00U) && (b != 0xFFU))
            {
                return a;               /* 就用这个地址 */
            }
        }
    }

    s_addr = MPU_ADDR;                  /* 都没找到, 复位成默认 */
    return 0U;
}
