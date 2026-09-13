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
#define MPU_REG_CONFIG    0x1A
#define MPU_REG_GYROCFG   0x1B
#define MPU_REG_GYRO_Z    0x47
#define MPU_REG_PWR_MGMT  0x6B
#define MPU_REG_WHOAMI    0x75

#define GYRO_LSB_PER_DPS  65.5f     /* ±500 dps 量程 */

static int32_t  s_yaw_x10;
static uint32_t s_last_ms;
static float    s_gyro_bias;        /* 零偏(LSB) */

/* ---- 底层: 写寄存器(单次 START+STOP) ---- */
static int i2c_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2];
    uint32_t guard = 200000U;

    buf[0] = reg; buf[1] = val;
    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--guard == 0U) return -1;
    }
    DL_I2C_fillControllerTXFIFO(MPU6050_INST, buf, 2U);
    DL_I2C_startControllerTransfer(MPU6050_INST, MPU_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2U);
    guard = 200000U;
    while (DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (--guard == 0U) return -1;
    }
    if (DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) return -1;
    return 0;
}

/* ---- 底层: 读寄存器(重复起始: 先写寄存器地址不 STOP, 再读) ---- */
static int i2c_read_reg(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i;
    uint32_t guard;

    guard = 200000U;
    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--guard == 0U) return -1;
    }

    /* 阶段1: 寄存器地址, START 但不停 */
    DL_I2C_fillControllerTXFIFO(MPU6050_INST, &reg, 1U);
    DL_I2C_startControllerTransferAdvanced(MPU6050_INST, MPU_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U,
        DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_DISABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);
    guard = 200000U;
    while (DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (--guard == 0U) return -1;
    }
    if (DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) return -1;

    /* 阶段2: 重复起始 + 读 len 字节 + STOP */
    DL_I2C_startControllerTransferAdvanced(MPU6050_INST, MPU_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, len,
        DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_ENABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);
    for (i = 0U; i < len; i++) {
        guard = 200000U;
        while (DL_I2C_isControllerRXFIFOEmpty(MPU6050_INST)) {
            if (--guard == 0U) return -1;
        }
        buf[i] = DL_I2C_receiveControllerData(MPU6050_INST);
    }
    guard = 200000U;
    while (DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (--guard == 0U) return -1;
    }
    if (DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) return -1;
    return 0;
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
    for (i = 0; i < 200; i++) {
        if (i2c_read_reg(MPU_REG_GYRO_Z, b, 2U) == 0) {
            acc += (int32_t)(int16_t)(((uint16_t)b[0] << 8) | b[1]);
            n++;
        }
        delay_ms(2);
    }
    if (n > 0) s_gyro_bias = (float)acc / (float)n;
}

void mpu6050_update(void)
{
    uint8_t b[2];
    uint32_t now, dt;
    int16_t raw;
    float dps;

    if (i2c_read_reg(MPU_REG_GYRO_Z, b, 2U) != 0) return;

    raw = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
    dps = ((float)raw - s_gyro_bias) / GYRO_LSB_PER_DPS;   /* 度/秒 */

    now = tick_get_ms();
    if (s_last_ms == 0U) { s_last_ms = now; return; }

    dt = now - s_last_ms;
    s_last_ms = now;
    if (dt > 100U) dt = 100U;                        /* 防卡顿造成大跳变 */

    /* 0.1° += (度/秒) * (ms/1000) * 10 = 度/秒 * ms / 100 */
    s_yaw_x10 += (int32_t)(dps * (float)dt / 100.0f);

    while (s_yaw_x10 >  1800) s_yaw_x10 -= 3600;
    while (s_yaw_x10 < -1800) s_yaw_x10 += 3600;
}

int32_t mpu6050_get_yaw_x10(void)
{
    return s_yaw_x10;
}

void mpu6050_zero_yaw(void)
{
    s_yaw_x10 = 0;
}
