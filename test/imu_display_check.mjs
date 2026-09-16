// IMU 屏显布局 + 6 轴解码 主机自检（不含硬件）
// 跑法: node test/imu_display_check.mjs
import assert from 'node:assert/strict';

let pass = 0;
const ok = (name, fn) => { fn(); pass++; console.log('  ok - ' + name); };

// ---- 1. 寄存器窗口: 0x3B 起 14 字节必须正好包住 GZ 的高字节 0x48 ----
ok('0x3B 起 14 字节覆盖到 GZ(0x48)', () => {
    const ACCEL_XOUT = 0x3B;
    const GYRO_Z_H = 0x48;
    assert.equal(ACCEL_XOUT + 13, GYRO_Z_H);
});

// ---- 2. 6 轴解码: 与 mpu6050_update() 里的 s_raw[0..5] 逐字节一致 ----
const decode = (b) => [
    (b[0] << 8) | b[1], (b[2] << 8) | b[3], (b[4] << 8) | b[5],
    (b[8] << 8) | b[9], (b[10] << 8) | b[11], (b[12] << 8) | b[13],
].map((v) => (v >= 0x8000 ? v - 0x10000 : v));   // 补码转有符号

ok('正负值解码正确', () => {
    // AZ = +16000(0x3E80), AY = -256(0xFF00), GZ = -1(0xFFFF)
    const b = [0x00, 0x00, 0xFF, 0x00, 0x3E, 0x80, 0x00, 0x00,
               0x00, 0x01, 0x00, 0x02, 0xFF, 0xFF];
    assert.deepEqual(decode(b), [0, -256, 16000, 1, 2, -1]);
});

ok('极端值不越界', () => {
    const b = [0x80, 0x00, 0x7F, 0xFF, 0x80, 0x00, 0x00, 0x00,
               0x7F, 0xFF, 0x80, 0x00, 0x7F, 0xFF];
    assert.deepEqual(decode(b), [-32768, 32767, -32768, 32767, -32768, 32767]);
});

// ---- 3. 屏显排版: 6 轴原始值必须放得下(SSD1306 128x64, 12px 字体 6x12) ----
const CHAR_W = 6, CHAR_H = 12, SCREEN_W = 128, SCREEN_H = 64;
const signed5W = CHAR_W + 5 * CHAR_W;              // 符号 + 5 位 = 36px

ok('一行两个轴的横向范围都在屏内', () => {
    const labelX = [0, 64];
    const valueX = [14, 78];
    for (const x of labelX) assert.ok(x + 2 * CHAR_W <= SCREEN_W, 'label ' + x);
    for (const x of valueX) assert.ok(x + signed5W <= SCREEN_W, 'value ' + x);
    // 左列数值不能压到右列标签
    assert.ok(valueX[0] + signed5W <= labelX[1]);
});

ok('所有行纵向都在屏内', () => {
    for (const y of [0, 12, 24, 36, 48]) assert.ok(y + CHAR_H <= SCREEN_H, 'row ' + y);
});

ok('int16 原始值一定放得进 5 位', () => {
    assert.ok(String(-32768).length - 1 <= 5);     // 去掉负号取位数
    assert.ok(String(32767).length <= 5);
});

console.log('\n' + pass + ' passed');