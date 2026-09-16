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

// ---- 4. 显示用的 8 点指数平均: 与 mpu6050.c 的 s_raw[i] += (raw-avg)/8 等价 ----
// C 的整数除法向零截断, 所以这里用 Math.trunc; 必须逐位等价, 否则测的不是同一个算法
const ema = (samples) => {
    let avg = 0;
    const out = [];
    for (const x of samples) { avg += Math.trunc((x - avg) / 8); out.push(avg); }
    return out;
};

const pp = (a) => Math.max(...a) - Math.min(...a);

ok('常数输入能收敛到真值(不会卡在半路)', () => {
    const out = ema(Array(100).fill(16000));
    assert.ok(Math.abs(out[99] - 16000) <= 100, 'settled at ' + out[99]);
});

ok('收敛后彻底不动(整数除法不留极限环)', () => {
    // 整数除法向零截断, 差值 < 8 时加数为 0 -> 停在真值下方最多 7 LSB, 然后完全静止。
    // 这 7 LSB 的静态偏差对"看传感器通不通"没影响, 但不能指望它精确等于真值。
    const out = ema(Array(300).fill(16000));
    assert.equal(out[299], out[249]);                       // 后 50 次一个数都不变
    assert.ok(Math.abs(out[299] - 16000) <= 7, 'error ' + (16000 - out[299]));
});

ok('噪声被压到 1/5 以下(AZ 静止时的乱跳)', () => {
    // 真值 16000, 单次采样 ±300 抖动(交替最坏情况)
    const noisy = Array.from({ length: 400 }, (_, i) => 16000 + (i % 2 ? 300 : -300));
    const rawPP = pp(noisy), avgPP = pp(ema(noisy).slice(100));   // 跳过前 100 次爬升
    assert.ok(avgPP * 5 < rawPP, 'raw pp=' + rawPP + ' avg pp=' + avgPP);
});

ok('真值变了还能跟上(时间常数够快)', () => {
    const out = ema([...Array(100).fill(16000), ...Array(20).fill(0)]);   // 翻个面, AZ 掉到 0
    assert.ok(out[119] < 16000 * 0.15, 'still at ' + out[119]);          // 200ms 内走到 85%
});

console.log('\n' + pass + ' passed');