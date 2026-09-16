// MPU6050 解码 + 平滑 主机自检（纯主机侧，不碰硬件）
// 跑法: node test/imu_display_check.mjs
// ★ 屏幕排版已经搬到 test/line_follow_ready_check.mjs（屏换成循迹调试屏了）
import assert from 'node:assert/strict';

let pass = 0;
const ok = (name, fn) => { fn(); pass++; console.log('  ok - ' + name); };

// ---- 1. 寄存器窗口: 0x3B 起 14 字节必须正好包住 GZ 的高字节 0x48 ----
ok('0x3B 起 14 字节覆盖到 GZ(0x48)', () => {
    const ACCEL_XOUT = 0x3B, GYRO_Z_H = 0x48;
    assert.equal(ACCEL_XOUT + 13, GYRO_Z_H);
});

// ---- 2. 6 轴解码: 与 mpu6050.c 的 decode6() 逐字节一致 ----
const decode = (b) => [
    (b[0] << 8) | b[1], (b[2] << 8) | b[3], (b[4] << 8) | b[5],
    (b[8] << 8) | b[9], (b[10] << 8) | b[11], (b[12] << 8) | b[13],
].map((v) => (v >= 0x8000 ? v - 0x10000 : v));   // 补码转有符号

ok('正负值解码正确(温度在 b[6]~b[7], GX 从 b[8] 起)', () => {
    const b = [0x00, 0x00, 0xFF, 0x00, 0x3E, 0x80, 0x00, 0x00,
               0x00, 0x01, 0x00, 0x02, 0xFF, 0xFF];
    assert.deepEqual(decode(b), [0, -256, 16000, 1, 2, -1]);
});

ok('极端值不越界', () => {
    const b = [0x80, 0x00, 0x7F, 0xFF, 0x80, 0x00, 0x00, 0x00,
               0x7F, 0xFF, 0x80, 0x00, 0x7F, 0xFF];
    assert.deepEqual(decode(b), [-32768, 32767, -32768, 32767, -32768, 32767]);
});

// ---- 3. 显示用的 8 点指数平均: 与 mpu6050.c 的 s_raw[i] += (raw-avg)/8 等价 ----
// C 的整数除法向零截断, 所以这里用 Math.trunc; 必须逐位等价, 否则测的不是同一个算法
const ema = (samples) => {
    let avg = 0;
    const out = [];
    for (const x of samples) { avg += Math.trunc((x - avg) / 8); out.push(avg); }
    return out;
};
const pp = (a) => Math.max(...a) - Math.min(...a);

ok('常数输入能收敛到真值附近(不会卡在半路)', () => {
    const out = ema(Array(100).fill(16000));
    assert.ok(Math.abs(out[99] - 16000) <= 7, 'settled at ' + out[99]);
});

ok('收敛后彻底不动(整数除法不留极限环)', () => {
    const out = ema(Array(300).fill(16000));
    assert.equal(out[299], out[249]);                       // 后 50 次一个数都不变
    assert.ok(Math.abs(out[299] - 16000) <= 7, 'error ' + (16000 - out[299]));
});

ok('噪声被压到 1/5 以下(AZ 静止时的乱跳)', () => {
    const noisy = Array.from({ length: 400 }, (_, i) => 16000 + (i % 2 ? 300 : -300));
    const rawPP = pp(noisy), avgPP = pp(ema(noisy).slice(100));
    assert.ok(avgPP * 5 < rawPP, 'raw pp=' + rawPP + ' avg pp=' + avgPP);
});

ok('真值变了还能跟上(时间常数够快)', () => {
    const out = ema([...Array(100).fill(16000), ...Array(20).fill(0)]);
    assert.ok(out[119] < 16000 * 0.15, 'still at ' + out[119]);
});

console.log('\n' + pass + ' passed');