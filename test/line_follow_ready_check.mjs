// 循迹接入自检（纯主机侧，不碰硬件）
// 跑法: node test/line_follow_ready_check.mjs
import assert from 'node:assert/strict';

let pass = 0;
const ok = (name, fn) => { fn(); pass++; console.log('  ok - ' + name); };

// ---- 1. 屏幕排版: 128x64, 12px 字体每位 6x12 ----
const CHAR_W = 6, CHAR_H = 12, SCREEN_W = 128, SCREEN_H = 64;

// empty.c show_status() 里的每一处绘制: [名字, x, 列数, y]
const draws = [
    ['E', 0, 1, 0], ['E值', 6, 4, 0], ['P', 34, 1, 0], ['P值', 40, 4, 0],
    ['Y', 0, 1, 12], ['Y值', 6, 4, 12], ['G', 34, 1, 12], ['G位图', 40, 8, 12],
    ['L', 0, 1, 24], ['L值', 6, 5, 24], ['R', 62, 1, 24], ['R值', 68, 5, 24],
    ['Emn', 0, 3, 36], ['Emn值', 18, 4, 36], ['Emx', 52, 3, 36], ['Emx值', 70, 4, 36],
    ['状态', 0, 11, 48],
];

ok('所有绘制都在 128x64 屏内', () => {
    for (const [name, x, cols, y] of draws) {
        assert.ok(x + cols * CHAR_W <= SCREEN_W, name + ' 右边界 ' + (x + cols * CHAR_W));
        assert.ok(y + CHAR_H <= SCREEN_H, name + ' 下边界 ' + (y + CHAR_H));
    }
});

ok('同一行的左右两栏不重叠', () => {
    const rows = new Map();
    for (const [name, x, cols, y] of draws) {
        if (!rows.has(y)) rows.set(y, []);
        rows.get(y).push([name, x, x + cols * CHAR_W]);
    }
    for (const [y, seg] of rows) {
        seg.sort((a, b) => a[1] - b[1]);
        for (let i = 1; i < seg.length; i++) {
            assert.ok(seg[i][1] >= seg[i - 1][2], y + ' 行: ' + seg[i - 1][0] + ' 和 ' + seg[i][0] + ' 撞了');
        }
    }
});

ok('状态字符串等长, 短字符串不会在屏上留残字', () => {
    const strs = ['ERR-ENCODER', 'MPU:NO     ', 'PIVOT      ', 'RUN        ', 'STOP  K2=GO'];
    for (const s of strs) assert.equal(s.length, 11, JSON.stringify(s));
});

// ---- 2. 外环符号约定: psi_ref = -KP*error (KP=2), 屏上显示 psi_ref/10 度 ----
const psi = (err) => Math.max(-600, Math.min(600, -2 * err));
const pDeg = (err) => Math.trunc(psi(err) / 10);

ok('线偏右(E>0) -> 目标航向为负(往右转); 偏左 -> 为正', () => {
    assert.equal(pDeg(50), -10);
    assert.equal(pDeg(-100), 20);
    for (let e = 14; e <= 100; e += 14) assert.ok(pDeg(e) < 0 && pDeg(-e) > 0);
});

ok('P 一定放得进 3 位(±60 度)', () => {
    for (let e = -100; e <= 100; e++) assert.ok(Math.abs(pDeg(e)) <= 60);
});

// ---- 3. 编码器故障保护状态机, 与 motor.c 的 motor_pid_update() 等价 ----
const DUTY_MAX = 900, FAULT_TICKS = 10, TARGET = 300, KP = 0.5, KI = 0.5;

function sim(speedOf, ticks) {
    let out = 0, cnt = 0, errLast = 0, trip = -1, dutyAfterTrip = -1;
    for (let t = 0; t < ticks; t++) {
        if (trip >= 0) { dutyAfterTrip = 0; continue; }        // 自锁: 不再给占空比
        const now = speedOf(t);
        const e = TARGET - now;
        out += KP * (e - errLast) + KI * e;
        errLast = e;
        out = Math.min(DUTY_MAX, Math.max(0, out));
        if (out >= DUTY_MAX - 1 && now < 1) {
            if (++cnt >= FAULT_TICKS) trip = t;
        } else {
            cnt = 0;
        }
    }
    return { trip, out, dutyAfterTrip };
}

ok('编码器正常 -> 永不误判', () => {
    assert.equal(sim((t) => (t < 1 ? 0 : 300), 400).trip, -1);
    assert.equal(sim((t) => (t < 5 ? 0 : 280), 400).trip, -1);      // 起步慢一点也不算
});

ok('编码器没接(顶死不动) -> 判故障', () => {
    const r = sim(() => 0, 400);
    assert.ok(r.trip >= 0, '没判出来');
    assert.ok(r.trip <= 20, '判得太慢: 第 ' + r.trip + ' 拍');       // 20 拍 = 1s
});

ok('故障后自锁: 不再输出占空比', () => {
    const r = sim(() => 0, 400);
    assert.equal(r.dutyAfterTrip, 0);
});

ok('轮子被卡住(速度恒 0) 也算故障, 这是有意的', () => {
    assert.ok(sim(() => 0, 400).trip >= 0);
});

// ---- 4. 电机极性: 实测确认过的那一组, 谁再改都得先解释清楚 ----
// (实测: 原来 A路倒转、B路是对的; 一度把两个都翻了, 结果 B路反而倒转)
const { readFileSync } = await import('node:fs');
const { fileURLToPath } = await import('node:url');
const { dirname, join } = await import('node:path');
const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const motorSrc = readFileSync(join(root, 'hardware/motor.c'), 'utf8');
const fnSrc = motorSrc.split('void motor_set_direction')[1].split('void motor_set_duty')[0];

// 取某个电机 direction == 1 分支里的引脚操作, 例: 'set1,clear2'
const forwardPins = (id) => {
    const seg = fnSrc.split('if(id == ' + id + ')')[1];
    const dir1 = seg.split('if(direction == 1)')[1].split('else if')[0];
    return [...dir1.matchAll(/DL_GPIO_(set|clear)Pins\(motor_\w+?IN(\d)_PORT/g)]
        .map((m) => m[1] + m[2]).join(',');
};

ok('两个电机 direction=1 都是 拉高 IN1 + 拉低 IN2 (实测极性, 勿改)', () => {
    assert.equal(forwardPins(1), 'set1,clear2', 'A路(左轮)极性被改了');
    assert.equal(forwardPins(2), 'set1,clear2', 'B路(右轮)极性被改了');
});

console.log('\n' + pass + ' passed');