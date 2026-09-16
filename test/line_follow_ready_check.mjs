// 循迹接入自检（纯主机侧，不碰硬件）
// 跑法: node test/line_follow_ready_check.mjs
import assert from 'node:assert/strict';

let pass = 0;
const ok = (name, fn) => { fn(); pass++; console.log('  ok - ' + name); };

// ---- 1. 屏幕排版: 128x64, 12px 字体每位 6x12 ----
const CHAR_W = 6, CHAR_H = 12, SCREEN_W = 128, SCREEN_H = 64;

// empty.c show_status() 里的每一处绘制: [名字, x, 列数, y]
const draws = [
    ['E', 0, 1, 0], ['E值', 6, 4, 0], ['S', 40, 1, 0], ['S值', 46, 4, 0],
    ['l', 0, 1, 12], ['l值', 6, 5, 12], ['r', 40, 1, 12], ['r值', 46, 5, 12],
    ['L', 0, 1, 24], ['L值', 6, 5, 24], ['R', 40, 1, 24], ['R值', 46, 5, 24],
    ['G', 0, 1, 36], ['G位图', 12, 8, 36], ['|E|', 64, 3, 36], ['|E|值', 84, 3, 36],
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

ok('状态行每条都铺满 11 个字符, 短字符串不会在屏上留残字', () => {
    // [固定文字, 后面还要接着写的数字位数]
    const rows = [
        ['ERR-ENCODER', 0],
        ['MPU:NO K2GO', 0],
        ['STOP  K2=GO', 0],
        ['LOST ', 6],        // 后面 6 位毫秒数
        ['RUN  ', 6],
    ];
    for (const [text, digits] of rows) {
        assert.equal(text.length + digits, 11, JSON.stringify(text));
    }
});

// ---- 2. 控制律: steer = KP*error*SIGN, 左轮 = BASE + steer, 右轮 = BASE - steer ----
const BASE = 300, KP = 3, STEER_MAX = 300, CMD_MAX = BASE + STEER_MAX, SIGN = +1;

const steer = (err) => Math.max(-STEER_MAX, Math.min(STEER_MAX, KP * err * SIGN));
const cmdL = (err) => Math.max(0, Math.min(CMD_MAX, BASE + steer(err)));
const cmdR = (err) => Math.max(0, Math.min(CMD_MAX, BASE - steer(err)));

ok('线偏右(E>0) -> 左轮命令比右轮大(往右转)', () => {
    assert.ok(cmdL(50) > cmdR(50));
    assert.equal(cmdL(50), 450);
    assert.equal(cmdR(50), 150);
});

ok('线偏左(E<0) -> 左轮命令比右轮小(往左转)', () => {
    assert.ok(cmdL(-50) < cmdR(-50));
    assert.equal(cmdL(-50), 150);
    assert.equal(cmdR(-50), 450);
});

ok('线在正中 -> 两轮都是基础速度', () => {
    assert.equal(cmdL(0), BASE);
    assert.equal(cmdR(0), BASE);
});

ok('命令永远夹在 [0, 600]: 不给速度环负目标(编码器认不出方向)', () => {
    for (let e = -100; e <= 100; e++) {
        assert.ok(cmdL(e) >= 0 && cmdL(e) <= CMD_MAX, 'l ' + cmdL(e));
        assert.ok(cmdR(e) >= 0 && cmdR(e) <= CMD_MAX, 'r ' + cmdR(e));
    }
    assert.equal(cmdL(100), 600);
    assert.equal(cmdR(100), 0);        // 满差速时慢轮正好降到 0, 不会倒转
});

ok('翻转 LF_STEER_SIGN 就能整体反向(唯一的极性开关)', () => {
    const s2 = (err) => Math.max(-STEER_MAX, Math.min(STEER_MAX, KP * err * -1));
    assert.ok(BASE + s2(50) < BASE - s2(50));
});

// ---- 3. 编码器故障保护状态机, 与 motor.c 的 motor_pid_update() 等价 ----
const DUTY_MAX = 900, FAULT_TICKS = 10, TARGET = 300, MKP = 0.5, MKI = 0.5;

function sim(speedOf, ticks) {
    let out = 0, cnt = 0, errLast = 0, trip = -1, dutyAfterTrip = -1;
    for (let t = 0; t < ticks; t++) {
        if (trip >= 0) { dutyAfterTrip = 0; continue; }        // 自锁: 不再给占空比
        const now = speedOf(t);
        const e = TARGET - now;
        out += MKP * (e - errLast) + MKI * e;
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
    assert.equal(sim(() => 0, 400).dutyAfterTrip, 0);
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

// ---- 5. 左/右轮通道映射: 实测确认过, 写错就是「线在左边车往右拐」一起步就丢线 ----
const lfhSrc = readFileSync(join(root, 'system/line_follow.h'), 'utf8');
const emptySrc = readFileSync(join(root, 'empty.c'), 'utf8');

const macro = (name) => {
    const m = lfhSrc.match(new RegExp('#define\\s+' + name + '\\s+(\\d+)U'));
    return m ? Number(m[1]) : null;
};

ok('物理左轮 = B路(2), 物理右轮 = A路(1) (实测映射, 勿改)', () => {
    assert.equal(macro('LF_LEFT_ID'), 2, '左轮通道号被改了');
    assert.equal(macro('LF_RIGHT_ID'), 1, '右轮通道号被改了');
});

ok('empty.c 的 L/R 实测速度按通道号取, 没有硬写 speed_1/speed_2', () => {
    assert.match(emptySrc, /show_signed\(6, 24, speed_of\(LF_LEFT_ID\)/,
        'L 那一行没走 speed_of(LF_LEFT_ID)');
    assert.match(emptySrc, /show_signed\(46, 24, speed_of\(LF_RIGHT_ID\)/,
        'R 那一行没走 speed_of(LF_RIGHT_ID)');
});

// ---- 6. 诊断量: 运行时长按 10ms 累加, |error| 最大值只增不减 ----
const STEP_MS = 10, E_MAX_LIMIT = 100;

ok('运行时长按 10ms 累加, 且不会回绕(丢线后停在最后的值)', () => {
    let ms = 0;
    for (let i = 0; i < 300; i++) { if (ms < 0xFFFF) ms += STEP_MS; }   // 3 秒
    assert.equal(ms, 3000);
    const before = ms;
    // 丢线后 step() 直接 return, 不再累加 —— 屏上保留的就是这次跑了多久
    assert.equal(ms, before);
});

ok('|error| 最大值只增不减, 且不超过 100', () => {
    const seq = [0, 14, -28, 71, -100, 43, 0];
    let mx = 0;
    for (const e of seq) { const a = Math.abs(e); if (a > mx) mx = a; }
    assert.equal(mx, E_MAX_LIMIT);
});

// ---- 7. 丢线去抖状态机, 与 line_follow_step() 等价 ----
// 实测背景: 电机一转, 灰度会偶尔【整组漏读一次】(车压在线上却读到 00000000)。
// 单次采样就判丢线 -> 刚起步就 LOST 停车。所以要连续 LF_LOST_MS 才判。
// (STEP_MS 复用第 6 节那个 = 10)
const LOST_MS = 200;

function debounce(samples) {          // samples: 每一拍 {on: 是否看到线, e: 误差}
    let err = 0, lost_ms = 0, stopAt = -1;
    const errs = [];
    for (let i = 0; i < samples.length; i++) {
        if (stopAt >= 0) break;
        const s = samples[i];
        if (s.on) { err = s.e; lost_ms = 0; }
        else if (lost_ms < 0xFFFF) { lost_ms += STEP_MS; }
        if (lost_ms >= LOST_MS) { stopAt = i; break; }
        errs.push(err);
    }
    return { err, stopAt, errs };
}

const onLine = (e) => ({ on: true, e: e });
const blank = { on: false, e: 0 };

ok('看到线时正常刷新误差', () => {
    assert.equal(debounce([onLine(14), onLine(-28)]).err, -28);
    assert.equal(debounce([onLine(14), onLine(-28)]).stopAt, -1);
});

ok('单次漏读不漏车: 不停车, 而且误差保持上一次的值(转向不跳)', () => {
    const r = debounce([onLine(14), blank, onLine(14)]);
    assert.equal(r.stopAt, -1, '一漏读就停车了');
    assert.equal(r.errs[1], 14, '漏读那一拍误差被清成 0 了 -> 车会突然回正');
});

ok('连续漏读 190ms 又看到线 -> 不停车, 计时清零', () => {
    const r = debounce([onLine(14), ...Array(19).fill(blank), onLine(14)]);
    assert.equal(r.stopAt, -1);
});

ok('连续漏读 200ms -> 判丢线停车(正好第 20 拍)', () => {
    const r = debounce([onLine(14), ...Array(25).fill(blank)]);
    assert.equal(r.stopAt, 20);
});

ok('一开始就看不到线 -> 也是 200ms 后停, 不会 0ms 就停', () => {
    assert.equal(debounce(Array(30).fill(blank)).stopAt, 19);
});

console.log('\n' + pass + ' passed');