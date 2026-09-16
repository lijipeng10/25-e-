// 循迹自检（纯主机侧，不碰硬件）
// 跑法: node test/line_follow_ready_check.mjs
// ★ 控制参数直接从 system/line_follow.c 里读出来, 不在测试里另写一份 —— 免得两边漂开
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

let pass = 0;
const ok = (name, fn) => { fn(); pass++; console.log('  ok - ' + name); };

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const lfcSrc = readFileSync(join(root, 'system/line_follow.c'), 'utf8');
const lfhSrc = readFileSync(join(root, 'system/line_follow.h'), 'utf8');
const emptySrc = readFileSync(join(root, 'empty.c'), 'utf8');
const motorSrc = readFileSync(join(root, 'hardware/motor.c'), 'utf8');
const encSrc = readFileSync(join(root, 'hardware/encoder.h'), 'utf8');

// 从 .c/.h 里抠出 #define NAME <数字> 的第一个整数(不用正则, 免得转义踩坑)
const num = (src, name) => {
    const head = '#define ' + name + ' ';
    const line = src.split('\n').find((l) => l.indexOf(head) === 0);
    assert.ok(line, '没找到宏 ' + name);
    // 去掉括号和 U 后缀: 能吃下 (+1) / (-1) / 260U 这几种写法
    const tok = line.slice(head.length).trim().split(' ')[0].replace(/[()]/g, '').replace(/U$/, '');
    const v = parseInt(tok, 10);
    assert.ok(Number.isFinite(v), name + ' 后面不是数字: ' + JSON.stringify(tok));
    return v;
};

const BASE      = num(lfcSrc, 'LF_BASE_SPEED');
const KP        = num(lfcSrc, 'LF_STEER_KP');
const STEER_MAX = num(lfcSrc, 'LF_STEER_MAX');
const SLOW_KP   = num(lfcSrc, 'LF_SLOW_KP');
const BASE_MIN  = num(lfcSrc, 'LF_BASE_MIN');
const LOST_MS   = num(lfcSrc, 'LF_LOST_MS');
const STEP_MS   = num(lfcSrc, 'LF_STEP_MS');
const SIGN      = 1;                      // LF_STEER_SIGN = (+1)
const CMD_MAX   = BASE + STEER_MAX;

// ---- 1. 屏幕排版: 128x64, 12px 字体每位 6x12 ----
const CHAR_W = 6, CHAR_H = 12, SCREEN_W = 128, SCREEN_H = 64;

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
    const rows = [
        ['ERR-ENCODER', 0], ['MPU:NO K2GO', 0], ['STOP  K2=GO', 0],
        ['LOST ', 6], ['RUN  ', 6],
    ];
    for (const [text, digits] of rows) assert.equal(text.length + digits, 11, JSON.stringify(text));
});

// ---- 2. 控制律: steer = KP*error, base = BASE - SLOW_KP*|error|(不低于 BASE_MIN) ----
const steer = (err) => Math.max(-STEER_MAX, Math.min(STEER_MAX, KP * err * SIGN));
const baseOf = (err) => Math.max(BASE_MIN, BASE - SLOW_KP * Math.abs(err));
const cmdL = (err) => Math.max(0, Math.min(CMD_MAX, baseOf(err) + steer(err)));
const cmdR = (err) => Math.max(0, Math.min(CMD_MAX, baseOf(err) - steer(err)));

ok('线偏右(E>0) -> 左轮命令比右轮大(往右转)', () => {
    assert.ok(cmdL(50) > cmdR(50));
});

ok('线偏左(E<0) -> 左轮命令比右轮小(往左转)', () => {
    assert.ok(cmdL(-50) < cmdR(-50));
});

ok('线在正中 -> 两轮都是基础速度', () => {
    assert.equal(cmdL(0), BASE);
    assert.equal(cmdR(0), BASE);
});

ok('命令永远夹在 [0, CMD_MAX]: 不给速度环负目标(编码器认不出方向)', () => {
    for (let e = -100; e <= 100; e++) {
        assert.ok(cmdL(e) >= 0 && cmdL(e) <= CMD_MAX, 'l ' + cmdL(e));
        assert.ok(cmdR(e) >= 0 && cmdR(e) <= CMD_MAX, 'r ' + cmdR(e));
    }
});

ok('满误差时快轮全速、慢轮停住(转弯半径最小)', () => {
    assert.equal(cmdL(100), baseOf(100) + steer(100));
    assert.equal(cmdR(100), 0);          // 慢轮命令被夹到 0, 不倒转
});

// ---- 3. 转弯减速: |error| 越大, 基础速度越低, 但不低于 BASE_MIN ----
ok('线偏出去时基础速度自动降低', () => {
    assert.ok(baseOf(0) > baseOf(50), '没有减速');
    assert.ok(baseOf(50) > baseOf(100));
});

ok('基础速度永远不低于 BASE_MIN(弯道再慢也不能停, 停了就转不动)', () => {
    for (let e = -100; e <= 100; e++) assert.ok(baseOf(e) >= BASE_MIN, 'e=' + e);
    assert.equal(baseOf(10000), BASE_MIN, '误差再大也应该停在 BASE_MIN');
});

ok('翻转 LF_STEER_SIGN 就能整体反向(唯一的极性开关)', () => {
    const s2 = (err) => Math.max(-STEER_MAX, Math.min(STEER_MAX, KP * err * -1));
    assert.ok(BASE + s2(50) < BASE - s2(50));
});

// ---- 4. 编码器故障保护状态机, 与 motor.c 的 motor_pid_update() 等价 ----
const DUTY_MAX = 900, FAULT_TICKS = 10, TARGET = 300, MKP = 0.5, MKI = 0.5;

function sim(speedOf, ticks) {
    let out = 0, cnt = 0, errLast = 0, trip = -1, dutyAfterTrip = -1;
    for (let t = 0; t < ticks; t++) {
        if (trip >= 0) { dutyAfterTrip = 0; continue; }
        const now = speedOf(t);
        const e = TARGET - now;
        out += MKP * (e - errLast) + MKI * e;
        errLast = e;
        out = Math.min(DUTY_MAX, Math.max(0, out));
        if (out >= DUTY_MAX - 1 && now < 1) { if (++cnt >= FAULT_TICKS) trip = t; }
        else { cnt = 0; }
    }
    return { trip, out, dutyAfterTrip };
}

ok('编码器正常 -> 永不误判', () => {
    assert.equal(sim((t) => (t < 1 ? 0 : 300), 400).trip, -1);
    assert.equal(sim((t) => (t < 5 ? 0 : 280), 400).trip, -1);
});

ok('编码器没接(顶死不动) -> 判故障并自锁', () => {
    const r = sim(() => 0, 400);
    assert.ok(r.trip >= 0 && r.trip <= 20, '第 ' + r.trip + ' 拍');
    assert.equal(r.dutyAfterTrip, 0);
});

// ---- 5. 硬件实测事实(勿凭猜改) ----
const forwardPins = (id) => {
    const fnSrc = motorSrc.split('void motor_set_direction')[1].split('void motor_set_duty')[0];
    const seg = fnSrc.split('if(id == ' + id + ')')[1];
    const dir1 = seg.split('if(direction == 1)')[1].split('else if')[0];
    const out = [];
    for (const line of dir1.split('\n')) {
        const which = line.indexOf('IN1_') >= 0 ? '1' : (line.indexOf('IN2_') >= 0 ? '2' : '');
        if (which === '') continue;
        if (line.indexOf('DL_GPIO_setPins') >= 0) out.push('set' + which);
        if (line.indexOf('DL_GPIO_clearPins') >= 0) out.push('clear' + which);
    }
    return out.join(',');
};

ok('两个电机 direction=1 都是 拉高 IN1 + 拉低 IN2 (实测极性, 勿改)', () => {
    assert.equal(forwardPins(1), 'set1,clear2', 'A路(左轮)极性被改了');
    assert.equal(forwardPins(2), 'set1,clear2', 'B路(右轮)极性被改了');
});

ok('物理左轮 = B路(2), 物理右轮 = A路(1) (实测映射, 勿改)', () => {
    assert.equal(num(lfhSrc, 'LF_LEFT_ID'), 2, '左轮通道号被改了');
    assert.equal(num(lfhSrc, 'LF_RIGHT_ID'), 1, '右轮通道号被改了');
});

ok('empty.c 的 L/R 实测速度按通道号取, 没有硬写 speed_1/speed_2', () => {
    assert.ok(emptySrc.indexOf('show_signed(6, 24, speed_of(LF_LEFT_ID)') >= 0, 'L 那一行');
    assert.ok(emptySrc.indexOf('show_signed(46, 24, speed_of(LF_RIGHT_ID)') >= 0, 'R 那一行');
});

// ---- 6. 丢线去抖状态机, 与 line_follow_step() 等价 ----
// 实测背景: 电机一转, 灰度会偶尔【整组漏读一次】(车压在线上却读到 00000000)。
// 单次采样就判丢线 -> 刚起步就 LOST 停车。所以要连续 LF_LOST_MS 才判。
function debounce(samples) {
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
});

ok('单次漏读不漏车: 不停车, 误差保持上一次的值(转向不跳)', () => {
    const r = debounce([onLine(14), blank, onLine(14)]);
    assert.equal(r.stopAt, -1, '一漏读就停车了');
    assert.equal(r.errs[1], 14, '漏读那一拍误差被清成 0 了 -> 车会突然回正');
});

ok('连续漏读 ' + (LOST_MS - STEP_MS) + 'ms 又看到线 -> 不停车, 计时清零', () => {
    const n = LOST_MS / STEP_MS - 1;
    assert.equal(debounce([onLine(14), ...Array(n).fill(blank), onLine(14)]).stopAt, -1);
});

ok('连续漏读 ' + LOST_MS + 'ms -> 判丢线停车(正好第 ' + (LOST_MS / STEP_MS) + ' 拍)', () => {
    assert.equal(debounce([onLine(14), ...Array(30).fill(blank)]).stopAt, LOST_MS / STEP_MS);
});

ok('一开始就看不到线 -> 也是 ' + LOST_MS + 'ms 后停, 不会 0ms 就停', () => {
    assert.equal(debounce(Array(30).fill(blank)).stopAt, LOST_MS / STEP_MS - 1);
});

// ---- 7. 速度环的「方向 + 幅值」拆分, 与 motor.c 的 motor_pid_update() 等价 ----
// 编码器现在能测出正反转(在 A 相跳变时读 B 相电平), 所以目标速度可以带符号。
// 方向由【目标的符号】决定, PID 只调【幅值】: 反馈取绝对值, 正负不会打架。
function pidStep(target, now, st) {
    const targetMag = target >= 0 ? target : -target;
    const nowMag = now >= 0 ? now : -now;
    const e = targetMag - nowMag;
    st.out += 0.5 * (e - st.eLast) + 0.5 * e;
    st.eLast = e;
    st.out = Math.min(900, Math.max(0, st.out));
    return { dir: target >= 0 ? 1 : 2, duty: Math.round(st.out) };
}

ok('目标为正 -> 方向脚 = 1(前进); 目标为负 -> 方向脚 = 2(后退)', () => {
    assert.equal(pidStep(300, 300, { out: 400, eLast: 0 }).dir, 1);
    assert.equal(pidStep(-300, -300, { out: 400, eLast: 0 }).dir, 2);
});

ok('倒转时反馈取绝对值: 转得不够快, 占空比应该继续加大', () => {
    const r = pidStep(-300, -100, { out: 300, eLast: 0 });
    assert.ok(r.duty > 300, '倒转时占空比没加大: ' + r.duty);
});

ok('要倒退但轮子还在正转: 幅值一样 -> 占空比不动, 靠方向脚把它拽回来', () => {
    const r = pidStep(-300, 300, { out: 300, eLast: 0 });
    assert.equal(r.dir, 2, '方向脚没翻成后退');
    assert.equal(r.duty, 300, '幅值相同时占空比不该变: ' + r.duty);
});

ok('目标为正时和以前算法完全一致(前进调参结果不受影响)', () => {
    const st1 = { out: 400, eLast: 0 }, st2 = { out: 400, eLast: 0 };
    for (const now of [0, 100, 300, 500]) {
        const a = pidStep(300, now, st1);                 // 新的(方向+幅值)
        const e = 300 - now;                              // 旧的(直接相减)
        st2.out += 0.5 * (e - st2.eLast) + 0.5 * e;
        st2.eLast = e;
        st2.out = Math.min(900, Math.max(0, st2.out));
        assert.equal(a.duty, Math.round(st2.out), 'now=' + now);
    }
});

// ---- 8. 编码器方向符号: 手转轮子实测出来的, 勿凭猜改 ----
// 实测: 物理左轮(B路/E2A) 正转显示正; 物理右轮(A路/E1A) 正转显示负, 所以要翻。
ok('编码器方向符号是实测确认过的(两个轮子符号不同是正常的)', () => {
    assert.equal(num(encSrc, 'ENCODER_1_SIGN'), -1, '物理右轮(A路/E1A)的符号被改了');
    assert.equal(num(encSrc, 'ENCODER_2_SIGN'), 1, '物理左轮(B路/E2A)的符号被改了');
});

ok('每圈脉冲数 = 260(用户实测; 双边沿才是 520, 现在 SysConfig 是单边沿)', () => {
    assert.equal(num(encSrc, 'ENCODER_PULSE'), 260, 'ENCODER_PULSE 被改了');
});

// ---- 9. 陀螺仪阻尼项: steer = KP*error + KD*角速度 ----
// 符号推导: 陀螺仪【左转为正】(实测) -> 车正在往左偏 -> 要压它往右 -> steer 取正;
//           而 steer > 0 = 左轮快 = 往右转, 所以阻尼项是【加】。
const KD = num(lfcSrc, 'LF_GYRO_KD');

const dampOf = (rateX10) => Math.trunc((KD * rateX10) / 10);      // 0.1度/秒 -> mm/s

ok('往左转(角速度为正) -> 阻尼项为正(往右拦)', () => {
    assert.ok(dampOf(+500) > 0, 'rate=+50度/秒 时阻尼项不是正的: ' + dampOf(+500));
});

ok('往右转(角速度为负) -> 阻尼项为负(往左拦)', () => {
    assert.ok(dampOf(-500) < 0, 'rate=-50度/秒 时阻尼项不是负的: ' + dampOf(-500));
});

ok('不转的时候阻尼项为 0(不影响静态核对 l/r)', () => {
    assert.equal(dampOf(0), 0);
});

ok('阻尼项不会把差速撑爆(和位置项相加后仍会被 STEER_MAX 夹住)', () => {
    const raw = KP * 100 * SIGN + dampOf(3000);       // 位置项满 + 300度/秒
    const clamped = Math.max(-STEER_MAX, Math.min(STEER_MAX, raw));
    assert.equal(clamped, STEER_MAX);
});

ok('LF_GYRO_KD 是可关的: 设成 0 就退回纯位置控制', () => {
    assert.ok(KD >= 0, 'KD 不该是负数(负数=陀螺仪左右符号反了才会用)');
});

console.log('\n' + pass + ' passed');
