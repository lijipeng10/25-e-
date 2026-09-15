/* ============================================================================
 *  pivot_sim.mjs —— 弯道"停车原地转向"状态机的宿主仿真(回归测试)
 * ----------------------------------------------------------------------------
 *  【为什么会有这个文件】
 *    原地转向这段逻辑在板子上非常难调: 车在动、看不清、跑一次要很久。
 *    而它偏偏是踩坑最多的: "转个不停"、"锁到【来路】那条线然后逆行"、
 *    "直道上也会掉头"。所以把状态机搬到电脑上跑, 几毫秒验证一遍。
 *
 *  【"来路"那条线在哪个角度 —— 这决定了要不要加角度门】
 *    原地转向只有两个触发口:
 *      (a) 急弯判据 |error| >= 71 连续 60ms -> 那时线在【阵列边缘】(约 22 度)
 *      (b) 连续丢线 150ms                    -> 那时【什么都看不见】
 *    两种情况下来路都不可能【正对着车头】; 它要等车头转过近半圈才重新进视野。
 *      -> "认线至少转过 N 度"这种门是多余的: 直道上误进原地转向时, 正确的线
 *         就在原地, 加了门就永远认不回来 -> 实车"直线都会掉头"。
 *      -> 真正防逆行的是"最多转多少度"(MAX)。
 *
 *  【★ 一处已知的残余风险(故意不判失败, 见下面的"残余风险"一节)】
 *    如果方向【判反】了, 而老线恰好就在判反那一侧的阵列边缘上, 车会转过去
 *    把老线认下来 —— 这时它只转了十几度。而"直道误触发"时车也是在小角度上
 *    认线。两者在状态机眼里【无法区分】, 所以没有一个 MIN 值能同时满足两边。
 *    取舍依据是频率: 直道误触发常见, 方向判反罕见(方向证据累积就是为它做的)。
 *
 *  【三组用例】
 *    正向     : 必须全过。
 *    阴性对照A: 把最小角度门加回去 -> 直道那一类必须失败(实车"直线都会掉头")。
 *    阴性对照B: 把最大角度门拿掉   -> 线找不到时必须锁到来路(逆行)。
 *
 *  【不验证什么】真实赛道几何 / 传感器张角 / 转速; 以及 C 代码本身 ——
 *    这里是照着 C 重写的一份模型, 改了 C 必须同步改这里。
 *
 *  用法:  node test/pivot_sim.mjs
 * ==========================================================================*/
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, join } from "node:path";

const here = dirname(fileURLToPath(import.meta.url));
const SRC = join(here, "..", "system", "line_follow.c");
const src = readFileSync(SRC, "utf8");

function macro(name) {
  const m = src.match(new RegExp("^\\s*#define\\s+" + name + "\\s+(\\d+)", "m"));
  if (m === null) { throw new Error("在 " + SRC + " 里找不到 #define " + name); }
  return Number(m[1]);
}

const MAX_DEG = macro("LF_PIVOT_MAX_DEG");
const PIVOT_OK = macro("LF_PIVOT_OK");
const SOURCE_HAS_MIN = /#define\s+LF_PIVOT_MIN_DEG/.test(src);

const ARRAY_HALF = 220;                 // 阵列半张角(0.1度), 按 ±22 度估
const SCALE      = 100 / ARRAY_HALF;
const STEP       = 30;                  // 10ms 转 3 度(原地转向约 300 度/秒)
const SAFE_YAW   = 1800 - ARRAY_HALF;   // 超过它就可能看见"来路"

const errOf = (yaw, Y) => (Y === null || Math.abs(yaw - Y) > ARRAY_HALF ? null : (yaw - Y) * SCALE);

function oldYawFor(mode, turn) {
  if (mode === "edge")   return -Math.sign(turn) * ARRAY_HALF;
  if (mode === "behind") return 1800 * -Math.sign(turn);
  return null;
}

function run(dir0, Ystar, oldY, maxDeg, minDeg) {
  let yaw = 0, dir = dir0, attempt = 0, ms = 0, maxAbs = 0;
  const ev = [];
  for (let i = 0; i < 900; i++) {
    ms += 10;
    maxAbs = Math.max(maxAbs, Math.abs(yaw));
    const turned = -yaw * dir;          // ★ 符号: lf_pivot(+1)=右转, yaw>0=左转, 反号

    let err = errOf(yaw, Ystar), which = "新线";
    if (err === null) { const oe = errOf(yaw, oldY); if (oe !== null) { err = oe; which = "老线(来路)"; } }

    if (err !== null && err <= PIVOT_OK && err >= -PIVOT_OK && turned >= minDeg)
      return { accepted: true, which, yaw, maxAbs, ev };
    if (turned >= maxDeg || ms >= 2500) {
      if (attempt === 0) { attempt = 1; ms = 0; dir = -dir; ev.push("掉头@" + yaw); }
      else return { accepted: false, which: "放弃", yaw, maxAbs, ev };
    }
    yaw += -dir * STEP;
  }
  return { accepted: false, which: "超时", yaw, maxAbs, ev };
}

const good = (r) => r.accepted && r.which === "新线" && r.maxAbs < SAFE_YAW;
let bad = 0, ok = 0, total = 0;

const TRACK = [
  { name: "C/D 角 76 度 ", turn:  760, dirIfRight: -1 },
  { name: "A/B 角 105 度", turn: -1050, dirIfRight: +1 },
];

console.log("参数(从 line_follow.c 解析): MAX=" + MAX_DEG + " OK=" + PIVOT_OK +
            "  MIN=" + (SOURCE_HAS_MIN ? "还在(不该!)" : "已删除"));
console.log("安全线: |yaw| 必须 < " + SAFE_YAW + "\n");

console.log("=== 正向 1: 弯道, 方向判对(必须全过) ===");
for (const t of TRACK) for (const m of ["edge", "behind"]) {
  total++;
  const r = run(t.dirIfRight, t.turn, oldYawFor(m, t.turn), MAX_DEG, 0);
  const g = good(r); if (g) ok++; else bad++;
  console.log((g ? "PASS  " : "FAIL  ") + t.name + " 来路" + (m === "edge" ? "在边缘" : "在半圈外") +
    " -> 停在[" + r.which + "] yaw=" + r.yaw + " 最大|yaw|=" + r.maxAbs);
}

console.log("\n=== 正向 2: 弯道, 方向判反(来路在半圈外, 必须能掉头找回新线) ===");
for (const t of TRACK) {
  total++;
  const r = run(-t.dirIfRight, t.turn, oldYawFor("behind", t.turn), MAX_DEG, 0);
  const g = good(r); if (g) ok++; else bad++;
  console.log((g ? "PASS  " : "FAIL  ") + t.name + " -> 停在[" + r.which + "] yaw=" + r.yaw +
    " 最大|yaw|=" + r.maxAbs + (r.ev.length ? " " + r.ev.join(" ") : ""));
}

console.log("\n=== 正向 3: 直道上误进原地转向(必须【立刻】认回来) ===");
{
  total++;
  const r = run(-1, 0, null, MAX_DEG, 0);
  const g = r.accepted && Math.abs(r.yaw) <= 200;
  if (g) ok++; else bad++;
  console.log((g ? "PASS  " : "FAIL  ") + "停在[" + r.which + "] yaw=" + r.yaw +
    "   (必须≈0; 转出去就成'直线掉头'了)");
}

console.log("\n=== 阴性对照 A: 最小角度门加回来(40度), 直道那一类必须【失败】===");
{
  total++;
  const r = run(-1, 0, null, MAX_DEG, 400);
  const fail = !(r.accepted && Math.abs(r.yaw) <= 200);
  if (fail) ok++; else bad++;
  console.log((fail ? "PASS  " : "FAIL  ") + "停在[" + r.which + "] yaw=" + r.yaw +
    (r.ev.length ? " " + r.ev.join(" ") : "") +
    "  <- " + (fail ? "如期转出去认不回来, 正是实车'直线掉头'" : "对照失效!"));
}

console.log("\n=== 阴性对照 B: 最大角度门拿掉, 新线找不到时必须【锁到来路】(逆行) ===");
{
  total++;
  const r = run(-1, null, oldYawFor("behind", -1050), 100000, 0);
  const latched = r.accepted && r.which !== "新线";
  if (latched) ok++; else bad++;
  console.log((latched ? "PASS  " : "FAIL  ") + "停在[" + r.which + "] yaw=" + r.yaw +
    "  <- " + (latched ? "如期绕半圈锁到来路, 所以 MAX 不能删" : "对照失效!"));
}

/* 已知残余风险: 只打印, 不判失败 —— 原因见文件头 */
console.log("\n=== 已知残余风险(不计入通过率) ===");
for (const t of TRACK) {
  const r = run(-t.dirIfRight, t.turn, oldYawFor("edge", t.turn), MAX_DEG, 0);
  console.log("  " + t.name + " 方向判反 + 来路在边缘 -> 停在[" + r.which + "] yaw=" + r.yaw +
    "  <- " + (r.which === "新线" ? "没事" : "会把老线认下来(与'直道误触发'无法区分)"));
}

console.log("\n" + ok + "/" + total + " 通过" + (bad === 0 ? " ✓" : " ✗ " + bad + " 项失败"));
if (bad !== 0) { process.exitCode = 1; }
