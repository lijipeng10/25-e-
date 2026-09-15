/* ============================================================================
 *  pivot_sim.mjs —— 弯道"停车原地转向"状态机的宿主仿真(回归测试)
 * ----------------------------------------------------------------------------
 *  【为什么会有这个文件】
 *    原地转向这段逻辑在板子上非常难调: 车在动、看不清、跑一次要很久。
 *    而它偏偏是踩坑最多的: "转个不停"、"锁到【来路】那条线然后原路逆行"。
 *    所以把状态机搬到电脑上跑: 纯逻辑 + 简化几何, 几毫秒验证一遍。
 *
 *  【它验证什么 / 不验证什么】★ 说清楚, 免得误用
 *    验证: 这段【设计】的逻辑与符号 —— 角度门的位置对不对、
 *          turned 的符号(-yaw*dir)对不对、掉头之后还收不收敛。
 *    验证: LF_PIVOT_MIN_DEG / LF_PIVOT_MAX_DEG / LF_PIVOT_OK 三个常数
 *          在【简化几何】下是否够用(直接从 line_follow.c 解析, 不会各写一份)。
 *    不验证: 真实赛道几何、真实传感器张角、真实转速 —— 这些只能上赛道量。
 *    不验证: C 代码本身。这里的状态机是【照着 C 重写的一份模型】,
 *            所以改了 C 里的判定顺序或符号, 必须同步改这里, 否则测试会假通过。
 *
 *  【阴性对照】结论来自实测的仿真输出, 不是猜的:
 *    把角度门关掉之后, 只有"来路在跟前"那两种情况会出问题 ——
 *    车刚起转就锁上来路, 停在 yaw=0(等于原地没动)然后沿来路反着走。
 *    而"来路在身后"那两种【不需要门也是安全的】, 因为车会先遇到新线。
 *    所以对照只针对"来路在跟前", 这也正是实车上看到的那个故障。
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

const MIN_DEG  = macro("LF_PIVOT_MIN_DEG");
const MAX_DEG  = macro("LF_PIVOT_MAX_DEG");
const PIVOT_OK = macro("LF_PIVOT_OK");

/* ---- 简化几何 -------------------------------------------------------------
 *   yaw : 相对"进弯那一刻"车头转过的角度(0.1度, 左正)
 *   Y*  : 车头朝向它时, 线正好在阵列中间
 *   新线  = 转弯角那么大; 来路(走过来的那条线)要么在 yaw≈0 附近
 *          (传感器还压在来路上时触发的急弯判据), 要么要转到 ±1800 才重新出现。 */
const ARRAY_HALF = 220;                 // 阵列半张角(0.1度), 按 ±22 度估
const SCALE      = 100 / ARRAY_HALF;    // 偏差 -> error(-100..+100)
const STEP       = 30;                  // 10ms 转 3 度(原地转向约 300 度/秒)
const SAFE_YAW   = 1800 - ARRAY_HALF;   // 超过它就可能看见来路

const errOf  = (yaw, Y) => (Math.abs(yaw - Y) > ARRAY_HALF ? null : (yaw - Y) * SCALE);
const oldErr = (yaw, mode) =>
  mode === "near0"  ? errOf(yaw, 0)
: mode === "behind" ? (errOf(yaw, 1800) ?? errOf(yaw, -1800))
: null;

/* ---- 被仿真的状态机(对应 line_follow_step 里"状态 A") ------------------- */
function run(dir0, Ystar, oldMode, minDeg, maxDeg) {
  let yaw = 0, dir = dir0, attempt = 0, ms = 0, maxAbs = 0;
  const ev = [];
  for (let i = 0; i < 800; i++) {
    ms += 10;
    maxAbs = Math.max(maxAbs, Math.abs(yaw));
    const turned = -yaw * dir;           // ★ 符号: lf_pivot(+1)=右转, yaw>0=左转, 两者反号

    let err = errOf(yaw, Ystar), which = "新线";
    if (err === null) {
      const oe = oldErr(yaw, oldMode);
      if (oe !== null) { err = oe; which = "老线(来路)"; }
    }
    if (err !== null && err <= PIVOT_OK && err >= -PIVOT_OK && turned >= minDeg) {
      return { accepted: true, which, yaw, maxAbs, ev };
    }
    if (turned >= maxDeg || ms >= 2500) {
      if (attempt === 0) { attempt = 1; ms = 0; dir = -dir; ev.push("掉头@" + yaw); }
      else { return { accepted: false, which: "放弃", yaw, maxAbs, ev }; }
    }
    yaw += -dir * STEP;
  }
  return { accepted: false, which: "超时", yaw, maxAbs, ev };
}

const good = (r) => r.accepted && r.which === "新线" && r.maxAbs < SAFE_YAW;

/* ---- 用例: H 题梯形赛道, C/D 角要转 76 度, A/B 角要转 105 度 -------------- */
const TRACK = [
  { name: "C/D 角 76 度 ", turn:  760, dirIfRight: -1 },
  { name: "A/B 角 105 度", turn: -1050, dirIfRight: +1 },
];

console.log("参数(从 line_follow.c 解析): MIN=" + MIN_DEG + " MAX=" + MAX_DEG + " OK=" + PIVOT_OK);
console.log("安全线: |yaw| 必须 < " + SAFE_YAW + " (超过就可能看见来路)\n");

let bad = 0;

console.log("=== 正向用例: 必须【全部通过】===");
let nPass = 0, nTotal = 0;
for (const t of TRACK) {
  for (const okDir of [true, false]) {
    for (const oldMode of ["near0", "behind"]) {
      nTotal++;
      const dir = okDir ? t.dirIfRight : -t.dirIfRight;
      const r = run(dir, t.turn, oldMode, MIN_DEG, MAX_DEG);
      const ok = good(r);
      if (ok) { nPass++; } else { bad++; }
      console.log((ok ? "PASS  " : "FAIL  ") + t.name +
        (okDir ? " 方向判对" : " 方向判反") + " 来路" + (oldMode === "near0" ? "在跟前" : "在身后") +
        "  -> 停在[" + r.which + "] yaw=" + r.yaw + " 最大|yaw|=" + r.maxAbs +
        (r.ev.length ? " " + r.ev.join(" ") : ""));
    }
  }
}
console.log("       " + nPass + "/" + nTotal + " 通过");

/* ---- 阴性对照: 关掉角度门, "来路在跟前"【必须失败】-----------------------
 * 证明测试有效: 没有角度门时, 车刚起转就锁上来路, 停在 yaw=0(等于没转),
 * 然后沿着来路反着走 —— 正是实车上观察到的"转回来...逆行"。 */
console.log("\n=== 阴性对照: 关掉角度门(来路在跟前), 必须【全部失败】===");
let cOk = 0, cTotal = 0;
for (const t of TRACK) {
  cTotal++;
  const r = run(t.dirIfRight, t.turn, "near0", 0, 100000);
  const expectedFail = !good(r);
  if (expectedFail) { cOk++; } else { bad++; }
  console.log((expectedFail ? "PASS  " : "FAIL  ") + t.name + " 无角度门  -> 停在[" + r.which +
    "] yaw=" + r.yaw + "  " + (expectedFail ? "(如期锁到来路)" : "(竟然还是对的, 对照失效!)"));
}
console.log("       " + cOk + "/" + cTotal + " 如期失败");

console.log("\n" + (bad === 0 ? "全部符合预期 ✓" : bad + " 项不符合预期 ✗"));
if (bad !== 0) { process.exitCode = 1; }
