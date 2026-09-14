# DEBUG.md —— 串口调试与故障快速定位

> **出问题先翻这个文件。** 所有判断只依赖**串口日志**，不需要示波器。
> 相关文档：PROJECT.md（工程全貌 / 引脚表 / 全部踩坑记录）、AGENTS.md（开发约定）

---

## 0. 怎么连串口

| 项目 | 值 |
| --- | --- |
| 外设 | **UART2** |
| TX（单片机发出来） | **PB15** |
| RX（收进单片机） | PB16 |
| 参数 | **115200 8N1**，无流控 |

接上 USB-TTL，打开串口终端，按一下复位键。

---

## 1. 正常的启动日志长这样

```
=== BOOT ===
[1] SYSCFG_DL_init OK  (clock init did NOT hang)
[2] tick/key/motor/grayscale/OLED OK
[3] line_follow OK -> entering main loop

=== LINE FOLLOW ===
[format] S=8bits(1=on line)  E=-100..100  L/R=duty%  HB=alive
S=00000000 E=+000 L=000 R=000
S=00000000 E=+000 L=000 R=000
HB
S=00000000 E=+000 L=000 R=000
...
```

**不管 `DBG_UART` 是 0 还是 1，上面这几行一定会打印。**
它是排障的生命线，不会因为要跑车就被关掉。
`DBG_UART` 只管周期性的 `S=` 数据行和 `HB` 心跳 —— 那两样才占主循环时间。

---

## 2. 每行日志什么意思

| 日志 | 含义 |
| --- | --- |
| `=== BOOT ===` | 启动开始 |
| `[1] SYSCFG_DL_init OK` | **时钟 / SysConfig 初始化跑完了**。这行出来就说明没卡在开机时钟里 |
| `[2] ...OK` | `tick_init / key_init / motor_init / Grayscale_Sensor_Init / OLED_Init` 都过了 |
| `[3] ...entering main loop` | 初始化全部完成，马上进主循环 |
| `S=00011000` | 8 路灰度位图，**从左到右**，`1` = 压到黑线 |
| `E=-014` | 线偏差，**负 = 线在左，正 = 线在右**，范围 ±100 |
| `L=040 R=040` | 左轮 / 右轮**实际输出**的占空比 % |
| `HB` | 心跳，约 0.4 秒一次。**只看循环圈数，不看时间** —— 所以就算 SysTick 死了它照样打 |
| `[KEY1] follow ON/OFF` | 按了 KEY1 |
| `[KEY2] motor TEST on/off` | 按了 KEY2 |
| `!!! HARDFAULT !!!` / `!!! NMI !!!` | **CPU 跑飞了**，同时 OLED 定格在 `!! FAULT !!` |

> ⚠️ `[format]` 那行是**格式说明**，不是数据。
> 以前这里打过两行写死的样板数据（`S=00011000 ...`），跟真实数据长得一模一样，
> 排查时被误导过好几次，已经删掉 —— **以后不要往这里加「示例数据」**。

---

## 3. 故障定位决策树

### 3.1 上电后串口一个字都没有

| 先确认 | 怎么确认 |
| --- | --- |
| 串口参数对不对 | 115200 8N1、TX 接 PB15、GND 共地 |
| 芯片到底跑没跑 | **看 OLED**：OLED 也全黑 → 大概率是同一个原因 |

**如果 OLED 全黑 + 串口也没有 `[1]`** → **卡在 `SYSCFG_DL_init()` 里**。
嫌疑最大的是时钟初始化，原因见 4.4 节。

### 3.2 跑着跑着不动了（「卡住 / 复位键没用」）

按复位后看串口：

| 日志表现 | 结论 | 下一步 |
| --- | --- | --- |
| 开头**又出现** `=== BOOT ===` | MCU **复位了**（不是卡死） | 查什么导致复位：供电跌落 / 电机干扰 / 看门狗 |
| 日志**戛然而止**，`HB` 也停 | **CPU 真卡住了** | 看前面有没有 `!!! HARDFAULT !!!`。有 → 跑飞；没有 → 卡在某个外设等待循环里 |
| 日志停但 **`HB` 还在打** | CPU 活着，**SysTick 停了** 或某个 `(now - last) >= xx` 判断出问题 | 查时钟 / `system/tick.c` |
| OLED 定格 `!! FAULT !!` | HardFault 或 NMI | 用调试器暂停，看 PC 停在哪 |
| `[1]` 有，`[2]` / `[3]` 没有 | 卡在 `tick / key / motor / grayscale / OLED` 的初始化里 | 逐个注释掉定位 |

### 3.3 编译不过 / 改完代码好像没生效

```powershell
& 'D:/TI/ccs2025/ccs/utils/bin/gmake.exe' -C Debug -j4 all
```

验收标准：**exit 0，且 0 error 0 warning**。只看 `Finished building target` 不够。

> **一个极其隐蔽的坑**：改 `empty.syscfg` 之后如果 SysConfig 生成失败，
> 编译会**中途停住**，你以为是代码问题，实际上**板子上跑的还是旧固件**，
> 于是表现为「改了没反应」。改完 `.syscfg` 一定要确认 `Debug/ti_msp_dl_config.c` 是新的。
>
> 好消息：`gmake` **会自动调用与工程版本匹配的 SysConfig**（`C:/TI/sysconfig_1.26.2/`），
> 不需要去点图形界面。

### 3.4 传感器 `S=` 不对

| 现象 | 结论 |
| --- | --- |
| 白底上 `S=00000000` | **正常**。`LF_LINE_LEVEL = 1U` 表示「压黑线 = 1」，白底当然全是 0 |
| 黑线压住某一路，那一位还是 0 | 那一路没读到：传感器离地太高，或该通道坏了 |
| 一直是 `S=11111111` | 极性反了 → 把 `LF_LINE_LEVEL` 改成 `0U`；或者是全黑背景 |
| 一直是 `S=00000000`，压黑线也没反应 | 见下面的**三步自检** |

**三步自检（按顺序做，能精确定位到哪一段坏了）：**

1. **拿个黑色物体从传感器下面慢慢划过**，盯 `S=`
   那个 `1` 应该**跟着手一路走过去**。
   这一步同时验证了 **OUT 读取** 和 **通道选择（AD0/AD1/AD2）**，是最可靠的一步。
2. **把 PA22（OUT）从模块上拔下来，手动短接到 3.3V / GND**
   - 短到 3.3V → `S` 应变成 `11111111`；短到 GND → `00000000`
   - 有反应 → **单片机这侧是好的**，问题在模块 / 供电 / 接线
   - 没反应 → 问题在单片机引脚配置
3. **单独查模块**：供电 VCC/GND、模块上的指示灯会不会随黑白变化

> ⚠️ **不要用打印 AD0/AD1/AD2 电平的办法来判断通道选择。**
> 那个读数是假的，原因见 4.3 节。

### 3.5 电机 / 循迹

| 现象 | 原因 | 改哪 |
| --- | --- | --- |
| 车不动 | 基础速度低于电机启动死区 | `LF_BASE_DUTY` 调大，**同时 `LF_MAX_STEER` 要 >= 它** |
| **速度调低反而左右摆得更凶** | 占空比贴着死区，一点差速就把某侧推到不转 | `LF_BASE_DUTY` 调**大** |
| 直线上小幅左右摆 | 数字量传感器的**量化台阶**（误差最小跳变 14） | `LF_DEADBAND`（现在 15） |
| 转弯冲过弯道 | 慢的一侧降不到 0，速度差上不去 | `LF_MAX_STEER` 必须 **>= `LF_BASE_DUTY`** |
| 到弯道不知道往哪转 / 原地转圈 | 方向证据不足 | 现在有**双向搜索**兜底，最多转 900ms 会自己掉头；调 `LF_PIVOT_*` |
| **数值越大越慢** | 占空比换算写反了（见 4.1） | `motor_duty_to_cmp()` |
| 按 KEY1 车原地扭两下就停 | **正常**：白底找不到线 → 判定到「弯节点」→ 转向也找不到 → 停车 | 把车放到线上 |

---

## 4. 已经踩过的坑（含根因，别再踩第二次）

### 4.1 电机 PWM 占空比换算是「反」的

MSPM0 的 TimerG 在 EDGE_ALIGN PWM 模式下**从 LOAD 往下数**，所以：

```
实际占空比 = (周期 - 比较值) / 周期      <- 比较值越大, 占空比越小!
```

TI 自己的 SysConfig 就是这么算的，见
`source/ti/driverlib/.meta/pwm/PWMTimerCC.syscfg.js` 第 107 行：

```js
proposedccValue = Math.round( (100 - inst.dutyCycle) * (period) / 100) - 1;
```

**坑**：`hardware/motor.c` 原来写成 `cmp = duty * PERIOD / 100`，正好反了。

| 填的值 | 实际输出 |
| --- | --- |
| 20 | 80% |
| 40 | 60% |
| **50** | **50%（唯一对称点，所以「填 50 看着是对的」，极易漏过去）** |
| 90 | 10% |

后果不只是「速度不对」——**循迹的差速方向也是反的**（变成正反馈，
车会朝着偏离方向越走越远）。

### 4.2 SPI1 的 MISO 引脚冲突 → 编译失败 → 旧固件还在跑

把 PB21 分给 LED 之后编译报：

```
error: OLED(/ti/driverlib/SPI) peripheral.misoPin: Resource conflict
```

因为 PB21 原本是 SPI1 自动分配的 MISO 脚，而 SPI1 的 MISO 候选引脚全被占或不可用。

**修法**：OLED 是单向写的，根本不需要 MISO → 在 `empty.syscfg` 里加一行
`SPI1.direction = "PICO";`
（依据：`source/ti/driverlib/.meta/spi/SPIMSPM0.syscfg.js` 的 `pinmuxRequirements()`）。

### 4.3 `DL_GPIO_readPins()` 读输出脚永远返回 0

`dl_gpio.h:1910` 的 `DL_GPIO_initDigitalOutput()` **不置 `INENA_ENABLE` 位** ——
引脚配成普通输出时**输入缓冲是关掉的**，所以读回来一律是 0。
（对比同文件 1985 行的 `DL_GPIO_initDigitalInput()`，那个才带 `INENA_ENABLE`）

**结论：不要用「驱动某个输出脚、再读回来验证」的办法做自检。**

### 4.4 开机时钟初始化有三处「没有超时」的死等 —— ✅ 已修复

**症状**（实测出现过）：

| 现象 | 说明 |
| --- | --- |
| OLED 全黑、串口不出 `[1]` | 卡在 `SYSCFG_DL_init()` 里，GPIO / SPI 都还没配置 |
| **时好时坏**，偶尔能跑起来 | 晶振起振是概率性的，那次恰好起来了 |
| 复位 / 重上电又卡死 | 复位后跑到同一行，再卡一次 |
| **连烧录都连不上** | 芯片卡在里面，调试口也进不去 |

**原因**：`Debug/ti_msp_dl_config.c`（SysConfig 生成）里有：

```c
while (DL_SYSCTL_isFCCDone() == 0) { }                      // 测 SYSPLLCLK0 频率
while (DL_SYSCTL_isFCCDone() == 0) { }                      // 测 HFCLK 频率
while (SYSCFG_DL_SYSCTL_SYSPLL_init() == false) { ... }     // 等 PLL 锁定
```

最后那句 **TI 自己在注释里就写了**：
*This can lead an infinite loop ... and can block entry to the application code.*

原来的时钟链是 `HFXT(外部晶振) -> SYSPLL -> MCLK(80MHz)`。**晶振起振天生是概率性的**
（虚焊 / 负载电容不对 / 温度 / 板子受力），一次没起振就永远卡住 ——
**跟供电质量无关，按复位也没用**。

**修复做法**：在 `empty.syscfg` 里把时钟源换成芯片内部的 SYSOSC：

```js
const mux8       = system.clockTree["HSCLKMUX"];
mux8.inputSelect = "HSCLKMUX_SYSOSC";      // 原来: "HSCLKMUX_SYSPLL0"

const pinFunction4  = system.clockTree["HFXT"];
pinFunction4.enable = false;               // 原来: true
```

> ⚠️ 关掉 HFXT 之后，文件底部那两行建议分配
> `pinFunction4.peripheral.hfxInPin.$suggestSolution` / `...hfxOutPin...`
> **必须一起删掉**，否则 SysConfig 报
> `Cannot read properties of undefined (reading hfxInPin)`。
>
> 另外 `UDIV`（ULPCLK 分频）在从 SYSOSC 取时钟时会被忽略，设了会报
> `UDIV will be disabled (/1)...` 警告 —— 不设即可。

**修复后**：`SYSCFG_DL_SYSCTL_init()` 只剩两行，**三处死等整个消失**：

```c
DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
DL_SYSCTL_disableHFXT();
```

**主频变化 80MHz → 32MHz，连带影响（都已由 SysConfig 自动重算并核对过）**：

| 项目 | 改前 | 改后 | 需不需要手改代码 |
| --- | --- | --- | --- |
| `CPUCLK_FREQ` | 80000000 | 32000000 | 不用，`delay.c` / `tick.c` 用的就是这个宏 |
| 电机 PWM | 20kHz | **16kHz** | 不用，`period` 仍是 1000，`MOTOR_PWM_PERIOD` 不用动 |
| 按键扫描定时器 | 50ms | 50ms | 不用 |
| OLED SPI 位率 | 8MHz | 自动重算 | 不用 |

> 结论：**占空比是比值，和频率无关**，所以 PWM 频率变了不影响控制；
> 16kHz 仍然远高于音频段、远低于 TB6612 上限。

> 正式板子到了、确认晶振稳定之后，也可以把 `HSCLKMUX` 改回 `SYSPLL0`
> 换回 80MHz —— 但**没有必要**，32MHz 对本项目完全够用。

### 4.5 串口里写死的「样板数据」会骗人

启动横幅里曾经硬编码过：

```
S=00011000 E=-014 L=040 R=040
K1=follow on/off   K2=motor test
```

跟真实数据的格式一模一样，排查时被误导过。**已经删掉，不要加回去。**

---

## 5. 参数在哪改

全部集中在 **`system/line_follow.c` 顶部的「可调参数」区**：

| 宏 | 作用 | 现在 |
| --- | --- | --- |
| `LF_MAX_DUTY` | 最高速度硬顶（任何一轮都不许超过） | 20 |
| `LF_BASE_DUTY` | 直行基础速度，**必须明显高于电机死区** | 18 |
| `LF_LOST_DUTY` | 丢线找线速度 | 16 |
| `LF_TEST_DUTY` | KEY2 自检速度 | 20 |
| `LF_MAX_STEER` | 转向量上限，**必须 >= `LF_BASE_DUTY`** | 18 |
| `LF_KP` / `LF_KD` | 转向 PID 的 P / D（**D 必须 0 或很小**） | 20 / 0 |
| `LF_DEADBAND` | 误差死区，专治直线小摆 | 15 |
| `LF_STEER_SIGN` | 转向极性 +1 / -1 | +1 |
| `LF_TRIM` | 左右电机补偿（车往一边偏才用） | 0 |
| `LF_PIVOT_TRIGGER_MS` | 连续丢线多久判定「到弯节点」 | 80 |
| `LF_PIVOT_DUTY` | 原地转向的占空比（一正一反） | 16 |
| `LF_PIVOT_OK` | 误差绝对值小于它算「对准了」 | 20 |
| `LF_PIVOT_TRY_MS` | 一个方向找多久没找到就掉头 | 900 |

> 这些宏之间有**编译期检查**（`#error`）。配错了会在编译时直接报错，
> 不会等到跑车才发现。

---

## 6. 快速自检清单（每次改动之后过一遍）

- [ ] 编译：`gmake -C Debug -j4 all` → exit 0，0 error 0 warning
- [ ] 串口：`[1] [2] [3]` 三行都出现
- [ ] 屏幕：OLED 显示 `LF:STOP` + `S:` 位图 + `E:` + `L:` `R:`
- [ ] 传感器：黑物划过，`S=` 里的 `1` 跟着走
- [ ] 电机：KEY2 两轮同速前进（看 `L:` `R:` 和实际转向）
- [ ] 转向极性：悬空按 KEY1，线往左偏 → 左轮变慢、右轮变快
- [ ] 限速：`L:` `R:` 任何时候都不超过 `LF_MAX_DUTY`

