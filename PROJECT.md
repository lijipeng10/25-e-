# 智能车工程 (MSPM0G3507)

> 基于 TI MSPM0G3507 的智能车 / 循迹小车工程。

> **出问题先翻 [DEBUG.md](DEBUG.md)** —— 串口日志怎么看、每种故障怎么定位、
> 已经踩过的坑和根因, 全在里面。本文件是工程全貌(引脚/模块/中断/构建)。

---

## 1. 开发环境

| 项目 | 值 |
| --- | --- |
| MCU | MSPM0G3507 (LQFP-64 / PM) |
| SDK | MSPM0 SDK 2.11.00.07 (`C:/TI/mspm0_sdk_2_11_00_07`) |
| 配置工具 | SysConfig 1.26.2 (`empty.syscfg`) |
| IDE | CCS Theia (`.ccsproject` / `.cproject`) |
| 编译器 | ti-cgt-armllvm 4.0.4 LTS |
| 主频 | **SYSOSC 32 MHz** (CPUCLK = BUSCLK = 32 MHz) |

> ⚠️ 原来用的是 `HFXT(外部晶振) -> SYSPLL -> MCLK 80MHz`，**已经换掉了**。
> 原因见 5.5：那条链上有三处没有超时的死等，晶振一起振不良就永远卡住。
> 改成芯片内部 SYSOSC 后那个故障模式整个消失。

---

## 2. 引脚分配

### 2.1 小车本体(在用)

| 功能 | 外设 / 引脚 | 备注 |
| --- | --- | --- |
| **按键** | KEY1 = PA29, KEY2 = PB27 | 输入 + 上拉, 按下为低 |
| **电机 A 路** | PWM = PB6, AIN1 = PB17, AIN2 = PB18 | TB6612 |
| **电机 B 路** | PWM = PB7, BIN1 = PB19, BIN2 = PB23 | TB6612 |
| **电机 STBY** | PA16 | 高 = 使能 |
| **LED1** | PB2 | ⚠️ **暂不使用** —— 扩展板还没到, 车上没有 LED |
| **LED2** | PB3 | 同上 (syscfg 里配着, 但代码里不驱动) |

> 📌 **LED 先都不要用。** 扩展板到货、灯能用了再说。
>
> - `empty.c` 里原来的 LED 诊断代码(心跳灯 / 异常闪灯)**已经删掉了**，
>   现在故障诊断走 **串口 + OLED**（见 `DEBUG.md` 第 2、3 节）。
> - 以前"三个 LED 都不亮"的结论**不用管** —— 那是因为最小系统板上压根没焊灯。
> - 以后要恢复：`hardware/led.c` 驱动还留着，直接 `led_on(id)` 即可
>   （`id`: 1 = PB2, 2 = PB3）。注意它按「高电平点亮」写，
>   如果你的板子是低电平点亮，把 `setPins` / `clearPins` 对调。
| **灰度传感器** | OUT = PA22, AD0 = PB24, AD1 = PA24, AD2 = PA26 | 8 路循迹 |
| **MPU6050** | I2C0: SDA = PA0, SCL = PA1 | 陀螺仪(航向) |
| **OLED** | SPI1: SCLK = PB9, MOSI = PB8 | 1.3 寸 128x64 SH1106, 7 针 |
| | GPIO: RES = PB10, DC = PB11, CS = PB14, BLK = PB26 | |
| | **SPI 只设为 `PICO`(只发)** | OLED 是单向写的, MISO 用不上。 |
| | 不设成 PICO 的话 SysConfig 必须给 MISO 分配引脚, |
| | 而 SPI1 的 MISO 唯一能用的就是 PB21, 会把 PB21 占掉。 |
| **蜂鸣器** | PB0 | |
| **PC 调试串口** | UART2: TX = PB15, RX = PB16, 115200 | |

### 2.2 云台/步进电机(遗留, 后续会改)

> ⚠️ **下面这张表是"当前 `empty.syscfg` 里实际配置的值"**，不是最终方案。
> 扩展板还没到，**后期这些引脚会改** —— 以 `empty.syscfg` 为准，本表只是快照。
>
> 注意区分两件事：
> - **外设是初始化了的** —— `SYSCFG_DL_init()` 会把 syscfg 里配置的外设全部初始化，
>   这些引脚在开机后就是"已配置"状态。
> - **模块逻辑没被调用** —— `sm_motor` / `vision` / `uart` 这些模块的 init 和
>   主循环逻辑都没有接进去。
>
> 所以这张表只能说明"引脚被占了"，**不能说明"这个功能能用"**。

| 功能 | 引脚 |
| --- | --- |
| 步进 DIR1/DIR2 | PA8 / PA9 |
| 步进 EN1/EN2 | PB12 / PB13 |
| 步进 STEP1 (TIMA1) | PA28 |
| 步进 STEP2 (TIMG12) | PA31 |
| MT6816 编码器 A1/B1/Z1 | PA15 / PA17 / PA30 |
| MT6816 编码器 A2/B2/Z2 | PB22 / PB1 / PA7 |
| 轮速编码器 E1A/E1B | PB20 / PA14 |
| 轮速编码器 E2A/E2B | PA25 / PB25 |
| 视觉串口 UART1 | TX = PB4, RX = PB5 |

---

## 3. 模块清单

### 3.1 在用

| 模块 | 文件 | 说明 |
| --- | --- | --- |
| 时基 | `system/tick.c` | SysTick 1ms, `tick_get_ms()` |
| 延时 | `system/delay.c` | 忙等延时, `delay_ms/us` |
| PID | `system/pid.c` | 通用位置式 PID |
| 按键 | `hardware/key.c` | 定时器中断扫键, `key_getnum()` 取键码 |
| 电机 | `hardware/motor.c` | TB6612 双直流, 占空比对外 0~100 (%) |
| LED | `hardware/led.c` | |
| 灰度 | `hardware/grayscale_sensor.c` | 8 路, `Grayscale_Sensor_Read_All()` |
| 陀螺仪 | `hardware/mpu6050.c` | 只用 Z 轴积分求航向, 必须固定周期 `mpu6050_update()` |
| OLED | `hardware/oled.c` + `oledfont.h` | SH1106 SPI 驱动 |
| **循迹** | `system/line_follow.c` | 灰度8路 -> 偏差 -> PID -> 左右轮差速; 到弯节点自动停车原地转向 |
| 蜂鸣器 | `hardware/buzzer.c` | |

### 3.2 云台模块(保留, 当前未初始化)

| 模块 | 文件 | 说明 |
| --- | --- | --- |
| 步进电机 | `hardware/sm_motor.c` (17KB) | 两轴步进云台, 加减速/走角度/画圆 |
| MT6816 编码器 | `hardware/sm_encoder.c` | 磁编码器四倍频计数, **占用 GROUP1_IRQHandler** |
| 视觉伺服 | `system/vision.c` | 视觉串口帧解析 + PID, 依赖 sm_motor |
| 视觉串口 | `system/uart.c` | UART1 环形缓冲, **占用 UART1_IRQHandler** |
| 轮速编码器 | `hardware/encoder.c` | 轮速计算(当前未启用中断) |

> 📌 **小车和云台都要保留**, 后续要在同一个主控里耦合运行。
> 目前 `empty.c` 只初始化小车用到的外设; 云台那套代码保留、参与编译, 但暂不初始化。
> 耦合时把这些模块的 init 一起加进 `empty.c` 即可(注意下面第 4 节的中断冲突)。

---

## 4. 中断分配

| 中断 | 触发源 | 用途 | 实现位置 |
| --- | --- | --- | --- |
| `SysTick_Handler` | SysTick 1 ms | 累加毫秒时基 | `system/tick.c` |
| `main_timer_INST_IRQHandler`<br>(= `TIMA0_IRQHandler`) | TIMA0, 50 ms | `key_tick()` 扫键 | `empty.c` |
| `GROUP1_IRQHandler` | GPIOA/GPIOB | MT6816 编码器计数(遗留) | `hardware/sm_encoder.c` |
| `UART1_IRQHandler` | UART1 | 视觉串口收字节(遗留) | `system/uart.c` |
| `TIMA1_IRQHandler` / `TIMG12_IRQHandler` | TIMA1 / TIMG12 | 步进电机步进(遗留) | `hardware/sm_motor.c` |

### 4.1 轮速编码器的中断方案 —— **旧的作废，等新方案**

> 🔄 **2026/09/14 更新：轮速编码器的硬件方案已经换了，下面这段旧分析作废。**
>
> 旧方案讨论的是"GPIO 中断 / 定时器 QEI 怎么读正交编码器"，
> 但**新硬件方案还没定**，所以这一节暂时清空，免得影响现在的判断。
>
> 等开发到那个阶段，再拿新方案来评估可行性。
>
> 现在只需要知道一条**不变的事实**：
>
> ```c
> GPIOA_INT_IRQn = 1      // mspm0g350x.h:81
> GPIOB_INT_IRQn = 1      // mspm0g350x.h:80  <- 同一个中断号
> ```
>
> **GPIOA 和 GPIOB 共用同一个 NVIC 向量**（IRQ 1 = `GROUP1_IRQHandler`）。
> 以后不管什么方案，只要走 GPIO 中断，就都要考虑共用这个向量。
>
> 空闲定时器（以后可能有用）：**TIMG0、TIMG6、TIMG7**。
> 已占用：TIMA0(按键扫描) / TIMA1 和 TIMG12(云台步进) / TIMG8(电机 PWM)。

```c
GPIOA_INT_IRQn = 1      // mspm0g350x.h:81
GPIOB_INT_IRQn = 1      // mspm0g350x.h:80  <- 同一个中断号!
```

GPIOA 和 GPIOB 是**同一个 NVIC 向量**(IRQ 1 = `GROUP1_IRQHandler`)。
所以只要走 GPIO 中断, 就必然和 MT6816 编码器共用一个中断服务函数。

**轮速编码器计数有三种做法, 按推荐度排序:**

| 方案 | 做法 | 评价 |
| --- | --- | --- |
| **① 定时器 QEI 硬件解码** | SysConfig 里加 **"TIMER - QEI"** 实例(Quadrature Encoder Interface), 用空闲定时器硬件做正交解码 | **最推荐**: 不占中断、不占 CPU, 直接读计数寄存器 |
| ② 并入现有 GROUP1 | 在 `GROUP1_IRQHandler` 里分别判断 GPIOA/GPIOB 的中断状态位, 两个编码器各管各的引脚 | 可行, 简单; 注意各自清各自的挂起标志 |
| ③ 定时器周期采样 | 定时器中断里定期读 A/B 电平做软件解码 | 不占 GPIO 中断, 但占 CPU, 高速时会丢计数 |

空闲定时器(可用于 QEI 或方案③): **TIMG0、TIMG6、TIMG7**
已占用: TIMA0(main_timer) / TIMA1(云台ST1) / TIMG12(云台ST2) / TIMG8(电机PWM)

> QEI 具体支持哪些定时器, 在 SysConfig 里加一个 "TIMER - QEI" 实例,
> 外设下拉框里能选的就是支持的。

---

## 5. 已确认的关键配置(踩坑记录)

这些都是实测确认过的, 改动前先读一遍, 能省很多时间。

### 5.1 OLED(1.3 寸 SH1106) —— 坑最多

| 项目 | 值 | 说明 |
| --- | --- | --- |
| **控制器** | SH1106 | 1.3 寸绝大多数是 SH1106, **不是 SSD1306** |
| **列偏移** | **2** | SH1106 显存 132 列, 可见的 128 列从**第 2 列**开始。<br>实测: 偏移 0/1 -> 左边缺一截; 3 -> 右边缺一截; **2 才完整** |
| **显存位序** | 页内 **bit0 = 最上面一行** | 写成 `1<<(7-y%8)` 会导致每个 8 像素横条上下翻转 -> **全屏乱码** |
| **方向** | `0xA1` / `0xC8` | 段重映射 / COM 扫描 |
| **复位** | 高 -> 100ms -> 低 -> 200ms -> 高 | |
| **初始化** | `0x20,0x02` + `0x8D,0x14` | 页寻址 + 电荷泵(SH1106 会忽略 0x20, 无害) |
| **SPI** | 模式 0 + MSB 先发 | 与官方例程的软件 SPI 时序一致 |

> ⚠️ 官方 1.3 寸例程的列偏移**藏在 `OLED_Refresh()` 里(`0x02`), 不在 `OLED_Init()` 里(`0x00`)** ——
> 因为每次刷屏都会重设列地址, Init 里的值会被覆盖。只看初始化会被误导。

### 5.2 电机 PWM

| 项目 | 值 | 说明 |
| --- | --- | --- |
| 定时器 | TIMG8 (`motor_pwm`) | 边沿对齐 PWM |
| 计数周期 | `timerCount = 1000` | 定时器时钟 16MHz -> **PWM = 16kHz**（主频换 SYSOSC 后自动重算） |
| 占空比 | 对外 **0~100 (%)** | `motor.c` 里 `MOTOR_PWM_PERIOD` 负责换算 |

> ⚠️ **`motor.c` 的 `MOTOR_PWM_PERIOD` 必须和 `empty.syscfg` 的 `PWM3.timerCount` 保持一致**,
> 否则占空比会按比例失真。
> 之前用过 `timerCount=100` -> PWM 198kHz, 超出 TB6612 上限, 表现为**电机狂转、噪音大、震动**。

#### 5.2.1 ★★ 占空比换算是「反」的（已修的严重 bug）

MSPM0 的 TimerG 在 EDGE_ALIGN PWM 模式下**从 LOAD 往下数**，CCP 输出在
「计数 > 比较值」这段时间为高，所以：

```
实际占空比 = (周期 - 比较值) / 周期        <-- 比较值越大, 占空比越小!
```

TI 自己的 SysConfig 就是这么算的，见
`source/ti/driverlib/.meta/pwm/PWMTimerCC.syscfg.js` 第 107 行：

```js
proposedccValue = Math.round( (100 - inst.dutyCycle) * (period) / 100) - 1;
```

官方例 `timx_timer_mode_pwm_edge_sleep` 也可交叉验证：
`timerCount = 2000`、`dutyCycle = 75` -> 生成 `ccValue = 500`（不是 1500）。

**踩过的坑**：`motor.c` 原来写成 `cmp = duty * PERIOD / 100`，正好写反了：

| `motor_set_duty()` 填的值 | 实际输出占空比 |
| --- | --- |
| 20 | **80%** |
| 40 | **60%** |
| 50 | 50%（唯一对称点，所以「填 50 看着是对的」，极易漏过去） |
| 90 | **10%** |

两个后果：

1. 表观现象是「**速度数值越大反而越慢**」，很容易误判成电机或机械问题
2. 更严重的是**循迹的差速方向也是反的** —— 想让左轮快，比较值变大，左轮实际更慢。
   闭环变成正反馈，车会朝着偏离方向越走越远。而因为 50 是守恒点，
   「KEY2 自检」看起来还完全正常，非常有迷惑性。

**修复**：`motor_duty_to_cmp()` 改成 `(100 - duty) * PERIOD / 100`。
同理 `motor_init()` 里给占空比清零**不能直接写 0**（比较值 0 = 100% 占空比），
必须写 `motor_duty_to_cmp(0)`。

### 5.3 按键

| 项目 | 值 | 说明 |
| --- | --- | --- |
| 扫描周期 | **50 ms** | `main_timer`(TIMA0) 中断里 `key_tick()` |
| 键码 | `key_getnum()` 返回 0/1/2 | 松手时出码 |
| 定时器配置 | `key_init()` 里配成周期模式 + 开中断 + NVIC | SysConfig 生成的是"一次性+不启动", 必须重配 |

> ⚠️ 配置定时器时 `DL_Timer_TimerConfig cfg = { 0 };` **必须清零** ——
> `DL_Timer_initTimerMode()` 会读 `genIntermInt`/`counterVal`, 栈上随机值会把定时器配坏
> (表现为中断永远不进)。

### 5.4 MPU6050 探测不到 —— 根因是 SysConfig 漏了一行（已修）

> 🔄 **这一节的内容整个换过了。** 原来写的是"驱动没有超时保护、所以不敢调用 init"，
> **那已经过时**：驱动的每个 I2C 等待**早就加了超时**，`mpu6050_init()` 现在也
> **已经在主程序里调用了**。

真正踩到的坑是另一个，而且没有任何日志提示，白查了两轮：

**现象**：接线正常（SDA=PA0 / SCL=PA1），但屏幕上一直显示 `MPU:NO`，
`0x68` / `0x69` 两个从机地址都不应答。

**根因**：`empty.syscfg` 里少了这一行 ——

```js
I2C1.basicEnableController = true;
```

没有它，SysConfig 认为「这个 I2C 实例不做控制器」，于是生成的
`SYSCFG_DL_MPU6050_init()` 里**只有时钟和滤波，没有控制器配置、
也没有 `DL_I2C_enableController()`** —— 外设根本没跑起来，
任何地址都不可能应答。**和接线、地址都无关。**

补上之后，生成的代码才变成完整版：

```c
DL_I2C_resetControllerTransfer(MPU6050_INST);
DL_I2C_setTimerPeriod(MPU6050_INST, 31);          /* 100 kHz */
DL_I2C_enableControllerClockStretching(MPU6050_INST);
DL_I2C_enableController(MPU6050_INST);            /* ★ 关键 */
```

> 💡 **通用教训**：SysConfig 里「加了外设实例」**不等于**「配好了」。
> 判断方法：**去看生成的 `ti_msp_dl_config.c` 里有没有 `DL_xxx_init()` /
> `DL_xxx_enable()`**。只有 setClockConfig + 滤波、没有 init/enable，
> 就说明还差一个勾。（OLED 的 SPI 就是对的：`DL_SPI_init` + `DL_SPI_enable` 都在）

**另外两条使用注意**：

- **必须先 `mpu6050_ping()` 再 `mpu6050_init()`** —— init 里的零偏标定要做 200 次读取，
  传感器没接时每次都等满超时，**合计十几秒**，看起来就像死机。
  我们刚修掉一个开机卡死，不能再引进一个。
- 从机地址会**自动探测 0x68 / 0x69**（很多模块 AD0 悬空，实际地址是 0x69）。
  屏幕上会显示 `MPU:68` / `MPU:69` / `MPU:NO`。

### 5.5 ★ 系统为什么容易死机 / 稳定性差 —— 根因已修

> 🔄 **这一节整个重写过。** 原来记的是"用 LED 闪灯区分死机类型"那套方案，
> **LED 诊断代码已经删掉了**（最小系统板上没焊灯）。现在诊断走**串口 + OLED**，
> 判读方法见 [DEBUG.md](DEBUG.md) 第 3 节。下面讲**根因**。

**两条看起来完全不同的症状，其实是同一个根因：**

| | 症状 | 表现 |
| --- | --- | --- |
| **A** | 运行中冻住 | OLED **有显示**但数字不动、按键没用、**按复位也没用** |
| **B** | 开机就死 | OLED **全黑**、串口一行都不出、**连烧录都连不上**（时好时坏） |

**根因：时钟链 `HFXT(外部晶振 40MHz) -> SYSPLL -> MCLK(80MHz)`。**

`Debug/ti_msp_dl_config.c`（SysConfig 生成）的开机初始化里有**三处没有超时的死等**：

```c
while (DL_SYSCTL_isFCCDone() == 0) { }                      // 测 SYSPLLCLK0 频率
while (DL_SYSCTL_isFCCDone() == 0) { }                      // 测 HFCLK 频率
while (SYSCFG_DL_SYSCTL_SYSPLL_init() == false) { ... }     // 等 PLL 锁定
```

最后那句 **TI 自己在注释里就写了**：
*This can lead an infinite loop ... and can block entry to the application code.*

**为什么这三处特别危险 —— 判据严得离谱：**

```c
#define FLOAT_TO_INT_SCALE   (1000U)
#define FCC_EXPECTED_RATIO  2000
#define FCC_UPPER_BOUND     (FCC_EXPECTED_RATIO * (1 + 0.003))   /* 2006 */
#define FCC_LOWER_BOUND     (FCC_EXPECTED_RATIO * (1 - 0.003))   /* 1994 */
```

它拿 LFCLK 当尺子，量出 SYSPLL 输出和 HFXT 的频率算比值，
**必须落在 ±0.3% 窗口内**才认为「锁对了」，否则就一直重试、永远出不来。

**这就解释了"时好时坏"：**

| 这次开机 | 结果 |
| --- | --- |
| 晶振正常起振 | ✅ 启动成功 |
| 晶振没起振（虚焊 / 负载电容不对 / 温度 / 板子受力） | ❌ **永远卡在开机里** -> 症状 B |
| 运行中 PLL 失锁（电源扰动 / 晶振抖动） | ❌ **MCLK 没了，CPU 直接停** -> 症状 A |

**两条症状都对上了：**

- **症状 B（OLED 全黑）**：卡在 `SYSCFG_DL_init()` 里，**GPIO / SPI 都还没配**，
  所以屏幕什么都不显示
- **症状 A（有显示但冻住）**：已经跑起来了，运行中 PLL 失锁 -> **CPU 停止取指** -> 屏幕定格
- **「复位没用」**：复位后跑到同一行，又卡一次
- **「跟供电质量无关」**：所以换纯净电源也不改善 ✓（这一点当初误导过我们，
  以为是电源问题查了很久）

**修复（已做）**：时钟源换成芯片内部的 SYSOSC —— 不用外部晶振、不用 PLL，
**那三处死等整个消失**：

```c
SYSCFG_DL_SYSCTL_init(void) {
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_disableHFXT();
}
```

**实测结果**：换完以后重上电、按复位都稳定，不再出现这两种症状 ✓

> 代价：主频 80MHz -> 32MHz。外设频率由 SysConfig 自动重算，已逐个核对
> （PWM 20kHz->16kHz、按键定时器仍 50ms、SPI 位率自动调）。
> **占空比是比值，和频率无关**，所以控制不受影响。
>
> ⚠️ **如果以后换回外部晶振**（比如正式板子到了想跑 80MHz），
> **这两个症状会一起回来。** 换回去之前先确认晶振起振稳定。

**现在怎么诊断（不再用 LED）：**

| 情况 | 怎么看 |
| --- | --- |
| 开机就死（症状 B） | 串口不出 `[1]` / OLED 全黑 -> 卡在 `SYSCFG_DL_init()` 里 |
| CPU 跑飞（HardFault / NMI） | 串口打 `!!! HARDFAULT !!!`，OLED 定格 `!! FAULT !!` |
| 时基停了 | 串口 `HB` 还在打、但 `S=` 数据行不再更新 |
| CPU 卡在外设等待 | 串口 `HB` 也停了 |

完整判读流程见 [DEBUG.md](DEBUG.md) 第 3 节。

---

## 6. 构建

在 CCS Theia 里直接 `Build` 即可(SysConfig 会自动重新生成 `Debug/ti_msp_dl_config.*`)。

命令行构建:

```powershell
& 'D:/TI/ccs2025/ccs/utils/bin/gmake.exe' -C Debug -j4 all
```

`Debug/` 整个目录都是生成物, 不入库(见 `.gitignore`)。

> ⚠️ `gmake` 会**自动调用 SysConfig** 重新生成 `ti_msp_dl_config.*`
> (用的是与工程匹配的 `C:/TI/sysconfig_1.26.2/`), 所以改了 `empty.syscfg` 直接构建即可。
> **但一定要确认生成本身没报错** —— SysConfig 失败时编译会中途停住,
> 结果就是"改了代码但板子上跑的还是旧固件"。详见 [DEBUG.md](DEBUG.md) 3.3 节。

---

## 7. 当前状态

> 这一节写的是**【现在】**的状态。
> 被推翻的旧结论单列在 **7.3** —— **不要再按那些改**。

### 7.1 已经做成的（含实测数据）

**已实测验证的事实：**

| 项目 | 实测值 | 怎么测的 |
| --- | --- | --- |
| 电机启动死区 | **≤ 10** | KEY2 升档法：一档一档升占空比，看哪一档开始能持续转 |
| 传感器 8 路左右顺序 | **正确** | 黑物从车左侧划到右侧，`1` 位依次右移、`E` 从 -100 到 +100 |
| 两轮对称性 | 轻微往**左**偏 | 按 KEY2（两轮同指令）在地上跑 -> `LF_TRIM = +2` |
| 开机卡死 / 烧录不进 | **✅ 已断根** | 时钟换 SYSOSC（见 5.5），重上电和复位都稳定 |

**当前参数**（全部在 `system/line_follow.c` 顶部；开机参数页也会显示，不用翻代码）：

```
LF_MAX_DUTY         20     最高速度硬顶（任何一轮都超不过它）
LF_BASE_DUTY        20     直行基础速度
LF_LOST_DUTY        16     丢线找线速度
LF_TEST_DUTY        20     KEY2 自检的参考上限
LF_MAX_STEER        20     转向量上限（必须 >= LF_BASE_DUTY）
LF_KP               15     转向比例
LF_KD                0     数字量误差上 D 没用
LF_GYRO_KD          10     陀螺仪阻尼系数（★ 符号还没实测确认）
LF_DEADBAND          0     误差死区（已证明：加死区反而更摆）
LF_TRIM             +2     左右电机补偿
LF_STEER_SIGN       +1     转向极性（已实测正确）
LF_CORNER_ERR       71     急弯判据：|error| 到 71 = 线甩到传感器边上了
LF_CORNER_MS        60     连续 60ms 满足才算急弯
LF_PIVOT_TRIGGER_MS 150    丢线兜底判据
LF_PIVOT_DUTY       20     原地转向占空比
LF_PIVOT_OK         20     对准判据
LF_PIVOT_TRY_MS     1300   一个方向找这么久没找到就掉头找另一边
```

**循迹逻辑**（`line_follow_step`，每 10 ms 跑一次）：

```
看到线 -> 直道: 灰度 8 路 -> error -> PID -> 左右轮差速
         -> 急弯: |error| >= 71 持续 60ms -> 【停车原地转向】(不等丢线!)
丢线   -> 150ms 内: 按"方向证据"继续找线, 同时降速
         -> 超过 150ms: 停车原地转向
原地转向 -> 转到线落在中间 (|error| <= 20) 就算对准, 回去循迹
         -> 一个方向找 1300ms 没找到 -> 掉头找另一边
         -> 两边都找遍还是没有 -> 判定掉出赛道, 停车
```

### 7.2 还没做完的（按优先级）

- [ ] **★ 小车整体还没达到稳定运行的预期** —— 直道上仍然左右摆尾，走不直
- [ ] **陀螺仪闭环只做了一半**：目前只用**角速度**做阻尼（`LF_GYRO_KD`）；
      **航向角闭环还没做**。而且要先把 `LF_GYRO_KD` 的**符号**实测出来
      （车头往左转时看屏幕 `R:` 的正负）
- [ ] **赛道标定**：`LF_KP` / `LF_BASE_DUTY` / `LF_CORNER_ERR` / `LF_PIVOT_*`
      都还没在梯形赛道上定下来
- [ ] 轮速编码器：**硬件方案已换**，等新方案（旧的 4.1 分析已作废）
- [ ] LED：扩展板未到，代码里暂不使用（驱动留着）
- [ ] 小车与云台耦合：两套模块的 init 都加进 `empty.c`
- [ ] 汉字字库：`oledfont.h` 里每个汉字表只有「中」一个字，要显示别的字需自己取模

### 7.3 ⚠️ 被推翻的旧结论（不要再按这些改）

| 旧结论 | 实测结果 |
| --- | --- |
| 「直道摆尾是因为基础速度贴着电机死区」-> 所以把 `LF_BASE_DUTY` 往低调 | ❌ **错**。实测死区只有 ≤10，当时 BASE=14 离得很远。真因是**修正力度曲线里的台阶** |
| 「加 `LF_DEADBAND=15` 能把量化抖动压掉」 | ❌ **错**。加了**反而摆得更凶** —— 死区把 ±14 变成 0，造成「完全不管 -> 到 ±28 猛踢一下」的台阶，典型极限环。现在**改成 0** |
| 「MPU6050 驱动没有超时保护，不能调用 `mpu6050_init()`」 | ❌ **过时**。驱动早就有超时了，现在**已经在用** |
| 「开机时钟死循环跟本工程无关」 | ❌ **错**。那就是「开机卡死 / 烧录不进」的根因，已换成 SYSOSC |
| 「三个 LED 都不亮，是电路问题」 | ❌ **错**。最小系统板上**压根没焊 LED** |
| 「把基础速度调低能让车跑慢点」 | ⚠️ 反直觉：**速度越低摆得越凶**（会踩到电机死区）。要调慢得连 `LF_MAX_STEER` 一起降 |

**这一节存在的目的：以后（包括 AI）翻到这个文件时，不会再去踩已经验证过是错的坑。**


---

## 8. 能力地图

> **这不是待办清单，是索引。** 写进来的必须是**已经存在、并且验证过入口**的能力。
> 加新功能前先查这张表 —— 能复用就别新写。
> 某个能力的接口或行为变了，**在同一次改动里更新对应行**。
> 状态用 `AGENTS.md` 里的阶梯：`Implemented` / `Build Passed` / `HW Verified`。

| 能力 | 实现 | 入口 | 约束 / 注意 | 状态 |
| --- | --- | --- | --- | --- |
| 启动 / 复位 | SysConfig `SYSCFG_DL_init()` | `empty.c` 的 `main()` | 时钟已换 SYSOSC（不用外部晶振）；启动日志 `[1][2][3]` 永远打印 | HW Verified |
| 时钟 / 电源 | SYSOSC 32MHz，BUSCLK 32MHz | `empty.syscfg` 的 SYSCTL | 改时钟影响**所有**外设频率，必须逐个核对 | HW Verified |
| GPIO | SysConfig 生成 | `ti_msp_dl_config.h` 的 `*_PORT/_PIN/_IOMUX` | **输出脚读不回来**（`INENA` 未置位，`DL_GPIO_readPins` 恒为 0） | HW Verified |
| 定时器 / PWM | TIMG8 `motor_pwm` | `hardware/motor.c` | 16kHz；`MOTOR_PWM_PERIOD` 必须与 `timerCount` 同步 | HW Verified |
| 时基 | SysTick 1ms | `system/tick.c` | 用 `CPUCLK_FREQ` 宏，改主频自动跟 | HW Verified |
| 忙等延时 | 纯软件计数 | `system/delay.c` | 用 `CPUCLK_FREQ` 宏 | HW Verified |
| 灰度传感器 | 8 路 + AD0/AD1/AD2 选通 | `hardware/grayscale_sensor.c` | **压黑线 = 1**；白底全 0 是正常的 | HW Verified |
| 电机驱动 | TB6612 | `hardware/motor.c` | 换算 `(100-duty)*PERIOD/100`；**基础速度必须明显高于死区(实测 ≤10)** | HW Verified |
| 按键 | TIMA0 50ms 扫描 | `hardware/key.c` | 定时器结构体必须 `= {0}` 初始化，否则中断不来 | HW Verified |
| 显示 | SH1106 / SPI | `hardware/oled.c` | 列偏移**必须 = 2**；点阵 `bit0` 在上 | HW Verified |
| PID | 通用位置式 | `system/pid.c` | 数字量误差上**微分项 D 没用**（放大的全是量化台阶） | HW Verified |
| 循迹 | 灰度→误差→PID→差速 + 弯道停车转向 | `system/line_follow.c` | 参数全在文件顶部；有编译期检查兜错 | Build Passed（待赛道标定） |
| 陀螺仪 | MPU6050 / I2C0 (PA0/PA1) | `hardware/mpu6050.c` | 地址自动 0x68/0x69；**必须先 `ping` 再 `init`**；`empty.syscfg` 里**必须有 `basicEnableController`** | Implemented（待烧录确认） |
| 调试面板 | OLED 参数页 + 状态页 | `empty.c` | 参数页开机 4 秒；状态页含 `Emin/Emax`、`F` 摆动次数、`R` 角速度 | Build Passed |
| 串口日志 | UART2 115200 8N1 | `empty.c` | 启动日志与故障报告**永远打印**；周期性数据受 `DBG_UART` 控制 | HW Verified |
| 云台（保留不删） | 步进电机 / 视觉 | `hardware/sm_motor.c`、`system/vision.c` | **未接入主循环**，不要在没验证的情况下调用 | 未验证 |
