# 智能车工程 (MSPM0G3507)

> 基于 TI MSPM0G3507 的智能车 / 循迹小车工程。本仓库是**待开发初始版本**:
> `empty.c` 里所有外设已初始化完毕, 主循环为空, 等你往里加应用逻辑。

---

## 1. 开发环境

| 项目 | 值 |
| --- | --- |
| MCU | MSPM0G3507 (LQFP-64 / PM) |
| SDK | MSPM0 SDK 2.11.00.07 (`C:/TI/mspm0_sdk_2_11_00_07`) |
| 配置工具 | SysConfig 1.26.2 (`empty.syscfg`) |
| IDE | CCS Theia (`.ccsproject` / `.cproject`) |
| 编译器 | ti-cgt-armllvm 4.0.4 LTS |
| 主频 | CPUCLK 80 MHz (HFXT 40 MHz -> SYSPLL), BUSCLK 40 MHz |

---

## 2. 引脚分配

### 2.1 小车本体(在用)

| 功能 | 外设 / 引脚 | 备注 |
| --- | --- | --- |
| **按键** | KEY1 = PA29, KEY2 = PB27 | 输入 + 上拉, 按下为低 |
| **电机 A 路** | PWM = PB6, AIN1 = PB17, AIN2 = PB18 | TB6612 |
| **电机 B 路** | PWM = PB7, BIN1 = PB19, BIN2 = PB23 | TB6612 |
| **电机 STBY** | PA16 | 高 = 使能 |
| **LED1** | PB2 | 正常: 循迹中亮。异常: 被故障处理占用, 快闪 = 死机 |
| **LED2** | PB3 | 心跳: 主循环每转 HB_LOOPS 圈翻转一次, 一直在闪 |
| **灰度传感器** | OUT = PA22, AD0 = PB24, AD1 = PA24, AD2 = PA26 | 8 路循迹 |
| **MPU6050** | I2C0: SDA = PA0, SCL = PA1 | 陀螺仪(航向) |
| **OLED** | SPI1: SCLK = PB9, MOSI = PB8 | 1.3 寸 128x64 SH1106, 7 针 |
| | GPIO: RES = PB10, DC = PB11, CS = PB14, BLK = PB26 | |
| **蜂鸣器** | PB0 | |
| **PC 调试串口** | UART2: TX = PB15, RX = PB16, 115200 | |

### 2.2 云台/步进电机(遗留, 未初始化)

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
| **循迹** | `system/line_follow.c` | 灰度8路 -> 偏差 -> PID -> 左右轮差速(最基础版) |
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

### 4.1 关于"GPIO 中断能不能换个中断"(重要)

**不能。** MSPM0G3507 上:

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
| 计数周期 | `timerCount = 1000` | 定时器时钟 20MHz -> **PWM ≈ 20kHz** |
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

### 5.4 MPU6050 驱动有"卡死"风险

`hardware/mpu6050.c` 的 I2C 收发里, 等待标志位的循环**没有超时保护**:

```c
while (DL_I2C_isControllerRXFIFOEmpty(MPU6050_INST)) { }   /* 传感器不应答就死等 */
```

如果 MPU6050 没接好/没上电, **开机会卡在这里, 整个程序起不来**。
所以当前循迹版本的主程序里**故意没有调用** `mpu6050_init()`。
以后要用陀螺仪做航向纠偏时, 建议先给这些等待循环加上超时(比如计数 10 万次就返回错误)。

### 5.5 死机排查: 异常处理 + 心跳 LED

现象: 换用电源模块供电后系统卡住、屏幕数字冻住, 按复位键没用。
OLED 有显示说明 GPIO 已配置完, 所以**不是** 5.6 说的那种开机时钟死循环,
而是**跑起来之后主循环停了**。

麻烦在于下面三种「停」现象完全一样, 光看屏幕分不出来:

| 情况 | 病因 |
| --- | --- |
| HardFault | CPU 跑飞, 掉进启动文件默认的 `while(1)` |
| 时基停了 | 主循环还在转, 但 `tick_get_ms()` 不涨, 所有 `>= 10ms` 判断都不成立 |
| 卡在外设等待 | 死在某句 SPI/GPIO 等待里 |

**解决办法: 给它们各自一个可见的出口。** `empty.c` 里做了两件事:

1. **覆盖 `NMI_Handler` / `HardFault_Handler`**(启动文件里是弱定义的空循环)。
   改成疯狂闪 LED1 —— 闪 1 下 = NMI, 闪 2 下 = HardFault。
   用 LED 不用屏幕, 是因为异常发生时 SPI 可能已经不可靠, 写 GPIO 只要有电就亮。
2. **主循环加心跳**(`empty.c` 的 `s_hb` / `HB_LOOPS` / LED2)。
   ★ 关键: 它**不看时间、不依赖 SysTick 和任何中断**, 只数循环圈数,
   所以能精确区分「CPU 死了」和「CPU 活着但时基停了」。

判读表(屏幕冻住时看两个 LED):

| LED1 | LED2 | 结论 |
| --- | --- | --- |
| 快闪 2 下 | — | HardFault, 基本是供电不稳导致 CPU 出错 |
| 不闪 | 在闪 | CPU 活着, 时基停了 -> 查 PLL / 晶振 / VDD |
| 不闪 | 不闪 | CPU 死在某个等待循环 -> 挂调试器暂停看 PC |
| 正常 | 正常 | 屏幕或 SPI 的问题, 与控制无关 |

> ⚠️ 另外注意: SysConfig 把欠压复位阈值设成了**最低档**
> (`DL_SYSCTL_setBOR_THRESHOLD_LEVEL_0`)。电压掉下来时 MCU **不会干净地复位**,
> 而是继续在超范围电压下跑 → 跑飞, 这正好解释了「按复位键没用」。
> 如果以后还遇到类似的怪问题, 可以在 SysConfig 里把 BOR 阈值调高一档试试。

---

## 6. 构建

在 CCS Theia 里直接 `Build` 即可(SysConfig 会自动重新生成 `Debug/ti_msp_dl_config.*`)。

命令行构建:

```powershell
& 'D:/TI/ccs2025/ccs/utils/bin/gmake.exe' -C Debug -j4 all
```

`Debug/` 整个目录都是生成物, 不入库(见 `.gitignore`)。

---

## 7. 待开发

`empty.c` 的主循环已经在跑: 10ms 循迹控制 + 100ms 刷屏 + (可选)100ms 串口打印。

- [x] 循迹: 灰度 8 路 -> 位置误差 -> PID -> 左右轮差速 (`system/line_follow.c` 最基础版)
- [x] 按键启停: KEY1 开/关循迹, KEY2 电机自检(两轮 50%)
- [x] OLED 状态显示: 灰度位图 / 线偏差 / 左右轮占空比 / 循迹状态
- [x] 灰度传感器 8 路读取 + 通道选择脚 AD2/AD1/AD0 (`S=` 位图已实测正确)
- [x] `LF_LINE_LEVEL` 传感器极性: 实测"压黑线 = 1", 保持默认值 `1U` 正确

### 7.1 实车调参还没做完的部分

落地跑之前按顺序做**两次悬空验证**(车拿在手上, 轮子离地), 详见 `empty.c` 顶部注释:

| 验证 | 怎么看 | 不对就改 |
| --- | --- | --- |
| 前进方向 | 按 KEY2, 两轮都往前转 | `LF_LEFT_FWD_DIR` / `LF_RIGHT_FWD_DIR` |
| 转向极性 | 按 KEY1, 传感器在黑线上左右平移, 看屏上 `L:` `R:` 一增一减 | `LF_STEER_SIGN` (+1 <-> -1) |
| 两轮对称性 | 按 KEY2 在长直道上跑, 看车往哪偏 | `LF_TRIM` (往左偏就加大) |

两项都对之后再落地跑, 然后调:

- [x] **`LF_TRIM` 左右电机补偿** —— 两个电机死区/阻力不同, 同样占空比转速不一样,
      车会往一边偏。用 KEY2 在长直道上调 `LF_TRIM` 到走直为止
- [x] `LF_KD = 0` —— 8 路数字量传感器的 error 是台阶信号, 微分项放大的只是台阶。
      `LF_KD = 12` 时 D 是 P 的 3 倍, 会让车在线上画龙 (推导见 `line_follow.c` 注释)
- [x] **限速**: `LF_MAX_DUTY = 20` 硬顶, 在 `lf_set_wheel()` 这个唯一出口限幅,
      任何一轮都超不过它
- [x] **转弯力度**: `LF_MAX_STEER` 必须 >= `LF_BASE_DUTY`。
      差速输出是 `慢轮 = BASE - steer` / `快轮 = BASE + steer`,
      **转弯力度 = 快轮 - 慢轮**, 要让这个差拉满就必须让**慢轮能降到 0**。
      曾经 `BASE=14` + `MAX_STEER=6` -> 慢轮最低 8, 速度差只有 12,
      表现为**转弯冲过弯道**。改成 `MAX_STEER=14` 后慢轮可到 0, 速度差 20。
- [ ] `LF_KP` —— 画龙(线上打摆)就调小, 转不过来就调大 (现在 20)
      `steer = LF_KP*error/100`, 而 error 最小跳变是 14, 所以:
      误差 14 -> steer 2.8 (轻微), 误差 43 -> 8.6, 误差 70 -> 14 (打满)
- [ ] `LF_BASE_DUTY` —— 基础速度 (现在 14)。必须在启动死区之上,
      太低车不走; 太高弯道过不去
- [ ] `LF_LOST_TIMEOUT_MS` —— 丢线多久停车, 现在 500ms

### 7.2 反馈方式: OLED 为主, 串口默认关闭

**日常调试只看 OLED**, 屏幕上有全部需要的信息(见 `empty.c` 顶部注释):
循迹状态 / 8 路灰度位图 / 线偏差 / 左右轮占空比 / 按键提示。

串口那套代码保留着, 由 `empty.c` 顶部的 `#define DBG_UART` 控制, **默认 0(关)**:

- `0` = 整段打印代码不参与编译, 主循环不再被串口阻塞(**当前值**)
- `1` = 每 100ms 往 UART2(PB15, 115200)打印一行 `S=... AD=... E=... L=... R=...`

**为什么默认关**: 串口发送是死等的, 一行约 40 字符 ≈ 3.8ms, 会把 10ms 的
循迹控制周期拖出抖动。要看波形/抓时序的时候再打开。
- [ ] 航向纠偏: `mpu6050_update()` 固定周期 + PID (先解决 5.4 的超时问题)
- [ ] 轮速编码器: 用空闲定时器 TIMG6/TIMG7 配 QEI 硬件解码(见 4.1)
- [ ] 小车与云台耦合: 两套模块的 init 都加进 `empty.c`
- [ ] 汉字字库: 现在 `oledfont.h` 里每个汉字表**只有「中」一个字**, 要显示别的字需自己取模
