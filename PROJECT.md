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
| **LED** | LED1 = PB2, LED2 = PB3 | 高电平点亮 |
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
| 蜂鸣器 | `hardware/buzzer.c` | |

### 3.2 遗留(云台项目, 编译但不初始化)

| 模块 | 文件 | 说明 |
| --- | --- | --- |
| 步进电机 | `hardware/sm_motor.c` (17KB) | 两轴步进云台, 加减速/走角度/画圆 |
| MT6816 编码器 | `hardware/sm_encoder.c` | 磁编码器四倍频计数, **占用 GROUP1_IRQHandler** |
| 视觉伺服 | `system/vision.c` | 视觉串口帧解析 + PID, 依赖 sm_motor |
| 视觉串口 | `system/uart.c` | UART1 环形缓冲, **占用 UART1_IRQHandler** |
| 轮速编码器 | `hardware/encoder.c` | 轮速计算(当前未启用中断) |

> ⚠️ 想删掉云台那套(sm_motor / sm_encoder / vision / uart), 直接删文件 + 从
> `Debug` 重新构建即可; git 里有历史记录, 随时能找回。

---

## 4. 中断分配

| 中断 | 触发源 | 用途 | 实现位置 |
| --- | --- | --- | --- |
| `SysTick_Handler` | SysTick 1 ms | 累加毫秒时基 | `system/tick.c` |
| `main_timer_INST_IRQHandler`<br>(= `TIMA0_IRQHandler`) | TIMA0, 50 ms | `key_tick()` 扫键 | `empty.c` |
| `GROUP1_IRQHandler` | GPIOA/GPIOB | MT6816 编码器计数(遗留) | `hardware/sm_encoder.c` |
| `UART1_IRQHandler` | UART1 | 视觉串口收字节(遗留) | `system/uart.c` |
| `TIMA1_IRQHandler` / `TIMG12_IRQHandler` | TIMA1 / TIMG12 | 步进电机步进(遗留) | `hardware/sm_motor.c` |

> ⚠️ **新增 GPIO 中断前注意**: GROUP1 已被 `sm_encoder.c` 占用。
> 如果你要用 GPIO 中断(比如给轮速编码器计数), 要么把代码加到现有的
> `GROUP1_IRQHandler` 里, 要么先把云台那套删掉。

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

### 5.3 按键

| 项目 | 值 | 说明 |
| --- | --- | --- |
| 扫描周期 | **50 ms** | `main_timer`(TIMA0) 中断里 `key_tick()` |
| 键码 | `key_getnum()` 返回 0/1/2 | 松手时出码 |
| 定时器配置 | `key_init()` 里配成周期模式 + 开中断 + NVIC | SysConfig 生成的是"一次性+不启动", 必须重配 |

> ⚠️ 配置定时器时 `DL_Timer_TimerConfig cfg = { 0 };` **必须清零** ——
> `DL_Timer_initTimerMode()` 会读 `genIntermInt`/`counterVal`, 栈上随机值会把定时器配坏
> (表现为中断永远不进)。

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

`empty.c` 的主循环是空的, 常用接口见文件里的速查注释。典型待做项:

- [ ] 循迹: 灰度 8 路 -> 位置误差 -> PID -> 左右轮差速(`system/line_follow.c` 需重写)
- [ ] 按键启停: KEY1 开/关循迹, KEY2 备用
- [ ] OLED 状态显示: 速度 / 偏差 / 模式
- [ ] 航向纠偏: `mpu6050_update()` 固定周期 + PID
- [ ] 汉字字库: 现在 `oledfont.h` 里每个汉字表**只有「中」一个字**, 要显示别的字需自己取模
