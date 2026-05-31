# STM32 自动行驶小车 — 项目参考手册

> **2024 年全国大学生电子设计竞赛 H 题「自动行驶小车」**
>
> MCU: STM32F103RC（Cortex-M3, 72 MHz, 256KB Flash / 48KB RAM）
> 固件库: STM32F10x Standard Peripheral Library V3.5.0

---

## 目录

1. [项目概述](#1-项目概述)
2. [硬件总览](#2-硬件总览)
3. [时钟树](#3-时钟树)
4. [引脚分配表](#4-引脚分配表)
5. [外设详细配置](#5-外设详细配置)
   - [5.1 PWM — TIM3](#51-pwm--tim3)
   - [5.2 编码器 — TIM2 / TIM4](#52-编码器--tim2--tim4)
   - [5.3 系统定时器 — TIM1 / TIM8](#53-系统定时器--tim1--tim8)
   - [5.4 串口 — USART2](#54-串口--usart2)
   - [5.5 I2C 总线（软件模拟）](#55-i2c-总线软件模拟)
   - [5.6 GPIO 输入 / 按键](#56-gpio-输入--按键)
   - [5.7 SysTick 延时](#57-systick-延时)
6. [软件模块说明](#6-软件模块说明)
   - [6.1 电机控制模块 `Motor.c`](#61-电机控制模块-motorc)
   - [6.2 PWM 模块 `PWM.c`](#62-pwm-模块-pwmc)
   - [6.3 编码器模块 `Encoder.c`](#63-编码器模块-encoderc)
   - [6.4 PID 控制器 `pid.c`](#64-pid-控制器-pidc)
   - [6.5 MPU6050 驱动 `MPU6050.c`](#65-mpu6050-驱动-mpu6050c)
   - [6.6 姿态解算 `MadgwickAHRS.c`](#66-姿态解算-madgwickahrsc)
   - [6.7 OLED 显示 `OLED.c`](#67-oled-显示-oledc)
   - [6.8 灰度巡线 `gray_track.c`](#68-灰度巡线-gray_trackc)
   - [6.9 串口通信 `Serial.c`](#69-串口通信-serialc)
   - [6.10 按键 `Key.c`](#610-按键-keyc)
   - [6.11 蜂鸣器 `Buzzer.c`](#611-蜂鸣器-buzzerc)
   - [6.12 软件 I2C `MyI2C.c`](#612-软件-i2c-myi2cc)
7. [控制算法](#7-控制算法)
8. [主程序流程](#8-主程序流程)
9. [构建系统](#9-构建系统)
10. [目录结构](#10-目录结构)
11. [已知问题与注意事项](#11-已知问题与注意事项)
12. [调试接口](#12-调试接口)

---

## 1. 项目概述

本项目是 2024 年全国大学生电子设计竞赛 H 题「自动行驶小车」的参赛固件。小车搭载双轮差速驱动底盘，能够通过灰度传感器阵列实现**自主巡线行驶**，并通过 MPU6050 六轴 IMU 配合 Madgwick 姿态解算算法实现**航向角闭环控制**。两者交替协作，使小车在赛道上按锯齿形路径自动导航。

### 核心功能

| 功能 | 实现方式 |
|------|----------|
| **巡线行驶** | 8 路灰度传感器 + 模糊逻辑转向控制 |
| **航向控制** | MPU6050 IMU + Madgwick AHRS 姿态解算 + 目标偏航角闭环 |
| **速度闭环** | 霍尔编码器 + 增量式 PID（100ms 周期） |
| **人机交互** | 3 按键 + SSD1306 OLED 128×64 菜单显示 |
| **调试输出** | USART2 串口 @ 115200bps，中断接收 |

### 导航策略 (`runX()`)

小车按 4 个状态循环运行：

```
State 1: 旋转到目标航向角 (180° + 38.66°)
  ↓ (检测到黑线)
State 2: 巡线直行
  ↓ (脱离黑线，进入空白区)
State 3: 旋转到目标航向角 (360° - 38.66°)
  ↓ (检测到黑线)
State 4: 巡线直行
  ↓ (脱离黑线，回到 State 1)
```

---

## 2. 硬件总览

| 外设 / 模块 | 型号 / 说明 | 通信接口 | 占用引脚 |
|-------------|-------------|----------|----------|
| MCU | STM32F103RC（高密度） | — | — |
| 电机驱动 ×2 | L298N 兼容（方向 + PWM） | GPIO + PWM | PA6, PA7, PA11, PA12, PA14, PA15 |
| 霍尔编码器 ×2 | 13 PPR, 减速比 28:1 | 正交编码器 | PA0/PA1 (TIM2), PB6/PB7 (TIM4) |
| 6 轴 IMU | MPU6050（加速度 + 陀螺仪） | I2C（软件模拟） | PB8 (SCL), PB9 (SDA) |
| OLED 显示屏 | SSD1306 128×64 单色 | I2C（软件模拟） | PC12 (SCL), PB12 (SDA) |
| 灰度传感器 ×8 | 红外反射式数字输出 | GPIO 数字输入 | PA4, PA5, PA8, PB0, PB1, PB3, PC4, PC5 |
| 按键 ×3 | 微动开关 | GPIO 输入 | PB5, PC1, PD2 |
| 有源蜂鸣器 | 高低电平驱动（预留） | GPIO 输出 | PB12（与 OLED SDA 复用，实际未启用） |
| 串口调试 | USB-TTL 模块 | USART2 | PA2 (TX), PA3 (RX) |

---

## 3. 时钟树

配置在 [`Start/system_stm32f10x.c`](Start/system_stm32f10x.c) 的 `SetSysClockTo72()`：

```
HSE 8MHz 晶振
    │
    └── PLL (×9) ──→ 72MHz PLLCLK
                          │
                  ┌───────┴────────┐
                  │                │
              SYSCLK = 72MHz
                  │
        ┌─────────┼─────────┐
        │         │         │
    HCLK=72MHz  PCLK2=72MHz  PCLK1=36MHz
    (AHB 总线)  (APB2 总线)  (APB1 总线)
        │         │           │
    Cortex-M3  GPIOA~GPIOD   TIM2, TIM3
    Flash      USART2        TIM4
    DMA        TIM1          USART2
    FSMC       TIM8
```

| 时钟域 | 频率 | 挂载外设 |
|--------|------|----------|
| SYSCLK | **72 MHz** | Cortex-M3 内核 |
| HCLK (AHB) | **72 MHz** | Flash, DMA, FSMC, GPIO |
| PCLK2 (APB2) | **72 MHz** | TIM1 (PID 定时器), TIM8 (毫秒计数器), USART2 |
| PCLK1 (APB1) | **36 MHz** | TIM2 (左编码器), TIM3 (PWM), TIM4 (右编码器) |

### Flash 配置

- 等待周期: **2 个**（72MHz 下必需）
- 预取缓冲: **已开启**

---

## 4. 引脚分配表

| 引脚 | 标识 | 模式 | 外设功能 | 备注 |
|------|------|------|----------|------|
| PA0 | TIM2_CH1 | 输入上拉 | 左编码器 A 相 | — |
| PA1 | TIM2_CH2 | 输入上拉 | 左编码器 B 相 | — |
| PA2 | USART2_TX | AF 推挽 | 串口发送 | — |
| PA3 | USART2_RX | 输入上拉 | 串口接收 | 中断接收 |
| PA4 | D2 | 输入上拉 | 灰度传感器 2 | — |
| PA5 | D1 | 输入上拉 | 灰度传感器 1 | — |
| PA6 | TIM3_CH1 | AF 推挽 | 右电机 PWM (B) | — |
| PA7 | TIM3_CH2 | AF 推挽 | 左电机 PWM (A) | — |
| PA8 | D4 | 输入上拉 | 灰度传感器 4 | — |
| PA11 | AIN1 | 推挽输出 | 左电机方向 1 | — |
| PA12 | AIN2 | 推挽输出 | 左电机方向 2 | — |
| PA14 | BIN1 | 推挽输出 | 右电机方向 1 | SWJ 完全禁用，释放为 GPIO |
| PA15 | BIN2 | 推挽输出 | 右电机方向 2 | SWJ 完全禁用，释放为 GPIO |
| PB0 | D8 | 输入上拉 | 灰度传感器 8 | 已定义，init 中注释 |
| PB1 | D7 | 输入上拉 | 灰度传感器 7 | 已定义，init 中注释 |
| PB3 | D3 | 输入上拉 | 灰度传感器 3 | 已定义，init 中注释 |
| PB5 | KEY_ADD | 输入上拉 | 按键 1 (DEC) | — |
| PB6 | TIM4_CH1 | 输入上拉 | 右编码器 A 相 | — |
| PB7 | TIM4_CH2 | 输入上拉 | 右编码器 B 相 | — |
| PB8 | MyI2C_SCL | 开漏输出 | MPU6050 I2C 时钟 | — |
| PB9 | MyI2C_SDA | 开漏输出 | MPU6050 I2C 数据 | — |
| PB12 | OLED_SDA | 开漏输出 | OLED I2C 数据 | 与蜂鸣器冲突，蜂鸣器未启用 |
| PC1 | KEY_DEC | 输入下拉 | 按键 2 (INC) | — |
| PC4 | D6 | 输入上拉 | 灰度传感器 6 | — |
| PC5 | D5 | 输入上拉 | 灰度传感器 5 | — |
| PC12 | OLED_SCL | 开漏输出 | OLED I2C 时钟 | — |
| PD2 | KEY_SET | 输入下拉 | 按键 3 (确认) | — |

> **⚠️ SWJ 调试接口**：`Motor.c` 中调用了 `GPIO_Remap_SWJ_Disable`，完全禁用了 SWD/JTAG 接口，
> 以释放 PA13/PA14/PA15/PB3/PB4 用作普通 GPIO。**烧录时需先将 BOOT0 拉高进入 ISP 模式**，
> 否则调试器无法连接。

---

## 5. 外设详细配置

### 5.1 PWM — TIM3

用于驱动双路电机调速。

| 参数 | 值 |
|------|-----|
| 定时器 | TIM3（挂载 APB1, 36MHz） |
| 预分频器 | 36（即 PSC=35，寄存器值 36-1） |
| 自动重装载 | 4096（即 ARR=4095） |
| PWM 频率 | 36MHz / 36 / 4096 = **≈ 488 Hz** |
| 通道 1 (PA6) | 右电机 (B) 速度调节，PWM1 模式，高电平有效 |
| 通道 2 (PA7) | 左电机 (A) 速度调节，PWM1 模式，高电平有效 |
| 占空比范围 | 0 ~ 4096（对应 0% ~ 100%） |
| 模式 | 边沿对齐 |

**文件**: [`Hardware/PWM.c`](Hardware/PWM.c)

### 5.2 编码器 — TIM2 / TIM4

两路正交编码器，用于测量左右轮速度与位移。

| 参数 | TIM2（左编码器） | TIM4（右编码器） |
|------|------------------|-------------------|
| 通道 | CH1: PA0, CH2: PA1 | CH1: PB6, CH2: PB7 |
| 模式 | 编码器模式 3（双边沿 ×4） | 编码器模式 3（双边沿 ×4） |
| 预分频器 | 0（无分频） | 0（无分频） |
| 周期 | 65536（16 位满量程） | 65536（16 位满量程） |
| 溢出中断 | TIM2_IRQHandler | TIM4_IRQHandler |
| 输入滤波器 | 未配置 | 未配置 |

**编码器物理参数**（在 [`main.c`](User/main.c#L118) 中配置）：

| 参数 | 值 |
|------|-----|
| 编码器线数 (PPR) | 13 |
| 减速比 (Reduction) | 28:1 |
| 倍频系数 (Multiple) | 4（×4 解码） |
| 车轮半径 (Radius) | 0.065 m（65mm） |

**左侧轮子 `count_increment` 为正时表示前进；右侧轮子方向取反（硬件接线方向）**。

**文件**: [`Hardware/Encoder.c`](Hardware/Encoder.c)

### 5.3 系统定时器 — TIM1 / TIM8

| 参数 | TIM1（速度 PID 计时器） | TIM8（毫秒计数器） |
|------|--------------------------|---------------------|
| 预分频器 | 7200（即 PSC = 7199） | 72（即 PSC = 71） |
| 周期 | 1000（即 ARR = 999） | 1000（即 ARR = 999） |
| 中断频率 | **10 Hz**（每 100ms） | **1000 Hz**（每 1ms） |
| 中断服务 | TIM1_UP_IRQHandler | TIM8_UP_IRQHandler |
| 中断优先级 | 抢占 2, 子 2 | 抢占 2, 子 1 |
| 功能 | 读取编码器 + 执行 PID 更新 | 递增全局 `millis` 变量 |

**文件**: [`Hardware/Timer.c`](Hardware/Timer.c)

### 5.4 串口 — USART2

| 参数 | 值 |
|------|-----|
| 波特率 | **115200 bps** |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | 无 |
| 硬件流控 | 无 |
| TX 引脚 | PA2（AF 推挽） |
| RX 引脚 | PA3（输入上拉，中断接收） |
| 中断优先级 | 抢占 1, 子 2 |
| 接收协议 | `@MSG\r\n` 格式，状态机解析 |
| 发送 | `fputc` 重定向 + `Serial_Printf`（支持格式化） |

**文件**: [`Hardware/Serial.c`](Hardware/Serial.c)

### 5.5 I2C 总线（软件模拟）

项目使用**两个独立的软件模拟 I2C 总线**，各自采用 GPIO 开漏输出 + 延时方式实现。

| 参数 | I2C-1（MPU6050） | I2C-2（OLED SSD1306） |
|------|-------------------|------------------------|
| SCL | PB8 | PC12 |
| SDA | PB9 | PB12 |
| 速率 | ≈ 50 kHz（10μs 延时） | ≈ 50 kHz（10μs 延时） |
| 从机地址 | 0xD0（7-bit: 0x68 << 1） | 0x78（7-bit: 0x3C << 1） |
| 实现文件 | [`Hardware/MyI2C.c`](Hardware/MyI2C.c) | 内嵌在 [`Hardware/OLED.c`](Hardware/OLED.c) 中 |

> **⚠️ 设计缺陷**：两路 I2C 未共享代码，I2C 主机协议在 `MyI2C.c` 和 `OLED.c` 中各自实现了一遍。
> 复用时应注意提取公共逻辑。

### 5.6 GPIO 输入 / 按键

3 个按键，混合使用上下拉电阻，读取时采用 20ms 延时消抖。

| 逻辑标识 | 功能名 | 引脚 | 模式 | 有效电平 |
|----------|--------|------|------|----------|
| DECKey | 按键 — | PB5 | 输入上拉 | 低电平（接地导通） |
| ADDKey | 按键 + | PC1 | 输入下拉 | 高电平（VCC 导通） |
| SETKey | 按键确认 | PD2 | 输入下拉 | 高电平（VCC 导通） |

**文件**: [`Hardware/Key.c`](Hardware/Key.c)

### 5.7 SysTick 延时

不使用 SysTick 中断（`SysTick_Handler` 在 `stm32f10x_it.c` 中为空），而是**直接轮询 SysTick 计数寄存器**实现阻塞延时。

```c
void Delay_us(uint32_t xus);   // 微秒延时（72 个计数器周期/μs）
void Delay_ms(uint32_t xms);   // 毫秒延时
void Delay_s(uint32_t xs);     // 秒延时
```

**文件**: [`System/Delay.c`](System/Delay.c)

---

## 6. 软件模块说明

### 6.1 电机控制模块 `Motor.c`

- **功能**: 控制双路直流电机的方向和速度
- **输入**: `Motor_SetSpeedA(int16_t speed)` / `Motor_SetSpeedB(int16_t speed)`
- **方向控制**:
  - `speed > 0`: IN1=1, IN2=0（正转）
  - `speed < 0`: IN1=0, IN2=1（反转）
  - `speed = 0`: IN1=IN2=0（停止）
- **速度控制**: 通过调用 `PWM_SetCompare1/2` 设定 TIM3 占空比，值为 `abs(speed)`
- **⚠️**: 初始化时会调用 `GPIO_Remap_SWJ_Disable`，**彻底禁用 SWJ 调试接口**

### 6.2 PWM 模块 `PWM.c`

- **功能**: TIM3 双通道 PWM 输出，为电机提供速度控制
- **API**: `PWM_SetCompare1(uint16_t compare)` / `PWM_SetCompare2(uint16_t compare)`
- **范围**: 0 ~ 4096（自动饱和到 ARR）
- **频率**: ≈488 Hz

### 6.3 编码器模块 `Encoder.c`

- **功能**: 读取霍尔编码器，计算增量、速度、位置
- **数据结构**:
  ```c
  typedef struct {
      int64_t count_increment;  // 一个采样周期内的脉冲增量
      int64_t count_total;      // 累计脉冲数
      float position;           // 旋转圈数 / 距离(m) / 角度(rad)
      float velocity;           // 角速度 / 线速度
  } Encoder;
  ```
- **核心函数**:
  - `initEncoder(Encoder*, Parameter)` — 初始化编码器物理参数
  - `updateEncoderLoopSimpleVersion(Encoder*, uint32_t period_ms, TIM_TypeDef*)` — 读取并更新
- ⚠️ **右编码器方向**: 代码中对 TIM4 的计数值做了**取反**处理（`line 193`），补偿硬件接线方向

### 6.4 PID 控制器 `pid.c`

- **类型**: 增量式 PID
- **数据结构**:
  ```c
  typedef struct {
      float kp, ki, kd;                    // PID 参数
      float target, input, output;         // 目标、输入、输出
      Error error;                         // { now, last, integral }
      float MAX_OUTPUT;                    // 输出限幅
      float MAX_ERROR_INTEGRAL;            // 积分抗饱和限幅
  } PID;
  ```
- **API**:
  - `initPID(PID*, max_output, max_error_integral)`  — 初始化限幅
  - `setPIDParam(PID*, kp, ki, kd)` — 设置 PID 参数
  - `setPIDTarget(PID*, target)` — 设置目标值
  - `updatePID(PID*, input)` — 执行 PID 计算，结果写入 `output`
- **手动整定参数**:
  - 两轮参数相同: `Kp=7.5, Ki=0.7, Kd=0.4`
  - 最大输出: 2100
  - 积分上限: 5000

### 6.5 MPU6050 驱动 `MPU6050.c`

- **通信**: 软件 I2C (PB8/PB9)，从机地址 0x68
- **初始化配置**:
  | 寄存器 | 值 | 说明 |
  |--------|-----|------|
  | PWR_MGMT_1 | 0x01 | 退出睡眠，时钟源 = X 轴陀螺仪 |
  | SMPLRT_DIV | 9 | 采样率 = 1kHz / (1+9) = **100 Hz** |
  | CONFIG (DLPF) | 0x06 | 低通滤波 5Hz 带宽 |
  | GYRO_CONFIG | 0x18 | 满量程 ±2000°/s |
  | ACCEL_CONFIG | 0x18 | 满量程 ±16g |
- **校准流程**: `dataGetERROR()` 在启动时采集 100 个样本取平均，得到零偏值
- **单位换算**:
  - 加速度: `raw / 32768 * 16 * 9.8` → m/s²
  - 角速度: `raw / 32768 * 2000 * 3.5 / 1.244` → °/s（含额外校准系数）

**文件**: [`Hardware/MPU6050.c`](Hardware/MPU6050.c)、[`Hardware/MPU6050_Reg.h`](Hardware/MPU6050_Reg.h)

### 6.6 姿态解算 `MadgwickAHRS.c`

- **算法**: Sebastian Madgwick 的梯度下降 IMU 融合算法（无磁力计版本）
- **输入**: 三轴加速度计 + 三轴陀螺仪（100Hz 更新率）
- **输出**: Roll / Pitch / Yaw（欧拉角，度）
- **参数**:
  - Beta（梯度下降步长）: **0.1**
  - 采样频率: **100 Hz**（`begin(1000.0f / dt)` 初始化时设定）
- **Yaw 输出范围**: 0° ~ 360°（原始 -180~+180 值 + 180）

**文件**: [`Hardware/MadgwickAHRS.c`](Hardware/MadgwickAHRS.c)

### 6.7 OLED 显示 `OLED.c`

- **型号**: SSD1306 128×64 单色
- **通信**: 软件 I2C（PC12 时钟, PB12 数据），地址 0x78
- **字符显示**: 8×16 像素 ASCII 字库 (`OLED_Font.h`)
- **行布局**: 4 行 × 16 字符（第 1~4 行）
- **API**:
  - `OLED_ShowString(line, col, str)` — 显示字符串
  - `OLED_ShowNum(line, col, num, len)` — 显示无符号整数
  - `OLED_ShowSignedNum(line, col, num, len)` — 显示有符号整数
  - `OLED_Clear()` — 清屏

**文件**: [`Hardware/OLED.c`](Hardware/OLED.c)、[`Hardware/OLED_Font.h`](Hardware/OLED_Font.h)

### 6.8 灰度巡线 `gray_track.c`

- **传感器**: 8 通道红外灰度数字传感器，黑线 = 0（触发），白色 = 1（未触发）
- **传感器编号与引脚对应**:

  | 编号 | 引脚 | 初始化状态 |
  |------|------|------------|
  | D1 | PA5 | ✅ 已初始化 |
  | D2 | PA4 | ✅ 已初始化 |
  | D3 | PB3 | ⚠️ 已定义，注释中 |
  | D4 | PA8 | ✅ 已初始化 |
  | D5 | PC5 | ✅ 已初始化 |
  | D6 | PC4 | ✅ 已初始化 |
  | D7 | PB1 | ⚠️ 已定义，注释中 |
  | D8 | PB0 | ⚠️ 已定义，注释中 |

- **巡线算法** (`track()`): 基于传感器位置的模糊逻辑控制器
  - 中间传感器 (D4/D5) 无信号 → 直行 (100, 100)
  - 左侧传感器触发 → 右转补偿（左速度快，右速度慢）
  - 右侧传感器触发 → 左转补偿
  - 偏离越大，速度差越大（最多 180 vs 40）

- **⚠️ 已知问题**: D3/D7/D8 的 GPIO 初始化代码被注释掉了，实际仅使用 5 路传感器 (D1/D2/D4/D5/D6)

### 6.9 串口通信 `Serial.c`

- **UART**: USART2
- **发送**: `fputc` 重定向实现 `printf` 支持 + `Serial_Printf` 格式化输出
- **接收**: 中断驱动，状态机解析 `@` 为帧头的变长 ASCII 报文（`\r\n` 结尾）
- **接收缓冲区**: `Serial_RxPacket` 结构

### 6.10 按键 `Key.c`

- **3 按键** + 主菜单/子菜单两级交互
- **主菜单**: 翻页选择功能（"1 Speed" / "2 Test"）
- **子菜单**: 执行对应功能
  - Menu 1 (Speed): 显示编码器增量 + 执行巡线
  - Menu 2 (Test): 显示运行毫秒 + 偏航角 + 执行 `runX()`
- **消抖**: 20ms 阻塞延时（⚠️ 会阻塞主循环）

### 6.11 蜂鸣器 `Buzzer.c`

- **引脚**: PB12（推挽输出）
- **状态**: 代码完整（ON/OFF/Toggle），但 **`Buzzer_Init()` 在 `main.c` 中未被调用**
- ⚠️ 与 OLED SDA 共用 PB12，不可同时使用

### 6.12 软件 I2C `MyI2C.c`

- **引脚**: PB8 (SCL), PB9 (SDA)，开漏输出
- **时序**: 10μs 延时 → ≈50 kHz 时钟
- **协议**: 完整的 I2C 主机操作（Start/Stop/SendByte/ReadByte/Ack/Nack）
- **用途**: 专为 MPU6050 通信设计

---

## 7. 控制算法

### 7.1 速度闭环 — 增量式 PID

```
error = target - input
P = Kp * (error)
I = Ki * (error + error_last)    // 累加（限幅）
D = Kd * (error - error_last)
output += P + I + D              // 增量叠加（输出限幅）
error_last = error
```

### 7.2 航向闭环 — targetYaw()

```c
error_yaw = target - yaw
// 处理 0°/360° 环绕问题
if (error_yaw >= 270)      error_yaw = -yaw - (360 - target)
if (error_yaw <= -270)     error_yaw = target + 360 - yaw

// 死区 ±1°
if (error_yaw > 1)         → 原地右转 (左轮40, 右轮0)
else if (error_yaw < -1)   → 原地左转 (左轮0, 右轮40)
else                       → 停止旋转 (100, 100)
```

### 7.3 巡线控制 — 模糊逻辑

基于灰度传感器位置的分段速度调节，偏离中心越远则两侧速度差越大，实现平滑转向。

### 7.4 Madgwick AHRS — 姿态融合

梯度下降算法融合陀螺仪积分和加速度计方向观测值，输出四元数并转换为欧拉角。无磁力计，Yaw 仅靠陀螺仪积分（无绝对参考），**长时间运行会有漂移**。

---

## 8. 主程序流程

```
main()
│
├─ 硬件初始化
│   ├─ OLED_Init()           → OLED 显示就绪
│   ├─ begin(100Hz)          → Madgwick 滤波器初始化
│   ├─ Motor_Init()          → 电机 GPIO + PWM + SWJ 禁用
│   ├─ MPU6050_Init()        → IMU 唤醒与配置
│   ├─ Encoder_Init_TIM_All()→ TIM2 + TIM4 编码器模式
│   ├─ Timer_Init()          → TIM1 (100ms PID 中断)
│   ├─ TIM8_Init()           → TIM8 (1ms 毫秒中断)
│   ├─ Delay_ms(5000)        → 等待外设稳定
│   ├─ dataGetERROR()        → MPU6050 100 样本校准
│   ├─ Key_Init()            → 3 按键初始化
│   ├─ Serial_Init()         → USART2 串口初始化
│   └─ gray_init()           → 灰度传感器 GPIO
│
├─ 软件初始化
│   └─ myCarControlCodeInit()
│       ├─ 编码器参数: 4x, 28:1, 13 PPR, 0.065m 轮径
│       └─ PID 参数: Kp=7.5, Ki=0.7, Kd=0.4, Out=2100
│
└─ while(1) 主循环
    ├─ Motor_SetSpeedA/B()              ← 用 PID 输出值更新电机
    ├─ [10ms 间隔] MPU6050 读取 + 滤波 + Madgwick 姿态更新
    ├─ 菜单显示刷新（仅在页面/标志变化时）
    ├─ 按键扫描
    └─ 子菜单执行（巡线 / runX 自动驾驶）

--- 中断服务 ---

TIM8_UP_IRQHandler (1ms)
    └─ millis++

TIM1_UP_IRQHandler (100ms)
    ├─ updateEncoderLoopSimpleVersion()  ← 读取编码器增量
    ├─ updatePID()                       ← 计算 PID 输出
    └─ 清除中断标志

USART2_IRQHandler
    └─ 状态机解析串口接收数据
```

---

## 9. 构建系统

### Keil MDK-ARM（主构建方式）

| 项目 | 详情 |
|------|------|
| 工程文件 | [`Project.uvprojx`](Project.uvprojx) |
| 编译器 | ARMCC V5.06 update 5 (build 528) |
| 器件包 | Keil.STM32F1xx_DFP.2.4.1 |
| 预定义宏 | `STM32F10X_HD, USE_STDPERIPH_DRIVER` |
| 输出 | `Objects/Project.axf` |

### CMake + ARM GCC（备选构建）

| 项目 | 详情 |
|------|------|
| 配置文件 | [`CMakeLists.txt`](CMakeLists.txt) |
| 工具链 | [`arm-gcc-toolchain.cmake`](arm-gcc-toolchain.cmake) |
| 编译器 | `arm-none-eabi-gcc` |
| 编译选项 | `-mcpu=cortex-m3 -mthumb -g -O0` |
| 预定义宏 | `STM32F10X_MD, STM32F10X_HD, USE_STDPERIPH_DRIVER` |

> ⚠️ CMake 同时定义了 `STM32F10X_MD` 和 `STM32F10X_HD`，这是不正确的；应为 `STM32F10X_HD` 一处。

### 串口下载（ISP）

由于 SWJ 被禁用，需通过 **USART1 ISP** 方式烧录：
1. BOOT0 拉高，BOOT1 拉低
2. 复位进入系统 Bootloader
3. 使用 `flymcu` 或 `stm32flash` 通过 USART1 烧录

---

## 10. 目录结构

```
STM32Car_H/
├── README.md                    # 项目简介（中文）
├── PROJECT_REFERENCE.md         # ← 本文件
├── LICENSE
├── Project.uvprojx              # Keil MDK 工程
├── Project.uvoptx               # Keil MDK 工程选项
├── CMakeLists.txt               # CMake 构建
├── arm-gcc-toolchain.cmake      # ARM GCC 交叉编译工具链
├── .clang-format                # 代码格式化规则
│
├── Start/                       # CMSIS + 启动文件
│   ├── startup_stm32f10x_hd.s   # 高密度启动汇编
│   ├── core_cm3.c / .h          # Cortex-M3 内核访问
│   ├── stm32f10x.h              # 器件头文件
│   └── system_stm32f10x.c / .h  # 系统时钟初始化
│
├── Library/                     # STM32F10x SPL V3.5.0
│   ├── misc.c / .h              # NVIC / SysTick 辅助
│   └── stm32f10x_*.c / .h       # 全部外设驱动
│
├── User/                        # 应用层
│   ├── main.c                   # 主程序
│   ├── stm32f10x_conf.h         # SPL 外设使能配置
│   └── stm32f10x_it.c / .h      # 中断处理（实际实现分散在各模块）
│
├── Hardware/                    # 硬件抽象层
│   ├── Motor.c / .h             # 电机驱动
│   ├── PWM.c / .h               # PWM 输出
│   ├── Encoder.c / .h           # 编码器读取
│   ├── pid.c / .h               # PID 控制器
│   ├── Timer.c / .h             # TIM1/8 定时器
│   ├── MPU6050.c / .h           # MPU6050 IMU 驱动
│   ├── MPU6050_Reg.h            # MPU6050 寄存器定义
│   ├── MadgwickAHRS.c / .h      # 姿态解算
│   ├── OLED.c / .h              # SSD1306 OLED
│   ├── OLED_Font.h              # OLED 字库 (8×16)
│   ├── MyI2C.c / .h             # 软件 I2C
│   ├── Serial.c / .h            # 串口通信
│   ├── Key.c / .h               # 按键输入
│   ├── gray_track.c / .h        # 灰度巡线
│   └── Buzzer.c / .h            # 蜂鸣器（未启用）
│
├── System/
│   └── Delay.c / .h             # 系统延时
│
├── .cmsis/                      # CMSIS 公共文件
├── Objects/ / Listings/ / build/# 构建产物
└── DebugConfig/                 # 调试配置
```

---

## 11. 已知问题与注意事项

| 编号 | 问题 | 影响 | 建议 |
|------|------|------|------|
| 1 | **SWJ 完全禁用**（`GPIO_Remap_SWJ_Disable`） | 无法通过 SWD/JTAG 调试或烧录 | ISP 模式烧录，或改为 `GPIO_Remap_SWJ_JTAGDisable` 保留 SWD |
| 2 | **PB12 引脚复用** | OLED SDA 与蜂鸣器共享 PB12 | 蜂鸣器未启用，无实际影响；如需使用蜂鸣器，换其他引脚 |
| 3 | **灰度传感器 D3/D7/D8 未初始化** | `gray_init()` 中对应 GPIO 初始化被注释 | 实际仅 5 路有效；需要 8 路时取消注释 |
| 4 | **CMake 宏冲突** | 同时定义 `STM32F10X_MD` 和 `STM32F10X_HD` | 删掉 `STM32F10X_MD`，保留 `STM32F10X_HD` |
| 5 | **文件夹命名误导** | 路径名含 `f407zgt6`/`STM32HAL`，但实际是 F103RC + SPL | 文件夹命名与内容不符 |
| 6 | **按键延时消抖阻塞主循环** | `Key_GetNum()` 中有 20ms 阻塞延时 | 对实时性要求高的控制可能造成抖动，建议改为非阻塞状态机消抖 |
| 7 | **Yaw 无绝对参考** | Madgwick 无磁力计，Yaw 仅靠陀螺仪积分 | 长时间运行 Yaw 会漂移，可通过巡线传感器进行周期性校正 |
| 8 | **软件 I2C 代码重复** | `MyI2C.c` 和 `OLED.c` 各自实现了一套 I2C 主机协议 | 可提取公共 I2C 层以减少代码冗余 |
| 9 | **PID 参数是硬编码** | 整定值写在 `myCarControlCodeInit()` 中 | 可考虑支持运行中按键调整参数以方便调试 |
| 10 | **编码器参数轮径 0.065m** | 轮子半径 65mm，但文中注释的位置计算使用了该值 | 确认实测轮径与代码一致，否则位移误差会累积 |

---

## 12. 调试接口

| 接口 | 用途 | 引脚 | 参数 |
|------|------|------|------|
| **USART2 串口** | printf 调试输出 / 命令行 | PA2(TX), PA3(RX) | 115200-8-N-1 |
| **SWD** | 烧录与调试 | PA13(SWDIO), PA14(SWCLK) | ⚠️ 已被 `GPIO_Remap_SWJ_Disable` 禁用 |
| **USART1 ISP** | 系统 Bootloader 烧录 | PA9(TX), PA10(RX) | BOOT0=1, BOOT1=0 |

### 恢复 SWD 调试能力

如需恢复 SWD 调试，将 `Hardware/Motor.c` 第 34 行的：

```c
GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
```

改为：

```c
GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);  // 仅禁用 JTAG，保留 SWD
```

这样 PA14/PA15 仍可用作 GPIO（SWCLK/SWDIO 保留在 PA13/PA14），但至少保留了 SWD 调试通路。

> ⚠️ 注意改完后 PA14(SWCLK) 仍会被用作 BIN1 输出，可能需要在调试和运行态之间切换。

---

*文档生成时间: 2026-05-31 | 基于源码分析自动生成*
