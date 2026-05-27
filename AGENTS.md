# STM32Car_H — 两轮平衡小车

## 硬件平台

| 项 | 值 |
|----|-----|
| MCU | STM32F103VE (High Density) |
| 启动文件 | `Start/startup_stm32f10x_hd.s` |
| IDE | Keil MDK (`Project.uvprojx`) |
| 库 | 标准外设库 (Standard Peripherals Library) |

## 外设引脚总览

### 电机驱动 (DRV8833 / TB6612 双路)
| 信号 | 引脚 | 说明 |
|------|------|------|
| AIN1 | PA11 | 左电机方向1 |
| AIN2 | PA12 | 左电机方向2 |
| PWMA | PA7 (TIM3_CH2) | 左电机 PWM, ARR=4095, PSC=35 |
| BIN1 | PA14 | 右电机方向1 |
| BIN2 | PA15 | 右电机方向2 |
| PWMB | PA6 (TIM3_CH1) | 右电机 PWM |

PA14/PA15 默认是 SWCLK/SWDIO，代码禁用了 SWD (`GPIO_Remap_SWJ_Disable`)，烧录时需要手动复位进入下载模式。

### 编码器
| 编码器 | 定时器 | 引脚 | 模式 |
|--------|--------|------|------|
| 左轮 (Motor A) | TIM2 | PA0, PA1 | 编码器接口模式 |
| 右轮 (Motor B) | TIM4 | PB6, PB7 | 编码器接口模式 |

参数：PPR=500 (GMR编码器), 减速比=30, 轮半径=0.0325m, 4倍频

### MPU6050 (I2C, 软件模拟)
| 信号 | 引脚 |
|------|------|
| SCL | PB8 |
| SDA | PB9 |

使用软件 I2C (MyI2C)，Madgwick AHRS 姿态解算，更新频率 100Hz (dt=10ms)

### OLED 显示屏 (I2C)
与 MPU6050 共用 PB8/PB9 软件 I2C 总线。

### 串口 (USART2)
| 信号 | 引脚 |
|------|------|
| TX | PA2 |
| RX | PA3 |
| 波特率 | 115200 |

### 灰度传感器 (8路)
| 通道 | 引脚 |
|------|------|
| D1 | PA5 |
| D2 | PA4 |
| D3 | PB3 |
| D4 | PA8 |
| D5 | PC5 |
| D6 | PC4 |
| D7 | PB1 |
| D8 | PB0 |

灰度初始化时禁用了 JTAG (`GPIO_Remap_SWJ_JTAGDisable`)，PB3 作为普通 IO。

### 按键
| 功能 | 引脚 | 触发方式 |
|------|------|----------|
| Key1 (DEC) | PB5 | 上拉，低电平触发 |
| Key2 (ADD) | PC1 | 下拉，高电平触发 |
| Key3 (SET) | PD2 | 下拉，高电平触发 |

### 蜂鸣器
| 引脚 | 说明 |
|------|------|
| 待确认 | Buzzer.c |

### 定时器
| 定时器 | 用途 | 周期 |
|--------|------|------|
| TIM1 | 速度环 PID + 编码器更新 | 100ms |
| TIM2 | 左轮编码器接口 | — |
| TIM3 | 电机 PWM (CH1 + CH2) | — |
| TIM4 | 右轮编码器接口 | — |
| TIM8 | millis 时间戳 | 1ms |

## 项目结构

```
STM32Car_H/
├── User/main.c              # 主程序 (状态机 + 菜单 + MPU6050)
├── Hardware/                # 外设驱动层
│   ├── Motor.c/h            # 电机驱动
│   ├── Encoder.c/h          # 编码器 (速度/位置解算)
│   ├── pid.c/h              # PID 控制器
│   ├── MPU6050.c/h          # MPU6050 传感器
│   ├── MPU6050_Reg.h        # MPU6050 寄存器定义
│   ├── MadgwickAHRS.c/h     # Madgwick 姿态解算
│   ├── MyI2C.c/h            # 软件 I2C
│   ├── OLED.c/h             # OLED 显示
│   ├── gray_track.c/h       # 灰度巡线
│   ├── PWM.c/h              # PWM 驱动
│   ├── Serial.c/h           # 串口 (USART2)
│   ├── Timer.c/h            # 定时器初始化
│   ├── Key.c/h              # 按键
│   └── Buzzer.c/h           # 蜂鸣器
├── Start/                   # 启动文件 + CMSIS
│   ├── startup_stm32f10x_hd.s
│   ├── system_stm32f10x.c/h
│   └── core_cm3.c/h
├── Library/                 # 标准外设库
│   └── stm32f10x_*.c/h     # GPIO/TIM/USART/I2C 等
└── Project.uvprojx          # Keil MDK 工程文件
```

## 核心算法

### PID 控制 (pid.c)
- 位置式 PID，带积分分离 (MAX_ERROR_INTEGRAL)
- 速度环：左右轮独立 PID，kp=7.5, ki=0.7, kd=0.4
- 输出限幅：±2100

### 姿态解算 (Madgwick)
- Madgwick AHRS 算法
- 通过 `getYaw()` 获取偏航角 (0-360°)
- Yaw 角用于闭环转向控制 (`targetYaw()`)

### 小车运行状态机 (runX)
```
carFlag 1: 前进到 angle+180 方向 → 碰到灰线 → carFlag 2
carFlag 2: 巡线 → 离开灰线 → carFlag 3
carFlag 3: 前进到 360-angle 方向 → 碰到灰线 → carFlag 4
carFlag 4: 巡线 → 离开灰线 → carFlag 1 (循环)
```

## 编译与烧录

```bash
# Keil CLI 编译 (需要 Keil 安装)
UV4.exe -b Project.uvprojx -j0 -o build.log
```

烧录前注意：代码禁用了 SWD，需拉低 BOOT0 后用串口烧录，或在复位瞬间用 J-Link 连接。

## 已知问题 / TODO

- PID 参数可能需进一步整定
- 姿态角控制精度待验证
- PA14/PA15 禁用 SWD 后下载不便，考虑改用 PB6/PB7 驱动电机方向
- gray_track.c 中 D3/D7/D8 初始化被注释，传感器可能不全
