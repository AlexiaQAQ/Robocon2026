# Robocon 2026 — R2 主控固件 (自制 A 板)

[![MCU](https://img.shields.io/badge/MCU-STM32F407VET6-03234B?logo=stmicroelectronics)](https://www.st.com/en/microcontrollers-microprocessors/stm32f407ve.html)
[![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-6DB33F)](https://www.freertos.org/)
[![IDE](https://img.shields.io/badge/IDE-VSCode%20%2B%20EIDE%20%2F%20Keil-007ACC)](https://code.visualstudio.com/)
[![Board](https://img.shields.io/badge/Board-自制A板-555555)]()
[![Award](https://img.shields.io/badge/Award-国家一等奖-FFD700)]()

> 📖 [English version](README.en.md)

## 概述

本项目为 Robocon 2026 赛季 R2 机器人的主控制板固件，基于 **STM32F407VET6** (Cortex-M4F, 168MHz, 512KB Flash, 192KB SRAM) + **FreeRTOS** 实时操作系统开发。

> **硬件平台**: 自制 A 板（与 R1 上层主控相同），非 Robomaster 开发板 A 型。

机器人采用**四全向轮底盘 + 升降机构 + 左右双机械臂**的结构，主控负责底层驱动（电机控制、电磁阀控制），通过 SBUS 遥控器直连控制，或接收上位机串口指令实现对底盘和机械臂的控制。

> **2026-07-06 USART1 IDLE防半帧 + 100Hz DMA发送 + SBUS丢帧保护** | 2026-06-28 时序修正

---

## 硬件架构

### 主控芯片
- **MCU**: STM32F407VET6 (Cortex-M4F, 168MHz, 512KB Flash, 192KB SRAM)

### 外设资源与引脚映射

| 外设 | 引脚 | 用途 |
|------|------|------|
| CAN1 | PD0/PD1 | 底盘: 全向轮×4 (DM_3519 MIT) + 独立抬升×4 (DM_4310 POS) |
| CAN2 | PB5/PB6 | 左右双机械臂 DM_4340 ×6 (位置模式) |
| SPI1 + PA4 (CS) | PA5/PA6/PA7 | MCP2515 → hcan3，夹爪翻转 DM + 电磁阀 YV1~YV3 |
| SPI2 + PB12 (CS) | PB13/PB14/PB15 | MCP2515 → hcan4，预留 |
| SPI3 + PA15 (CS) | PB3/PB4/PB5 | MCP2515 → hcan5，预留 |
| UART4 | PA0(TX)/PA1(RX) | SBUS 遥控器接收 (100kbps, 9-bit, Even, 2-stop) |
| USART1 | PA9(TX)/PA10(RX) | 上位机→电控: 控制帧 30B (DMA, 50Hz) |
| USART2 | PD5(TX)/PD6(RX) | 电控→上位机: 状态帧 23B (50Hz) |
| USART3 | PD8(TX)/PD9(RX) | 预留: 串口陀螺仪 |
| TIM14 | — | FreeRTOS 系统时基 |
| GPIOE[0:7] | PE0~PE7 | LED 流水灯 |
| GPIOE[9] | PE9 | 前轮前光电 (输入) |
| GPIOE[11] | PE11 | 前轮后光电 (输入) |
| GPIOE[13] | PE13 | 后轮前光电 (输入) |
| GPIOE[14] | PE14 | 后轮后光电 (输入) |

### 执行器

| 执行器 | 数量 | 控制总线 | 电机型号/模式 | 说明 |
|--------|------|----------|--------------|------|
| 全向轮电机 | 4 | CAN1 (ID 1~4) | DM_3519 MIT | 四全向轮驱动 |
| 独立抬升电机 | 4 | CAN1 (ID 5~8) | DM_4310 POS | 齿条抬升, 满行程 420mm |
| 左机械臂电机 | 3 | CAN2 (ID 1~3) | DM_4340 POS | 左臂 pitch1~3 |
| 右机械臂电机 | 3 | CAN2 (ID 4~6) | DM_4340 POS | 右臂 pitch1~3 |
| 夹爪翻转电机 | 1 | hcan3 (MCP2515) | DM_4310 POS | 0=竖起, 1.57=翻下 |
| 夹爪气缸 | 1 | hcan3 | 电磁阀 YV1 | 1=开, 0=关 |
| 左吸盘 | 1 | hcan3 | 电磁阀 YV2 | 1=吸气, 0=松开 |
| 右吸盘 | 1 | hcan3 | 电磁阀 YV3 | 1=吸气, 0=松开 |

---

## 软件架构

### 目录结构

```
R2/main_body/
├── Core/Src/               # CubeMX 生成 + 主逻辑
│   ├── main.c              # 主程序：任务创建 + 控制逻辑
│   ├── freertos.c          # FreeRTOS 配置
│   ├── can.c / dma.c / gpio.c / spi.c / usart.c
│   └── stm32f4xx_it.c      # 中断服务 (CAN1/2 接收回调)
├── Lib/                    # 应用层库
│   ├── dm_motor.c/h        # DM 电机控制库 (MIT/POS/SPD/PSI + MCP2515)
│   ├── sbus_set.c/h        # SBUS 遥控器解码 (UART4 DMA)
│   ├── bsp_can.c/h         # CAN 滤波器初始化
│   ├── mcp2515.c/h         # MCP2515 SPI-CAN 驱动
│   ├── CAN_receive.c/h     # CAN 数据接收 + dm_rx_cbk 分发
│   ├── chassis.c/h         # 全向轮运动学 + 独立抬升控制
│   ├── arm.c/h             # 左右双机械臂控制
│   ├── solenoid_valves.c/h # 电磁阀 YV1/YV2/YV3
│   ├── uart_task.c/h       # 串口协议解析 + 状态帧组装
│   └── imu.c/h             # IMU 欧拉角解析 (未启用)
├── rc26_vehicle_serial_protocol_final.md  # 串口协议定稿
├── Drivers/                # STM32 HAL + CMSIS
├── Middlewares/            # FreeRTOS 源码
└── MDK-ARM/                # Keil 工程文件
```

### FreeRTOS 任务

| 任务 | 周期 | 栈 | 优先级 | 功能 |
|------|------|----|--------|------|
| `start_task` | 一次性 | 256 | 0 | 初始化外设, 创建子任务后自销毁 |
| `sbus_task` | 10ms | 256 | 1 | SBUS 解码 + 遥控映射 + CH4 使能 + 模式切换 |
| `uart_task` | 10ms(100Hz) | 1024 | 0 | USART1 IDLE解析 + USART2 DMA状态帧 |
| `chassis_task` | ~2ms | 512 | 0 | 全向轮运动学 + CAN1 MIT 驱动 |
| `up_cs_task` | 50ms | 512 | 0 | 抬升 + 夹爪翻转 + 电磁阀刷新 |
| `arm_task` | 10ms | 512 | 0 | 左右双机械臂位置控制 (各关节独立速度) |
| `led_task` | 200ms | 128 | 2 | LED 流水灯 (PE0→PE7) |

---

## 控制模式

### CH6 旋钮档位 + CH7 拨杆触发

| CH6 范围 | CH7 沿触发 | 功能 |
|----------|-----------|------|
| < 632 (低档) | ↑↓ | 翻转电机 0↔1.57 |
| 632~1415 (中档) | ↑↓ | 夹爪气缸 YV1 开↔关 |
| >1700 (升降/吸盘) | ↑↓ | 左右吸盘 YV2+YV3 同时取反 |

### 手动/自动模式

```
CH4 (主使能, 阈值1700)
├── OFF → 所有子系统失能, 速度归零
└── ON
    ├── CH5=高 → 手动遥控
    │   ├── CH0 → vw
    │   ├── CH2 → vx
    │   ├── CH3 → vy
    │   ├── CH6高(>1700) → CH8 同时控制前后升降 (0~420mm)
    │   └── CH6+CH7 → 翻转/夹爪/吸盘 (见上表)
    └── CH5=低 → 自动模式 (上位机串口)
        ├── USART1: 控制帧 30B (50Hz)
        └── USART2: 状态帧 23B (50Hz)
```

### 速度限幅

| 参数 | 范围 | 说明 |
|------|------|------|
| vx, vy | ±20000 (±2 m/s) | 串口解析 + 运动学双重限幅 |
| vw | ±32000 (±3.2 rad/s) | 同上 |
| 升降高度 | 0~420 mm | 齿条满行程 |
| 机械臂 pitch | ±3141 mrad (±180°) | 硬限幅保护 |

### 升降回传

升降实际高度从 `dm_motor[4~7]` 编码器实时计算:
- 前升降 = (FR + FL) / 2, 后升降 = (BL + BR) / 2
- 未使能返回 `0xFFFF`
- POS 电机反馈 ±12.5rad 单圈, 超出后软件解卷绕 (帧间追踪, 首帧 target 锚定)

### 机械臂关节速度

各关节独立配置 (rad/s), 定义在 `arm.h`:

| 关节 | slow | fast | 说明 |
|------|------|------|------|
| 左根 (L1) / 右根 (R1) | 0.5 | 0.85 | 根部, 惯量大, 宜慢 |
| 左肘 (L2) / 右肘 (R2) | 0.85 | 1.35 | 肘部, 可稍快 |
| 左腕 (L3) / 右腕 (R3) | 0.5 | 0.85 | 末端, 宜稳 |

`arm_left_update(hcan, fast)` / `arm_right_update(hcan, fast)` 自动按模式选速。

---

## 通信协议

详见 [rc26_vehicle_serial_protocol_final.md](rc26_vehicle_serial_protocol_final.md)

### 控制帧 (上位机→电控, USART1, 30B)

| 字节 | 字段 | 类型 |
|------|------|------|
| 0 | 0xCC | 帧头 |
| 1-2 | vx | int16, 10000=1m/s |
| 3-4 | vy | int16 |
| 5-6 | wz | int16, 10000=1rad/s |
| 7-8 | lift_front_target | uint16, mm |
| 9-10 | lift_back_target | uint16, mm |
| 11 | lift_mode | 0=normal, 1=fast |
| 12-23 | 左右臂 6×pitch | int16, mrad |
| 24-25 | 左右吸盘 | uint8 |
| 26-27 | weapon_pitch/gripper | uint8 |
| 28 | reserved | 0 |
| 29 | 0xEE | 帧尾 |

### 状态帧 (电控→上位机, USART2, 23B)

| 字节 | 字段 | 类型 |
|------|------|------|
| 0 | 0xCC | 帧头 |
| 1-2 | lift_front_actual | uint16, mm (编码器实测) |
| 3-4 | lift_back_actual | uint16, mm (编码器实测) |
| 5-8 | 光电/TOF 占位 | 0x00 (独立 MCU 处理) |
| 9-20 | 左右臂 6×pitch 当前值 | int16, mrad |
| 21 | reserved | 0 |
| 22 | 0xEE | 帧尾 |

### 底盘运动学

45° 全向轮逆运动学 (chassis_update):

```
t_scale = 1 / (r × cos45°)    // 补偿 COS45 使实际车速 = vx
ω_i = ±vx × t_scale ± vy × t_scale + vw / r
```

- `SPEED_SCALE = 0.0001` (协议 10000 = 1 m/s)
- `WHEEL_RADIUS = 0.064m` (直径 127mm)
- `CHASSIS_R = 0.245m`
- `CHASSIS_TORQUE = 7.0` (MIT kd 阻尼)
- 发送顺序: BR(1)→FR(2)→BL(4)→FL(3)

### USART3

预留串口陀螺仪接口，当前未启用。

---

## 安全机制

1. **SBUS 帧校验**: 字节 0 (0x0F) + 字节 23/24 (0x00)
2. **SBUS 超时断联**: 100ms 无有效帧 → 全部失能
3. **CH4 边沿检测**: 上升沿使能, 下降沿失能 + 停车
4. **速度限幅**: vx/vy ±2m/s, vw ±3.2rad/s (双重)
5. **升降限幅**: 0~420mm
6. **机械臂限幅**: ±180° (±3141 mrad)
7. **DMA 异常复位**: 帧头尾校验失败自动重置
8. **夹爪翻转保护**: `gripper_flip_ready` 标志, 首次 CH7 操作后才动作
9. **CH4 失能安全释放**: YV1 夹紧 + YV2/YV3 松开, 先发后失能 hcan3
10. **CH4 使能时序**: 先使能电机, 后置 `sys_enabled` 标志, 防止控帧先于使能到达导致电机异常
11. **USART1 IDLE 中断**: 仅完整接收一帧后才解析, 杜绝 DMA 写入中途读到半帧导致吸盘/翻转误动
12. **SBUS 防护**: 所有 `ch_high()` 加 `sbus_frame_valid()` 守卫; `last_ch7_state` 仅有效帧更新; 状态帧发送不依赖 SBUS (防丢帧断流)

---

## 相关链接

| 资源 | 链接 |
|------|------|
| 串口协议 | [rc26_vehicle_serial_protocol_final.md](rc26_vehicle_serial_protocol_final.md) |
| 英文文档 | [README.en.md](README.en.md) |
