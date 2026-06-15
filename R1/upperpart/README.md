# Robocon 2026 — R1 上层主控固件 (自制 A 板)

[![MCU](https://img.shields.io/badge/MCU-STM32F407VET6-03234B?logo=stmicroelectronics)](https://www.st.com/en/microcontrollers-microprocessors/stm32f407ve.html)
[![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-6DB33F)](https://www.freertos.org/)
[![IDE](https://img.shields.io/badge/IDE-VSCode%20%2B%20EIDE%20%2F%20Keil-007ACC)](https://code.visualstudio.com/)
[![Board](https://img.shields.io/badge/Board-自制A板-555555)]()

## 概述

本项目为 Robocon 2026 赛季 R1 机器人**上层**主控制板固件，基于 **STM32F407VET6** (Cortex-M4F, 168MHz, 512KB Flash, 192KB SRAM) + **FreeRTOS** 实时操作系统开发。

> **硬件平台**: 自制 A 板（与 R2 主控相同），非 Robomaster 开发板 A 型。R1 下层底盘使用 Robomaster 开发板 A 型（STM32F427IIHx）。

R1 当前采用**四轮麦克纳姆轮底盘 + 机械臂 + 夹取/升降机构**，上层主控负责机械臂 IK、夹取状态机、电磁阀、吸盘等上层机构的控制，同时兼任底盘运动学计算。与下层底盘共享同一个 SBUS 接收机（Y 线并联），上/下层各自独立解析。

> **2026-05-18**: CH6 三段模式切换（底盘/机械臂/夹取）+ 夹取工序状态机完成。

---

## 硬件架构

### 主控芯片
- **MCU**: STM32F407VET6 (Cortex-M4F, 168MHz, 512KB Flash, 192KB SRAM)
- **硬件板**: 自制 A 板（与 R2 主控相同）
- **时钟**: HSE 12MHz → PLL 168MHz (HCLK=168, APB1=42, APB2=84)

### 外设资源与引脚映射

| 外设 | 引脚 | 用途 |
|------|------|------|
| CAN1 | PD0/PD1 | 机械臂 DM 电机 (`ARM_CAN=hcan1`) + YV 电磁阀刷新 |
| CAN2 | PB5/PB6 | 底盘 4×DM4310 电机 (`chassis_hcan=hcan2`, MIT 模式, 1Mbps) |
| SPI1 + PA4(CS) | PA5/PA6/PA7 | MCP2515 → hcan5 (`grabbing_can`)，夹取/升降机构 |
| SPI2 + PB12(CS) | PB13/PB14/PB15 | MCP2515 → hcan4 (预留电磁阀) |
| SPI3 + PA15(CS) | PC10/PC11/PC12 | MCP2515 预留，当前夹取机构改用 SPI1+PA4 初始化 hcan5 |
| UART4 | PA0(TX)/PA1(RX) | SBUS 遥控器接收 (100kbps, 9E2, DMA) |
| USART1 | PA9(TX)/PA10(RX) | 调试串口 (115200-8N1) |
| USART2 | PD5(TX)/PD6(RX) | 通信串口 (115200-8N1) |
| TIM1_CH1 | PE9 | PWM 输出 (预留) |
| GPIOE[0:7] | PE0~PE7 | LED 流水灯 (开漏输出) |
| KEY1 | PB2 | 按键输入 (下拉) |

### 执行器

| 执行器 | 数量 | CAN ID | 控制总线 | 协议 |
|--------|------|--------|----------|------|
| DM4310 (FR) | 1 | 0x02 | CAN2 | MIT 速度模式 |
| DM4310 (FL) | 1 | 0x03 | CAN2 | MIT 速度模式 |
| DM4310 (BR) | 1 | 0x01 | CAN2 | MIT 速度模式 |
| DM4310 (BL) | 1 | 0x04 | CAN2 | MIT 速度模式 |
| 机械臂 DM | 4 | 0x01~0x04 | CAN1 | 位置模式 |
| 夹取/升降 DM | 5 | 0x01~0x05 | hcan5(SPI1+PA4) | 位置/MIT 控制 |

---

## 软件架构

### 目录结构

```
R1/upperpart/
├── .gitignore                  # Git 忽略规则
├── A_board.ioc                 # CubeMX 引脚配置
├── CLAUDE.md                   # AI 操作备忘
├── README.md                   # 中文项目文档
├── memory_box_short.md         # 短记忆匣（快速参考）
├── memory_box_long.md          # 长记忆匣（完整上下文）
├── Core/Src/                   # CubeMX 生成 + 主逻辑
│   ├── main.c                  # 主程序：任务创建 + 控制逻辑
│   ├── freertos.c              # FreeRTOS 初始化
│   ├── can.c / dma.c / gpio.c / spi.c / tim.c / usart.c
│   ├── stm32f4xx_it.c          # 中断服务
│   └── stm32f4xx_hal_msp.c     # HAL MSP 初始化
├── Lib/                        # 应用层库
│   ├── sbus_set.c/h            # SBUS 遥控器解码 (UART4 DMA)
│   ├── bsp_can.c/h             # CAN 滤波器初始化
│   ├── mcp2515.c/h + consts.h  # MCP2515 SPI-CAN 驱动
│   ├── can_receive.c/h         # CAN 数据接收
│   ├── motor_control.c/h       # DM/YUN 电机 MIT/位置模式控制
│   ├── chassis.c/h             # 麦轮底盘逆运动学
│   ├── pid.c/h                 # PID 控制器
│   ├── solenoid_valves.c/h     # 电磁阀控制宏 YV1~YV10
│   ├── arm.c/h                 # 机械臂关节控制 + 3D/2D IK
│   └── grabbing.c/h            # 夹取/升降机构控制
├── Doc/                        # 文档
├── Drivers/                    # STM32 HAL + CMSIS
├── Middlewares/                # FreeRTOS 源码
└── MDK-ARM/                    # Keil 工程文件 (A_board.uvprojx)
```

### FreeRTOS 任务

| 任务 | 周期 | 栈(words) | 优先级 | 功能 |
|------|------|-----------|--------|------|
| `start_task` | 一次性 | 256 | 0 | 初始化 SBUS + CAN 滤波器 + grabbing MCP2515，创建 3 个子任务后自销毁 |
| `led_task` | 100/200ms | 128 | 0 | GPIOE[0:7] 流水灯 (使能时 200ms，失能时 100ms) |
| `sbus_task` | 10ms | 512 | 0 | SBUS 解码 + CH4 使能 + CH6 模式切换 + 底盘/机械臂/夹取控制 |
| `chassis_task` | 5ms | 256 | 0 | 麦轮逆运动学计算 + CAN MIT 控制帧发送 |

所有任务优先级相同(0)，FreeRTOS 时间片轮询。

### 启动流程

```
main()
 ├─ HAL_Init()
 ├─ SystemClock_Config()         # HSE → PLL (168MHz)
 ├─ MX_GPIO / DMA / CAN1/2 / SPI1/2/3 / UART4 / USART1/2 / TIM1 _Init()
 ├─ xTaskCreate(start_task)      # 创建初始化任务
 ├─ MX_FREERTOS_Init()           # FreeRTOS 初始化
 └─ osKernelStart()             # 启动调度器

start_task():
 ├─ sbus_rx_init()               # 启动 UART4 DMA 接收 SBUS
 ├─ can_filter_init()            # 配置 CAN1 滤波器
 ├─ grabbing_init()              # SPI1+PA4 初始化 hcan5 夹取机构 MCP2515
 ├─ xTaskCreate(×3)              # 创建全部子任务
 └─ vTaskDelete(NULL)            # 自销毁
```

---

## 控制模式

### 遥控器规格

R1 上下层各有独立 SBUS 接收机，绑定同一遥控器。两块板之间无线路连接，各自独立解析 SBUS 信号。

| 通道 | 控件类型 | 输出范围 | 中位 | 说明 |
|------|---------|---------|------|------|
| CH0~CH3 | 摇杆 | 326 ~ 1659 | 992 | 左右/上下 |
| CH4~CH6 | 3 段拨杆 | 326 / 992 / 1659 | 992 | 上/中/下三段 |
| CH7 | 2 段拨杆 | 329 / 1659 | — | 上/下两段 |
| CH8~CH9 | 旋钮 (面板) | 326 ~ 1659 | — | 旋转 |
| CH10~CH11 | 旋钮 (背面) | 326 ~ 1659 | — | 旋转 |

> SBUS 帧: UART4, 100kbps 9E2 DMA, 25 字节/帧, 15 通道, 每通道 11-bit (0~2047)。

### 当前通道分配 (2026-06-15 底盘解耦后)

| 通道 | 用途 | 阈值 | 说明 |
|------|------|------|------|
| CH4 | 主使能 / 锁车 | `>1300` 解锁, `<650` 锁车 | 3 段拨杆上拨解锁，下拨锁车 |
| CH0~CH3, CH5~CH11 | 预留 | — | 待后续功能分配 |

### 锁车逻辑

```
CH4 上拨 (>1300) 且上升沿  →  sys_enabled = true   (解锁)
CH4 下拨 (<650) 且下降沿   →  sys_enabled = false  (锁车)
SBUS 断联 >50ms            →  sys_enabled = false  (自动锁车)
```

> 上下层各自独立锁车/解锁，互不影响。

---

## 库模块说明

### [sbus_set](Lib/sbus_set.h)
SBUS 协议解码。`rx_set()` 从 UART4 DMA 缓冲区解析 25 字节为 16 通道值，提供 `Map()` (float) 和 `map()` (int16) 映射函数。

### [motor_control](Lib/motor_control.h)
DM 系列电机和云台电机的 CAN 控制协议（当前 main.c 未引用，保留供后续使用）：
- **MIT 模式**: `dm_mit_ctrl()` 位置/速度/扭矩联合控制
- **位置模式**: `pos_ctrl()` 位置伺服控制
- **使能/失能**: `dm_enable()` / `dm_disable()`

### [chassis](Lib/chassis.h)
4 轮麦克纳姆轮底盘运动学（⚠️ 2026-06-15 上层已解耦底盘，当前 main.c 未引用）。

### [mcp2515](Lib/mcp2515.h) + [mcp2515_consts](Lib/mcp2515_consts.h)
MCP2515 SPI-CAN 桥接芯片驱动。支持多种速率、标准/扩展帧过滤掩码、3TX+2RX 缓冲。

### [solenoid_valves](Lib/solenoid_valves.h)
10 路电磁阀 YV1~YV10 的位操作宏（当前 main.c 未引用，保留供后续使用）。
> **注意**: YV9/YV10 的 flip 操作有 bug：异或时错误地操作了 `[0]` 而非 `[1]`

### [pid](Lib/pid.h)
位置式/增量式 PID 控制器（预留，当前未使用）。
> **注意**: pid.h 重新定义了 `int8_t`/`uint8_t` 等标准类型，与 `<stdint.h>` 可能冲突

### 其他 Lib 模块
`arm.c/h`、`grabbing.c/h`、`can_receive.c/h`、`bsp_can.c/h` 保留在 Lib 目录中，当前 main.c 未引用，待后续功能开发时重新接入。

---

## 安全机制

1. **SBUS 帧校验**: `buf[0]==0x0F && buf[23]==0x00 && buf[24]==0x00`
2. **SBUS 超时断联** (50ms): 自动失能 + 速度归零
3. **CH4 边沿检测**: 上升沿使能，下降沿失能 + 安全停车
4. **遥控信号中位死区**: CH0/CH2/CH3 在 1022~1026 范围内输出零

---

## 调试接口

| 接口 | 引脚 | 参数 | 用途 |
|------|------|------|------|
| USART1 | PA9/PA10 | 115200-8N1 | 调试日志 |
| USART2 | PD5/PD6 | 115200-8N1 | 上位机通信 |
| SWD | PA13/PA14 | — | J-Link / ST-Link |

---

## 已知问题 / TODO

1. **pid.h 类型重定义** — 重新声明 `int8_t`/`uint8_t` 等，与标准库可能冲突
2. **YV9/YV10 flip bug** — `solenoid_valves.h:68,74` 异或操作写错数组下标
3. **机械臂/夹取功能待恢复** — `arm.c/h`、`grabbing.c/h` 保留在 Lib 目录，待硬件方案确定后重新接入
4. **旧文件残留** — `chassis_ik.o`, `three_steering_wheel_ik.o`, `four_steering_wheel_ik.o`, `upstairs.o` 为旧版本编译残留

---

## 2026-06-15 更新：底盘解耦 + 骨架清理 + 遥控器参数修正

上下层电源分离，底盘控制完全移交给下层。上层 main.c 清理为最小骨架：

- **移除**: `chassis_task`、底盘运动学、机械臂 IK、夹取状态机、CH6 三段模式切换、吸盘 YV1 控制
- **保留**: LED 流水灯、SBUS 解码 + 50ms 断联保护、CH4 锁车/解锁
- **阈值修正**: `ch_high()` 从 `>1700` 改为 `>1300`，匹配实际遥控器 3 段拨杆量程 (326/992/1659)
- **新增**: `ch_low()` `<650`、`ch_mid()` `650~1300`

### 修正后遥控器参数

| 通道 | 控件 | 量程 | 阈值 (上层) |
|------|------|------|------------|
| CH0~CH3 | 摇杆 | 326~1659 (中位 992) | — |
| CH4~CH6 | 3 段拨杆 | 326 / 992 / 1659 | high>1300, low<650, mid 650~1300 |
| CH7 | 2 段拨杆 | 329 / 1659 | high>1300, low<650 |
| CH8~CH11 | 旋钮 | 326~1659 | — |

> 上下层各有独立接收机，无线路连接。CH4 锁车各自独立。

## 编译信息

- 2026-05-18 build: 0 errors, 0 warnings, Flash 21192 bytes, RAM 23520 bytes (清理前)
- 编译器: ARMCLANG V6.23 (AC6), C99 + GNU extensions
- 产物: `MDK-ARM/A_board/A_board.hex` / `MDK-ARM/A_board/A_board.axf`

---

## 相关链接

| 资源 | 链接 |
|------|------|
| R1 下层底盘 | [../lowerpart/](../lowerpart/) — Robomaster A板, STM32F427IIHx |
| R2 主控 | [../../R2/](../../R2/) — 自制A板, 同款MCU |
| 短记忆匣 | [memory_box_short.md](memory_box_short.md) |
| 长记忆匣 | [memory_box_long.md](memory_box_long.md) |
| Keil 工程 | [MDK-ARM/A_board.uvprojx](MDK-ARM/A_board.uvprojx) |

> ⚠️ 本 README 为手动维护，部分过时信息以代码实际状态为准。最新变更摘要见 `memory_box_short.md`，详细上下文见 `memory_box_long.md`。
