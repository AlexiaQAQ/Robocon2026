# Robocon 2026 — R1 底盘主控固件

[![MCU](https://img.shields.io/badge/MCU-STM32F407VET6-03234B?logo=stmicroelectronics)](https://www.st.com/en/microcontrollers-microprocessors/stm32f407ve.html)
[![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-6DB33F)](https://www.freertos.org/)
[![IDE](https://img.shields.io/badge/IDE-Keil%20MDK-009F4D)](https://www.keil.com/)

> 📖 [English version](README.en.md)

## 概述

本项目为 Robocon 2026 赛季 **R1 底盘车**的主控制板固件，基于 **STM32F407VET6** (Cortex-M4F, 168MHz) + **FreeRTOS** 实时操作系统开发。

R1 作为赛季首台验证车，承担了**电机驱动库搭建、SBUS 遥控链路验证、DM 系列电机 CAN 协议适配**的使命。其电机控制模块 (`motor_control.c/h`) 经过打磨后成为后续 R2 等车型的基础库。

> **2026-06-06** 电机控制重构: 引入 `dm_model_t` 多型号电机支持，替代全局硬编码参数。

---

## 硬件架构

### 主控芯片
- **MCU**: STM32F407VET6 (Cortex-M4F, 168MHz, 512KB Flash, 192KB SRAM)

### 外设资源与引脚映射

| 外设 | 引脚 | 用途 |
|------|------|------|
| CAN1 | PD0/PD1 | 底盘电机 CAN 总线 |
| CAN2 | PB5/PB6 | 第二路 CAN 总线 |
| USART1 | PB6(TX)/PB7(RX) | SBUS 遥控器接收 (100kbps, 9-bit, Even, 2-stop) |
| UART7 | PE8(TX)/PE7(RX) | 预留串口 (115200-8N1) |
| UART8 | PE1(TX)/PE0(RX) | 预留串口 (115200-8N1) |
| USART6 | PG14(TX)/PG9(RX) | 预留: 云台/上位机通信 (115200-8N1, DMA RX+TX) |
| SPI5 | — | SPI 扩展 |
| TIM4 | — | 通用定时器 |
| TIM5 | — | 通用定时器 |
| TIM14 | — | FreeRTOS 系统时基 |
| GPIOG[1:8] | PG1~PG8 | LED 流水灯 (推挽输出) |
| GPIOH[2:5] | PH2~PH5 | XT1~XT4 输出 (推挽输出, 下拉) |
| GPIOF[6] | PF6 | 输出 |
| GPIOD[12] | PD12 | 输入 (下拉) |

### 执行器

| 执行器 | 数量 | 控制总线 | 电机类型 | 状态 |
|--------|------|----------|----------|------|
| 底盘电机 | 4 | CAN1/CAN2 (ID 1~4) | DM 系列 MIT 模式 | ⚠️ 运动学待实现 |
| XT 输出 | 4 | PH2~PH5 | GPIO 直驱 | ✅ 已配置 |

---

## 软件架构

### 目录结构

```
R1/lowerpart/
├── Core/Src/               # CubeMX 生成 + 主逻辑
│   ├── main.c              # 主程序: 任务创建 + SBUS 遥控解析
│   ├── freertos.c          # FreeRTOS 配置
│   ├── can.c / dma.c / gpio.c / spi.c / tim.c / usart.c
│   ├── stm32f4xx_it.c      # 中断服务
│   └── stm32f4xx_hal_msp.c # HAL MSP 初始化
├── Core/Inc/               # 头文件
│   ├── main.h / can.h / gpio.h / dma.h
│   └── FreeRTOSConfig.h
├── Lib/                    # 应用层库 (✅ 代码质量高, 可直接复用)
│   ├── motor_control.c/h   # ★ DM/YUN 电机驱动 (支持 7 种型号)
│   ├── bsp_can.c/h         # CAN 滤波器初始化
│   ├── CAN_receive.c/h     # CAN 数据接收 (3508 电流模式)
│   ├── sbus_set.c/h        # SBUS 遥控器解码 (USART1 DMA)
│   └── pid.c/h             # PID 控制器 (位置式/增量式)
├── Drivers/                # STM32 HAL + CMSIS
├── Middlewares/            # FreeRTOS 源码
└── MDK-ARM/                # Keil 工程文件 (master_a.uvprojx)
```

### FreeRTOS 任务

| 任务 | 周期 | 栈 (words) | 优先级 | 功能 |
|------|------|-----------|--------|------|
| `start_task` | 一次性 | 256 | 0 | 初始化外设和子系统，创建子任务后自销毁 |
| `remote_task` | ~4ms | 128 | 0 | SBUS 解码 + CH4 使能边沿检测 + 速度映射 |
| `chassis_task` | ~2ms | 128 | 0 | ⚠️ 空骨架 — 底盘运动学待实现 |
| `led_task` | 200ms | 128 | 0 | GPIOG[1:8] 流水灯 |

### 启动流程

```
main()
 ├─ HAL_Init()
 ├─ SystemClock_Config()         # HSE → PLL (168MHz)
 ├─ MX_GPIO / DMA / CAN1/2 / USART1 / UART7/8 / USART6 / TIM4/5 / SPI5 _Init()
 ├─ xTaskCreate(start_task)      # 创建初始化任务
 ├─ MX_FREERTOS_Init()           # 创建 defaultTask (空闲)
 └─ osKernelStart()             # 启动调度器

start_task():
 ├─ can_filter_init()            # 配置 CAN1/2 滤波器 (全通)
 ├─ sbus_rx_init()               # 启动 USART1 DMA 接收 SBUS
 ├─ xTaskCreate(led_task)        # 流水灯
 ├─ xTaskCreate(remote_task)     # 遥控解析
 └─ vTaskDelete(NULL)            # 自销毁
```

---

## 控制模式

系统通过遥控器 CH4 通道控制使能状态：

```
CH4 (主使能, 阈值 1600)
├── OFF (<1600 或 SBUS断联)
│   └── sys_enabled = false, 速度归零
│
└── ON  (>1600)
    ├── CH0 (右摇杆水平) → vw (角速度)
    ├── CH2 (左摇杆垂直) → vx (前进速度)
    ├── CH3 (左摇杆水平) → vy (横向速度)
    └── 摇杆中位死区: 991~993 区间映射为 0
```

> **当前版本仅 SBUS 遥控一种模式**，无上位机串口自动模式。

---

## SBUS 遥控器协议

| 参数 | 值 |
|------|-----|
| 接口 | USART1 (PB6/PB7) |
| 波特率 | 100000 |
| 数据位 | 9 |
| 停止位 | 2 |
| 校验位 | Even |
| 帧长度 | 25 字节 |
| DMA | DMA2_Stream2, 循环模式 |

- 15 通道 (CH0~CH14)，每通道 11-bit (0~2047)
- 帧校验: `buf[0]==0x0F && buf[23]==0x00 && buf[24]==0x00`
- 校验失败自动复位 DMA 重新开始接收
- 50ms 超时断联 → `sys_enabled=false` → 速度归零

---

## 电机控制 (motor_control)

### 支持的电机型号

R1 的 `motor_control` 模块是赛季基础库，实现了**按型号管理 MIT 参数范围**，是后续车型的参考实现：

| 型号 | MIT 参数 (P±/V±/T±) | 说明 |
|------|---------------------|------|
| DM_4310 | 12.5 / 30 / 10 | 标准 DM 电机 |
| DM_4310_48V | 12.5 / 45 / 18 | 48V 高压版 |
| DM_4340 | 12.5 / 30 / 27 | 大扭矩 |
| DM_3519 | 12.5 / 20 / 8 | 减速电机 (19:1) |
| DM_8006 | 12.5 / 21 / 20 | 减速电机 (6:1) |
| DM_8009 | 12.5 / 18 / 40 | 大扭矩减速 (9:1) |
| DM_CUSTOM | 12.5 / 30 / 10 | 用户自定义 |

### 控制模式

```c
DM_MODE_MIT  // MIT 模式: pos + vel + kp + kd + torque
DM_MODE_POS  // 位置-速度模式: float pos + float vel
DM_MODE_SPD  // 速度模式
DM_MODE_PSI  // 位速流模式
```

### 设计亮点 (可供 R2 参考)

- **per-motor 范围**: 每个电机 `motor_t` 结构体自带 `p_max/v_max/t_max`，不同型号使用不同范围
- **mode 自动偏移**: `motor->mode` 字段自动计算 CAN ID 偏移 (`+MIT_MODE` / `+POS_MODE` 等)
- **统一反馈回调**: `dm_rx_cbk(motor_set, rx_data)` 自动从反馈首字节提取 ID 分发给对应 `motor_t`
- **命令统一格式**: 所有使能/失能/清零/清除错误使用同一个 `dm_send_cmd()` 内部函数

---

## 安全机制

1. **SBUS 帧校验**: 校验字节 0 (0x0F) + 字节 23 (0x00) + 字节 24 (0x00)，校验失败自动复位 DMA
2. **SBUS 超时断联**: 50ms 无有效帧 → `sys_enabled=false` → 速度归零
3. **CH4 边沿检测**: 上升沿使能，下降沿失能 + 安全停车
4. **摇杆中位死区**: 991~993 区间强制映射为 0，防止摇杆漂移

---

## 当前状态与待办

### ✅ 已完成
- 电机驱动库 (7 种 DM 型号 + YUN 系列, MIT/POS/SPD/PSI 四模式)
- SBUS 遥控器收发 (DMA 循环, 帧校验, 超时断联)
- FreeRTOS 多任务框架
- CAN 滤波器配置
- LED 流水灯

### ⚠️ 待实现
- **底盘运动学**: `chassis_task` 为空骨架，需实现全向轮/麦克纳姆轮逆运动学
- **电机使能流程**: 当前无 `dm_enable` 调用，需在 `start_task` 中添加使能逻辑
- **CAN 反馈处理**: CAN 中断回调已实现 (`stm32f4xx_it.c`)，但需验证 `dm_rx_cbk` 分发逻辑
- **云台/武器头控制**: `main.c` 中有注释掉的 USART6 云台控制代码，待重新启用

---

## 相关链接

| 资源 | 链接 |
|------|------|
| 仓库 | [Gitee — alexiaqaq/robocon2026](https://gitee.com/alexiaqaq/robocon2026) |
| R2 主控 | [../R2/](../R2/) |
| 中英文档 | [README.en.md](README.en.md) |
| 短记忆匣 | [memory_box_short.md](memory_box_short.md) |
| 长记忆匣 | [memory_box_long.md](memory_box_long.md) |

> ⚠️ 本 README 为手动维护。最新变更摘要见 `memory_box_short.md`，详细上下文见 `memory_box_long.md`。
