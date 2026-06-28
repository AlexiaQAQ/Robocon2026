# Robocon 2026 — R1 上层主控固件 (自制 A 板)

[![MCU](https://img.shields.io/badge/MCU-STM32F407VET6-03234B?logo=stmicroelectronics)](https://www.st.com/en/microcontrollers-microprocessors/stm32f407ve.html)
[![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-6DB33F)](https://www.freertos.org/)
[![IDE](https://img.shields.io/badge/IDE-VSCode%20%2B%20EIDE%20%2F%20Keil-007ACC)](https://code.visualstudio.com/)
[![Board](https://img.shields.io/badge/Board-自制A板-555555)]()

## 概述

本项目为 Robocon 2026 赛季 R1 机器人**上层**主控制板固件，基于 **STM32F407VET6** (Cortex-M4F, 168MHz) + **FreeRTOS** 实时操作系统开发。

> **硬件平台**: 自制 A 板（与 R2 主控相同），非 Robomaster 开发板 A 型。R1 下层底盘使用 Robomaster 开发板 A 型（STM32F427IIHx）。上下层各有独立 SBUS 接收机，无线路连接。

R1 上层负责：4×DM4340 抬升平台、左右双机械臂（2×2 关节）、抓取机构（齿条+翻转+夹爪）、吸盘电磁阀。与下层底盘共享遥控器，各自独立解析 SBUS。

> **2026-06-19**: 模块化重构 — 抬升/机械臂/抓取拆分独立状态机库，main.c 只做调度

---

## 硬件架构

### 主控芯片
- **MCU**: STM32F407VET6 (Cortex-M4F, 168MHz, 512KB Flash, 192KB SRAM)
- **硬件板**: 自制 A 板（与 R2 主控相同）
- **时钟**: HSE 12MHz → PLL 168MHz

### 外设引脚映射

| 外设 | 引脚 | 用途 |
|------|------|------|
| CAN1 | PD0/PD1 | 抬升×4 + 抓取×2 + YV 电磁阀刷新 (CAN ID 0x300) |
| CAN2 | PB5/PB6 | 双机械臂 DM 电机 ×4 |
| UART4 | PA0(TX)/PA1(RX) | SBUS 遥控器 (100kbps, 9E2, DMA 循环) |
| GPIOE[0:7] | PE0~PE7 | LED 流水灯 |
| SPI1/2/3 | — | MCP2515 预留 |
| USART1 | PA9/PA10 | 调试串口 (115200-8N1) |
| KEY1 | PB2 | 按键输入 |

### 执行器

| 执行器 | 数量 | CAN ID | 总线 | 型号 | 模式 |
|--------|------|--------|------|------|------|
| 抬升电机 | 4 | 1~4 | CAN1 | DM4340 | 位置-速度 |
| 左臂根部 | 1 | 1 | CAN2 | DM4340 | 位置 |
| 左臂末端 | 1 | 2 | CAN2 | DM4310 | 位置 |
| 右臂根部 | 1 | 3 | CAN2 | DM4340 | 位置 |
| 右臂末端 | 1 | 4 | CAN2 | DM4310 | 位置 |
| 翻转电机 | 1 | 5 | CAN1 | DM4310 | 位置-速度 |
| 齿条电机 | 1 | 6 | CAN1 | DM2325 | 位置-速度 |
| 电磁阀 | 3 | — | CAN1 0x300 | YV1(左吸盘) YV2(右吸盘) YV3(夹爪) | — |

---

## 软件架构

### 目录结构

```
R1/upperpart/
├── Core/Src/main.c              # 主程序: 锁车 + SBUS调度 + 任务创建
├── Lib/
│   ├── sbus_set.c/h             # SBUS DMA循环接收 + 帧解析
│   ├── dm_motor.c/h             # 达妙电机 CAN 驱动库
│   ├── lift.c/h                 # 抬升模块 (状态机 + 50Hz控制)
│   ├── arm.c/h                  # 双机械臂模块 (UP/DOWN状态机 + 限幅)
│   ├── grab.c/h                 # 抓取模块 (6步工序状态机)
│   ├── solenoid_valves.c/h      # 电磁阀 YV1~YV10
│   ├── bsp_can.c/h              # CAN 滤波器
│   ├── can_receive.c/h          # CAN 接收回调
│   ├── mcp2515.c/h              # MCP2515 SPI-CAN
│   └── pid.c/h                  # PID (预留)
├── Doc/                         # 文档
├── Drivers/ / Middlewares/      # HAL + FreeRTOS
└── MDK-ARM/                     # Keil 工程
```

### FreeRTOS 任务

| 任务 | 周期 | 栈 | 优先级 | 功能 |
|------|------|-----|--------|------|
| `start_task` | 一次性 | 256 | 0 | 初始化 → 建子任务 → 自销毁 |
| `sbus_task` | ~4ms | 512 | 0 | sbus_poll + 锁车 + 模式调度 |
| `led_task` | 100/200ms | 56 | 0 | LED 流水灯 |
| `lift_task` | 50Hz | 512 | 0 | 抬升 4×pos_ctrl + YV_flash |
| `arm_task` | 50Hz | 256 | 0 | 双机械臂 4×pos_ctrl + 限幅 |
| `grab_task` | 50Hz | 256 | 0 | 抓取 2×pos_ctrl |

---

## 遥控通道分配

> 物理拨杆与 SBUS 反相: 物理上拨 → ch_low(<650), 物理下拨 → ch_high(>1300)

| 通道 | 控件 | 模式 | 功能 |
|------|------|------|------|
| CH1 | 摇杆 | CH5中/下 | 连续微调抬升 (松手归零) |
| CH1 | 摇杆 | CH5上 | 增量微调对接高度 (±0.05/次, 累积±3.0) |
| CH4 | 3段拨杆 | 全局 | 中位=串口IR模式, 下拨=解锁, 上拨=锁车+清抓取 |
| CH5 | 3段拨杆 | 全局 | 上拨→抓取, 中位→吸方块, 下拨→放方块 (50ms消抖) |
| CH6 | 3段拨杆 | CH5中 | 抬升基准: 上拨24.0 / 中位14.0 / 下拨4.0 |
| CH6 | 3段拨杆 | CH5上 | 全局抬升: 上拨25.0 / 中位14.15(对接) / 下拨0.2(回零) |
| CH6 | 3段拨杆 | CH5下 | 放方块双档: 上拨16.5 / 中/下0.0 |
| CH7 | 2段拨杆 | CH5中/下 | 切换选中臂抬起/放下 |
| CH7 | 2段拨杆 | CH5上 | 步骤0→5: 推进工序; 步骤6: 切换翻转位置(0↔-0.4) |
| CH8 | 旋钮 | 全局 | >1300 左吸盘 (YV1) |
| CH9 | 旋钮 | 全局 | >1300 右吸盘 (YV2) |
| CH10 | 旋钮 | 全局 | 齿条 0~12 |
| CH11 | 旋钮 | CH5中/下 | >1000左臂, <1000右臂 |

### CH4 锁车

- 中位 → 串口IR指令 (CH7边沿→USART3+USART6发送 A1 F1 CC 01 EE)
- 下拨/中位 + 上升沿 → 解锁, 所有电机使能
- 上拨 + 下降沿 → 锁车 + 清零抓取工序
- 100ms 断联 → 自动锁车
- 失控保护 (failsafe) → 立刻锁车

---

## 机械臂 (短臂)

| 关节 | ID | 范围 | 说明 |
|------|-----|------|------|
| 左根 | 1 | 0~-1.57 | 0=水平, -1.57=竖起 |
| 左末 | 2 | 0~-1.57 | 0=顺臂朝前, -0.785=45°朝天 |
| 右根 | 3 | 0~1.57 | 0=水平, 1.57=竖起 |
| 右末 | 4 | 0~1.57 | 0=顺臂朝前, 0.785=45°朝天 |

- 默认: 竖起 (root=±1.57, tip=±0.785/45°)
- CH7 切换选中臂抬起/放下
- 放下: root=0, tip 朝前 (45°)
- 放方块模式: 臂抬起末端45°朝前即可, 保留CH7切换用于吸被戳落的方块
- arm_task 每帧 clampf 限幅

## 抓取工序

CH5 上拨进入, CH7 推进 0→5, 到步骤6停止:

| 步 | 状态 | 动作 |
|----|------|------|
| 0 | 空闲 | 翻转0 夹爪关 |
| 1 | 齿条回零 | 翻转0 夹爪关 |
| 2 | 开夹爪 | YV3(1) (齿条归零才开) |
| 3 | 翻转取杆 | 翻转→-1.8 |
| 4 | 关夹爪 | YV3(0) |
| 5 | 翻回 | 翻转→0 |
| 6 | 完成 | CH7切换翻转(0↔-0.4), CH6全局控抬升 |

- CH6 全局三档: 上拨25.0 / 中位14.15(对接) / 下拨0.2(回零)
- CH1 增量微调中档和高档 (±3.0)
- CH10 始终控制齿条
- 4310 翻下/翻上速度分离 (6.0/2.0), 硬限幅 0~-2.0

---

## 安全机制

1. **SBUS 帧校验**: `buf[0]==0x0F && buf[24]==0x00`
2. **SBUS DMA NDTR 陈帧检测**: 连续5次NDTR不变 → 确认DMA冻结, 丢弃陈帧
3. **SBUS 超时断联** (100ms): 自动锁车
4. **失控保护**: 接收机 failsafe 标志 → 立刻锁车
5. **CH4 边沿检测**: 上升沿解锁, 下降沿锁车
6. **CH5 模式消抖** (50ms): 路过中间档不触发
7. **抬升斜率限制**: 大跳2.5/tick快响应, 微调0.5/tick平滑
8. **抬升硬限幅**: LIFT_MIN(0.2) ~ LIFT_MAX(29.0)
9. **机械臂限幅**: arm_task 每帧 clampf
10. **翻转限幅**: FLIP_MAX(0) ~ FLIP_MIN(-2.0)
11. **跨模式CH7防抖**: tick间隔>50ms自动同步, 不误触

## 已知问题

- pid.h 类型重定义 — 与 `<stdint.h>` 可能冲突
- YV9/YV10 flip bug — `solenoid_valves.h` 异或写错数组下标

## 编译信息

- 编译器: ARMCLANG V6.23 (AC6), C99 + GNU extensions
- 产物: `MDK-ARM/A_board/A_board.hex` / `.axf`

---

## 相关链接

| 资源 | 链接 |
|------|------|
| R1 下层底盘 | [../lowerpart/](../lowerpart/) — Robomaster A板, STM32F427IIHx |
| R2 主控 | [../../R2/](../../R2/) — 自制A板, 同款MCU |
| 短记忆匣 | [memory_box_short.md](memory_box_short.md) |
| 长记忆匣 | [memory_box_long.md](memory_box_long.md) |
