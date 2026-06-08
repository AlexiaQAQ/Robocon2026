---
name: Robocon R2 项目上下文记忆匣
description: Robocon2026 R2 主控固件开发全上下文：项目架构、代码变更历史、关键修改细节、当前任务状态
type: project
---

# Robocon 2026 — R2 主控固件 记忆匣

## 1. 项目概述

- **项目**: Robocon 2026 赛季 R2 机器人主控板固件 (A板)
- **MCU**: STM32F407VET6 (Cortex-M4F, 168MHz, 512KB Flash, 192KB SRAM)
- **RTOS**: FreeRTOS (CMSIS-OS v1 封装, CubeMX 配置为 CMSIS_V1)
- **结构**: **四全向轮底盘 + 升降机构(前后模块) + 左右双机械臂** (2026-06-06 重构)
- **控制方式**: SBUS 遥控器直连 (手动模式) / 上位机串口指令 (自动模式)
- **IDE**: Keil MDK-ARM
- **工程文件**: `MDK-ARM/A_board.uvprojx`
- **工作目录**: `d:\Code\RC2026\R2\`

## 2. 硬件架构关键信息 (当前，2026-06-06)

| CAN总线 | 通路 | 用途 |
|---------|------|------|
| CAN1 (PD0/PD1) | 原生 | 底盘: 4×DM MIT (全向轮 ID 0x01-0x04) + 4×DM4310 抬升 (ID 0x05-0x08) |
| CAN2 (PB5/PB6) | 原生 | 左机械臂: DM电机 位置模式 (yaw/pitch/elbow/terminal, ID 0x01-0x04) |
| hcan3 (MCP2515, SPI1+PA4) | 扩展 | 夹爪翻转 DM 位置 (ID 0x01) + 电磁阀 YV1 (CAN ID 0x300) |
| hcan4 (MCP2515, SPI2+PB12) | 扩展 | ⚠️ 预留/待分配 |
| hcan5 (MCP2515, SPI3+PA15) | 扩展 | 升降机构: 2×DM电机 位置-速度模式 (伸展/回收) |
| ⚠️ 待定 | 待定 | 右机械臂: DM电机 位置模式 (yaw/pitch/elbow/terminal, 待定) |

| UART | 引脚 | 参数 | 用途 |
|------|------|------|------|
| UART4 | PA0(TX)/PA1(RX) | 100kbps, 9-bit, Even, 2-stop | SBUS 遥控器 (25字节帧, DMA1_Stream2) |
| USART1 | PA9(TX)/PA10(RX) | 115200, 8N1 | 上位机通信: 底盘速度+电磁阀+升降距离 (15字节帧, DMA) |
| USART2 | PD5(TX)/PD6(RX) | 115200, 8N1 | 上位机通信: 机械臂位置指令 (10字节帧, DMA) |
| USART3 | PD8(TX)/PD9(RX) | 115200, 8N1 | 传感器帧发送(阻塞5Hz)；IMU欧拉角接收(DMA)模块已实现但当前未创建任务，启用后两者共享总线 |

| 传感器 | GPIO | 说明 |
|--------|------|------|
| 前传感器 | PE9 | 输入, 无上下拉, PIN_SET=无障碍(0x00), PIN_RESET=有障碍(0x01) |
| 后传感器 | PE13 | 同上 |

| IMU | 型号 | 总线 | 说明 |
|-----|------|------|------|
| HLK-AS201 | USART3 DMA | 只订阅欧拉角, Yaw低通滤波 α=0.9, 14字节帧 |

## 3. FreeRTOS 任务 (全部优先级0, 时间片轮询)

| 任务 | 周期 | 栈(words) | 位置 | 功能 |
|------|------|-----------|------|------|
| `start_task` | 一次性 | 256 | main.c | 初始化外设→创建子任务→自销毁 (**无上电延时**) |
| `sbus_task` | 10ms | 256 | main.c | SBUS解码+CH4使能边沿+手动模式遥控映射+CH5下降沿通知 |
| `uart_task` | 100ms | 128 | uart_task.c | 自动模式串口解析(UART1底盘/阀+UART2机械臂)+传感器帧5Hz |
| `chassis_task` | ~2ms | 1280 | main.c | ⚠️ 全向轮运动学更新+CAN指令发送 (待重构) |
| `up_cs_task` | ~55ms | 256 | main.c | ⚠️ 升降底盘速度控制+电磁阀刷新 (待重构) |
| `arm_task` | 10ms | 512 | main.c | ⚠️ 左右双机械臂控制 (待重构) |
| `led_task` | 200ms | 56 | main.c | GPIOE[0:7]流水灯 |
| `imu_task` | 50ms | — | imu.c | IMU欧拉角解析+滤波模块已实现，但当前 `main.c` 未创建该任务，系统启动时不运行 |

> ⚠️ `chassis_task` 和 `up_cs_task` 为 2026-06-06 机械重构后的残留骨架，删除了旧舵轮/旧升降调用，待按全向轮+新升降重写。

## 4. 控制模式 (SBUS遥控器，未变)

- **CH4**: 主使能 (>1700=ON, 边沿检测使能/失能各子系统)
- **手动模式** (CH4=ON, CH5=HIGH, CH6=LOW): 遥控直控底盘+电磁阀, 机械臂自动归位安全位置
- **自动模式** (CH4=ON, CH5=LOW): 上位机串口全权控制
- **CH5下降沿**: 向上位机发送模式切换通知帧 (UART1+UART2+USART3传感器)

## 5. 关键代码变更历史

### 变更0: 机械结构重构清理 (2026-06-06) ⭐ 当前
**原因**: 机械结构大改：四舵轮 → 四全向轮，升降机构更换电机

**删除的文件**:
- `Lib/four_steering_wheel_ik.c` — 四舵轮串级PID运动学 (3508转向+DM驱动)
- `Lib/four_steering_wheel_ik.h` — 对应头文件
- `Lib/upstairs.c` — 旧升降 (云台电机MIT模式)
- `Lib/upstairs.h` — 对应头文件
- `Lib/arm.c` — 旧单机械臂逆运动学 (5-DOF, IK 3D/2D)
- `Lib/arm.h` — 对应头文件
- `Lib/solenoid_valves.c` — 旧10路电磁阀控制 (YV1~YV10 + hcan4 MCP2515)
- `Lib/solenoid_valves.h` — 对应头文件

**修改的文件**:
- `Core/Src/main.c`: 移除旧 include (four_steering_wheel_ik/upstairs/arm)、旧 Init 调用、chassis_task/up_cs_task/arm_task 改为空骨架、system_enable_handler 移除旧函数
- `Lib/uart_task.h`: 移除 `#include "four_steering_wheel_ik.h"` 和 `#include "arm.h"`，添加 arm 变量 extern
- `Lib/uart_task.c`: 新增 `fb_des/lr_des/ud_des/end_des` 定义 (从 arm.c 迁移)
- `README.md`: 重写机械结构章节
- `memory_box_short.md` / `memory_box_long.md`: 更新

**新机械结构**:
- 全向轮 ×4: DM电机 MIT模式, CAN1
- 升降底盘 ×4: DM电机 MIT模式, hcan3
- 升降机构 ×2: DM电机 位置-速度模式, hcan5
- 左机械臂 ×4: DM电机 位置模式, CAN2
- 右机械臂 ×4: DM电机 位置模式, ⚠️ CAN总线待定
- 夹爪翻转 ×1: DM电机 位置模式, hcan4
- 夹爪气缸 ×1: ⚠️ 待定 (旧 YV1~YV10 已全部删除)

**待办**: 
- 编写全向轮运动学 (替代 `four_steering_wheel_ik`)
- 编写新升降控制 (替代 `upstairs`)
- 编写双机械臂控制 (替代 `arm`) — 需 2套独立 IK
- 确定右机械臂 CAN 总线 (新增 CAN 或 MCP2515 扩展)
- 重写 `chassis_task`、`up_cs_task`、`arm_task`
- 更新 UART1/UART2 串口协议定义
- 更新 Keil 工程移除旧文件

---

### 变更1: 夹爪翻转+气缸串口接入 (2026-06-08) ⭐ 当前

**原因**: 夹爪翻转电机+气缸硬件验证完成, 接入遥控器和串口协议

**硬件迁移**:
- 夹爪翻转电机: hcan4 → **hcan3** (ID 0x01)
- 夹爪气缸 YV1: 接入 **hcan3** (CAN ID 0x300)
- hcan4 改为预留, solenoid_valves.c/h 保留用于 YV1 位操作

**抬升快慢速**:
- chassis.h/c:   rad/s
-  加 vel 参数
- 手动模式固定 slow, 自动模式按协议  选择 (0=slow, 1=fast)

**夹爪翻转控制** (: 0=竖起, 1.57=翻下):
- 速度宏:  / 
- 安全机制: , 上电使能不发送位置指令, 首次 CH7 操作或串口非零指令解锁
- 遥控: CH6低档(~240) + CH7沿 → 切换 0↔1.57
- 串口:  (offset 26) → 1=1.57, 2=0, 0=保持

**夹爪气缸 YV1 控制**:
- CAN ID 0x300,  开/得电,  关/失电,  翻转
-  每 50ms 刷新
- 遥控: CH6中档(632-1415) + CH7沿 → 切换开/关
- 串口:  (offset 27) → 1=开, 2=关, 0=保持

**CH6/CH7 组合遥控** (sbus_task, 10ms):
- CH6<632 (低档~240): CH7沿 → 翻转电机
- CH6 632-1415 (中档~1024): CH7沿 → 夹爪气缸
- CH6>1415 (高档~1807): 无操作

**构建状态**: 2026-06-08 build 成功, 0 errors, 0 warnings

**待办**:
- 左/右吸盘 (left_sucker/right_sucker) 硬件接入
- 右机械臂 CAN 总线确定
- 双机械臂 arm_task 重构
- hcan5 升降机构模块
- ToF 传感器接入 + 状态帧回传
- 抬升实际高度从编码器读取 (当前仅回传目标值)

---

### 历史变更 (保留供参考，部分信息已过时)

### 变更14: 机械臂新结构移植 (2026-05-20)
**原因**: 机械臂硬件更换，结构相同但臂长变化，末端电机新增减速器

**arm.h 臂长更新**:
- L1: 60→**40**, L2: 380→**340**, L3: 385.3→**370**, L4: 46→**50** (mm) *(注: 2026-06-06 实际代码中 L1=40)*

**arm.c 末端减速比**: `arm_ctrl()`: `pos_ctrl(&hcan2, 0x04, terminal * (80.0f / M_PI), 0.5f)`

**arm.c 禁区与限位**:
- FORBIDDEN_RADIUS: 120→**50** mm, Z_MAX: 150→**200** mm
- terminal 限位: 启用 **+25.0 / -30.0**
- pitch 限位: 启用 **+1.57 / -1.0**

**待机位置**: `(100, 0, 0, -1.57)`

### 变更1-13: (详见旧版 memory_box，已基本过时)

### 变更7: 串级 PID 恢复 + 参数调优 (2026-05-14/15) — ⚠️ 已过时，四舵轮已废弃
原四舵轮串级PID参数仅供历史参考：
- 角度外环: Kp=2.2, Ki=0.003, Kd=18.0
- 速度内环: Kp=3.0, Ki=0.01, Kd=0.001
- max_iout=5000, MIN_ANGLE_RATE=18 rad/s, STEER_SPEED_REF=0.6

### 变更8: IMU 模块新增 (2026-05-14)
- HLK-AS201 IMU, USART3 DMA 接收
- Yaw 一阶低通滤波 (α=0.9)
- `imu_task()`: 50ms 周期, 帧校验失败自动复位 DMA
- **当前状态**: `imu_task()` 未在 `main.c` 中创建

---

## 6. 当前工程文件结构 (2026-06-06)

```
R2/
├── Core/Src/main.c              # 主程序 (含待重构的 chassis_task/up_cs_task)
├── Lib/
│   ├── uart_task.c/h            # 串口任务库 (+传感器帧发送)
│   ├── imu.c/h                  # HLK-AS201 IMU 驱动
│   ├── motor_control.c/h        # DM/YUN电机CAN协议 (MIT/位置/速度模式)
│   ├── mcp2515.c/h/consts.h     # MCP2515 SPI-CAN驱动
│   ├── CAN_receive.c/h          # CAN接收
│   ├── sbus_set.c/h             # SBUS解码
│   ├── solenoid_valves.c/h      # 电磁阀宏 YV1-YV10
│   ├── pid.c/h                  # PID控制器
│   └── bsp_can.c/h              # CAN滤波器
│   ~~ four_steering_wheel_ik.c/h ~~  # ❌ 已删除 (2026-06-06)
│   ~~ upstairs.c/h ~~               # ❌ 已删除 (2026-06-06)
│   ~~ arm.c/h ~~                    # ❌ 已删除 (2026-06-06)
│   ~~ solenoid_valves.c/h ~~         # ❌ 已删除 (2026-06-06)
├── MDK-ARM/A_board.uvprojx      # Keil工程
└── README.md                    # 项目文档
```

## 7. 通信协议摘要 (未变)

**USART1 帧 (15字节)**: `[0xCC] [vyH] [vyL] [vxH] [vxL] [vwH] [vwL] [升降速度] [前距离] [后距离] [YV1] [YV2] [YV4] [YV5] [0xEE]`

**USART2 帧 (10字节)**: `[0xCC] [fbH] [fbL] [lrH] [lrL] [udH] [udL] [末端姿态] [夹爪] [0xEE]`

**USART3 传感器帧 (8字节, 5Hz)**: `[0xCC] [rear] [front] [0x00]x4 [0xEE]`

**IMU 帧**: `FA FB ... FC FD`, 14字节, 50Hz

**模式切换通知**: CH5下降沿→ `{0xCC,0,0,0,0,0,0x01,0xEE}` (USART1) + `{0xCC,0,0,0,0,0,0x02,0xEE}` (USART2) + 传感器帧 (USART3)

## 8. 全向轮机械参数 (待填入)

| 参数 | 值 |
|------|-----|
| 轮子布局 | 矩形, 待定尺寸 |
| 轮径 | 待定 |
| 电机 | 4×DM MIT, CAN1 |
| 控制周期 | ~2ms |

## 9. 机械臂参数 (⚠️ 待重新测量定义，双机械臂)

旧单臂参数已过时，新双机械臂参数待测量：
- 左右臂结构是否对称？连杆长度是否相同？
- 各自的安全禁区、关节限位、减速比
- 待机/回零位置各自定义
- 旧参考值 (仅历史): L1=40, L2=340, L3=370, L4=50 (mm), 末端减速比 80/π

## 10. 注意事项

- `ch_high()` 阈值 = **1700** (用户**有意设置**, 适配遥控器拨杆)
- 所有任务优先级相同(0), FreeRTOS 时间片轮询
- **无上电延时**
- SBUS 100ms 无有效帧自动失能
- YV9/YV10 bug 随 solenoid_valves 模块一同删除 (2026-06-06)
- USART3 共享总线: 当前传感器阻塞发送随系统运行；如果启用 `imu_task()` 的 DMA 接收，需注意发送期间可能丢 IMU 帧
- chassis_task / up_cs_task / arm_task **待重构**，当前为删除旧代码后的编译残留骨架
- 串口协议已变更，UART1/UART2 帧定义待更新

## 11. 构建信息 (最新)

| 项目 | 值 |
|------|-----|
| 日期 | 2026-06-06 |
| 编译器 | Keil MDK UV4 (C:\Programs\Keil_v5\UV4\UV4.exe) |
| 工程 | A_board.uvprojx, Target: A_board |
| 编译状态 | ⚠️ 待重新编译 (旧舵轮/升降模块已删除) |
| 调试器 | CMSIS-DAP |
