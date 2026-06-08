# Robocon 2026 — R2主控固件 (A板)

## 概述

本项目为 Robocon 2026 赛季 R2 机器人的主控制板固件，基于 **STM32F407VET6** (Cortex-M4F, 168MHz) + **FreeRTOS** 实时操作系统开发。

机器人采用**四全向轮底盘 + 升降机构 + 左右双机械臂**的结构，主控负责底层驱动（电机控制、电磁阀控制），通过 SBUS 遥控器直连控制，或接收上位机（小电脑）串口指令实现对底盘和机械臂的控制。

> **2026-06-08 夹爪翻转+气缸串口接入** | 2026-06-06 机械结构重大变更: 四舵轮 → 四全向轮，升降机构更换电机，单机械臂 → 左右双机械臂。

---

## 硬件架构

### 主控芯片
- **MCU**: STM32F407VET6 (Cortex-M4F, 168MHz, 512KB Flash, 192KB SRAM)

### 外设资源与引脚映射

| 外设 | 引脚 | 用途 |
|------|------|------|
| CAN1 | PD0/PD1 | 底盘: 全向轮×4 (DM MIT) + 独立抬升×4 (DM4310 位置-速度) |
| CAN2 | PB5/PB6 | 左机械臂电机 (DM系列, 位置模式) |
| SPI1 + PA4 (CS) | PA5/PA6/PA7 | MCP2515 → hcan3，夹爪翻转 DM + 电磁阀 YV1 |
| SPI2 + PB12 (CS) | PB13/PB14/PB15 | MCP2515 → hcan4，预留/待分配 |
| SPI3 + PA15 (CS) | PB3/PB4/PB5 | MCP2515 → hcan5，升降机构 DM 电机 (位置-速度模式) |
| UART4 | PA0(TX)/PA1(RX) | SBUS 遥控器接收 (100kbps, 9-bit, Even, 2-stop) |
| USART1 | PA9(TX)/PA10(RX) | 上位机通信：底盘速度 + 电磁阀 + 升降距离 (115200-8N1) |
| USART2 | PD5(TX)/PD6(RX) | 上位机通信：机械臂位置指令 (115200-8N1) |
| USART3 | PD8(TX)/PD9(RX) | 传感器帧发送 (阻塞, 5Hz)；IMU 欧拉角接收模块已实现但当前未随系统启动 |
| TIM1 | — | 通用定时器 |
| TIM14 | — | FreeRTOS 系统时基 |
| GPIOE[0:7] | PE0~PE7 | LED 流水灯 (开漏输出) |
| GPIOE[9] | PE9 | 前传感器 (GPIO输入, 无上下拉) |
| GPIOE[11] | PE11 | 输入 (预留) |
| GPIOE[13] | PE13 | 后传感器 (GPIO输入, 无上下拉) |
| KEY1 | — | 按键输入 (下拉) |

### 执行器

| 执行器 | 数量 | 控制总线 | 电机类型/控制方式 | 说明 |
|--------|------|----------|-------------------|------|
| 全向轮电机 | 4 | CAN1 (ID 1~4) | DM MIT 模式 | 四全向轮驱动 (chassis_update) |
| 独立抬升电机 | 4 | CAN1 (ID 5~8) | DM4310 位置-速度模式 | 齿条抬升, 4轮独立高度 (lift_update) |
| 升降机构电机 | 2 | hcan5 (MCP2515) | ⚠️ 待定 | 前后模块伸展/回收 (待重构) |
| 左机械臂电机 | 4 | CAN2 | DM 系列 位置模式 | 左臂: 偏航/俯仰/肘/末端 (待开发) |
| 右机械臂电机 | 4 | ⚠️ 待定 | DM 系列 位置模式 | 右臂: 偏航/俯仰/肘/末端 (待开发) |
| 夹爪翻转电机 | 1 | hcan3 (MCP2515) | DM 系列 位置模式 | 夹爪翻转: 0=竖起 1.57=翻下, 上电不动作 |
| 夹爪气缸 | 1 | hcan3 (MCP2515) | 电磁阀 YV1 | CAN ID 0x300, 1=开 0=关 |

---

## 软件架构

### 目录结构

```
R2_0606/
├── Core/Src/               # CubeMX 生成 + 主逻辑
│   ├── main.c              # 主程序：任务创建 + 控制逻辑
│   ├── freertos.c          # FreeRTOS 配置 (静态分配 Idle 任务)
│   ├── can.c / dma.c / gpio.c / spi.c / tim.c / usart.c
│   ├── stm32f4xx_it.c      # 中断服务 (CAN1/2 接收回调)
│   └── stm32f4xx_hal_msp.c # HAL MSP 初始化
├── Lib/                    # 应用层库
│   ├── sbus_set.c/h        # SBUS 遥控器解码 (UART4 DMA)
│   ├── bsp_can.c/h         # CAN 滤波器初始化
│   ├── mcp2515.c/h         # MCP2515 SPI-CAN 驱动
│   ├── mcp2515_consts.h    # MCP2515 寄存器定义
│   ├── CAN_receive.c/h     # CAN 数据接收
│   ├── motor_control.c/h   # DM/YUN 电机 MIT/位置模式控制
│   ├── pid.c/h             # PID 控制器 (位置式/增量式)
│   ├── chassis.c/h         # [新] 全向轮运动学 + 独立抬升控制
│   ├── uart_task.c/h       # 串口任务 (UART1/2 解析 + USART3 传感器帧)
│   └── imu.c/h             # HLK-AS201 IMU 欧拉角解析
├── Drivers/                # STM32 HAL + CMSIS
├── Middlewares/            # FreeRTOS 源码
└── MDK-ARM/                # Keil 工程文件
```

> **已删除**: `four_steering_wheel_ik.c/h`（四舵轮运动学）、`upstairs.c/h`（旧升降）、`arm.c/h`（旧单机械臂）— 均于 2026-06-06；`solenoid_valves.c/h` 恢复使用 (YV1 电磁阀)

### FreeRTOS 任务

| 任务 | 周期 | 栈 (words) | 优先级 | 功能 |
|------|------|-----------|--------|------|
| `start_task` | 一次性 | 256 | 0 | 初始化外设和子系统，创建子任务后自销毁 |
| `sbus_task` | 10ms | 256 | 0 | SBUS 解码 + 遥控速度映射 + 模式切换 + CH4 使能边沿检测 + CH5下降沿通知 |
| `uart_task` | 100ms | 128 | 0 | 串口命令解析 (UART1 底盘/阀 + UART2 机械臂) + USART3 传感器帧 5Hz |
| `chassis_task` | ~2ms | 1280 | 0 | 全向轮运动学 + CAN1 MIT 驱动 (chassis_update) |
| `up_cs_task` | ~50ms | 256 | 0 | 抬升电机位置控制 + 夹爪翻转电机 (lift_update) |
| `arm_task` | 10ms | 512 | 0 | ⚠️ 左右双机械臂控制 (待重构) |
| `led_task` | 200ms | 56 | 0 | GPIOE[0:7] 流水灯 |

> ⚠️ `chassis_task` 和 `up_cs_task` 为旧舵轮/旧升降代码的残留骨架，需按新机械结构重写。`imu_task()` 模块已实现但当前 `main.c` 未创建该任务。

### 启动流程

```
main()
 ├─ HAL_Init()
 ├─ SystemClock_Config()         # HSE → PLL (168MHz)
 ├─ MX_GPIO / DMA / CAN1/2 / SPI1/2/3 / UART4 / USART1/2/3 / TIM1 _Init()
 ├─ xTaskCreate(start_task)      # 创建初始化任务
 ├─ MX_FREERTOS_Init()           # 创建 defaultTask (空闲)
 └─ osKernelStart()             # 启动调度器

start_task():
 ├─ sbus_rx_init()               # 启动 UART4 DMA 接收 SBUS
 ├─ can_filter_init()            # 配置 CAN1/2 滤波器
 ├─ uart_rx_init()               # 启动 USART1/2 DMA 接收
 ├─ YV_mcp2515_init()            # 电磁阀 MCP2515 初始化
 ├─ xTaskCreate(×6)              # 创建全部子任务
 └─ vTaskDelete(NULL)            # 自销毁
```

---

## 控制模式

系统通过遥控器 CH4/CH5/CH6 通道切换工作模式：

```
CH4 (主使能, 阈值1700)
├── OFF (<1700 或 SBUS断联)
│   └── 所有子系统失能，速度归零
│
└── ON  (>1700)
    ├── CH5=高, CH6=低  →  手动遥控模式
    │   ├── CH0 → vw (角速度, Map: 268~1783 → -300~300)
    │   ├── CH1 → 升降底盘速度 (Map: 270~1800 → -10~10)
    │   ├── CH2 → vx (前进速度, Map: 256~1800 → -180~180)
    │   ├── CH3 → vy (横向速度, Map: 240~1780 → -180~180)
    │   ├── CH6低(~240) + CH7沿 → 切换翻转电机 (0↔1.57)
    │   ├── CH6中(~1024) + CH7沿 → 切换夹爪气缸 (开↔关)
    │   ├── CH8 → 前抬升高度 (0~417mm)
    │   ├── CH9 → 后抬升高度 (0~417mm)
    └── CH5=低  →  自动模式 (上位机控制)
        ├── USART1 : 底盘速度 (vx,vy,vw) + 前后升降高度 + lift_mode + 双机械臂 pitch + 吸盘 + weapon_pitch + weapon_gripper
        ├── USART2 : 状态帧回传 (升降高度/ToF/机械臂角度) 50Hz
        └── CH5 下降沿 → 向上位机发送模式切换通知
```

> **注意**: `ch_high()` 阈值 = **1700** (有意调低适配遥控器拨杆，非 1800)。

### 模式切换通知

当 CH5 从高切换到低（进入自动模式）时，主控通过 UART 向上位机发送通知帧：
- USART1 发送: `{0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xEE}`
- USART2 发送: `{0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xEE}`
- USART3 同步发送当前传感器帧

---

## 通信协议 (⚠️ 2026-06-06 协议已变更，以下旧协议待更新)

### UART4 — SBUS 遥控器

| 参数 | 值 |
|------|-----|
| 波特率 | 100000 |
| 数据位 | 9 |
| 停止位 | 2 |
| 校验位 | Even |
| 帧长度 | 25 字节 |
| DMA | DMA1_Stream2, 循环模式 |

- 15 通道 (CH0~CH15)，每通道 11-bit (0~2047)
- 帧校验: `buf[0]==0x0F && buf[23]==0x00 && buf[24]==0x00`
- 校验失败自动复位 DMA 重新开始接收

### USART1 数据帧 (⚠️ 旧15B协议文档, 新30B协议见 rc26_vehicle_serial_protocol_final.md)

```
帧头    vy高  vy低  vx高  vx低  vw高  vw低  升降  前距离 后距离 阀1   阀2   阀4   阀5   帧尾
0xCC    [1]   [2]   [3]   [4]   [5]   [6]   [7]   [8]    [9]   [10]  [11]  [12]  [13]  0xEE
```

| 偏移 | 类型 | 说明 |
|------|------|------|
| 0 | uint8 | 帧头 0xCC |
| 1-2 | int16 | vy 横向速度 (钳位: ±200, 解析后 ×0.1 映射为 set_vy) |
| 3-4 | int16 | vx 前进速度 (钳位: ±200, 解析后 ×0.1 映射为 set_vx) |
| 5-6 | int16 | vw 角速度 (钳位: ±200, 解析后 ×0.1 映射为 set_vw) |
| 7 | int8 | 升降底盘速度 (钳位: ±100, 映射为 -5~5 m/s) |
| 8 | uint8 | 前模块距离 (0~210mm) |
| 9 | uint8 | 后模块距离 (0~210mm) |
| 10 | uint8 | weapon_pitch (0=保持 1=平行地面 2=垂直地面) ⚠️ 字段位置已变更至新30B协议 |
| 14 | uint8 | 帧尾 0xEE |

### USART2 数据帧 (⚠️ 旧10B协议文档, 新27B状态帧见 rc26_vehicle_serial_protocol_final.md)

```
帧头    fb高  fb低  lr高  lr低  ud高  ud低  末端  夹爪  帧尾
0xCC    [1]   [2]   [3]   [4]   [5]   [6]   [7]   [8]  0xEE
```

| 偏移 | 类型 | 说明 |
|------|------|------|
| 0 | uint8 | 帧头 0xCC |
| 1-2 | int16 | fb 前后位置 (钳位: -800~800) |
| 3-4 | int16 | lr 左右位置 (钳位: -800~800) |
| 5-6 | int16 | ud 上下位置 (钳位: -800~800) |
| 7 | uint8 | weapon_gripper (0=保持 1=打开 2=闭合) ⚠️ 字段位置已变更至新30B协议 |
| 9 | uint8 | 帧尾 0xEE |

### USART3 传感器帧 (主控 → 上位机, 8字节, 5Hz)

```
帧头   rear  front  0x00   0x00   0x00   0x00   帧尾
0xCC   [1]   [2]    [3]    [4]    [5]    [6]   0xEE
```

- rear (PE13): 0x01=无障碍, 0x00=有障碍 (PIN_SET→0x00, PIN_RESET→0x01)
- front (PE9): 同上逻辑
- 发送方式: 单次 `HAL_UART_Transmit` 阻塞发送，每 2 个 uart_task 周期发送一次 (5Hz)

### IMU 数据 (HLK-AS201 → 主控, USART3 DMA)

- 协议帧: `FA FB ... FC FD`, 14字节/帧
- 上电配置: 发送 `FA FB 03 1F 04 23 FC FD` 订阅仅欧拉角
- 解析: Roll/Pitch/Yaw 原始 int16 × 0.0054931640625° → 度
- Yaw 一阶低通滤波: `α = 0.9`
- `imu_task()`: 50ms 周期, 帧校验失败自动复位 DMA；当前代码未创建该任务
- 输出变量: `imu_roll, imu_pitch, imu_yaw, imu_yaw_filtered, imu_ok`

---

## 安全机制

1. **SBUS 帧校验**: 校验字节 0 (0x0F) + 字节 23 (0x00) + 字节 24 (0x00)，校验失败自动复位 DMA
2. **SBUS 超时断联**: 100ms 无有效帧 → `sys_enabled=false` → 所有电机失能 + 速度归零
3. **CH4 边沿检测**: 上升沿统一使能各子系统，下降沿统一失能 + 安全停车
4. **范围钳位**: 所有速度/位置输入经 min/max 边界限制
5. **DMA 异常复位**: UART 帧头尾校验失败时，停止 DMA、清零缓冲区、重新启动接收
6. **机械臂安全区域** ⚠️ 待重新定义 (双机械臂需各自独立禁区)
7. **关节限位** ⚠️ 待重新定义 (双机械臂需各自独立限位)

## 已知问题

- YV9/YV10 宏仍有数组索引 bug (YV9 YV10 翻转操作错误使用 can_can_send_data[0])，当前未使用 YV9/YV10
- USART3 共享总线: 当前只有传感器帧发送随系统运行；若启用 IMU DMA 接收，发送期间可能丢 IMU 帧
- `arm_task` 为旧代码残留骨架，需按双机械臂重写
- 抬升实际高度回传为假数据 (lift_front_actual/lift_back_actual = target)，需从编码器读取
