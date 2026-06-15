---
name: Robocon R1 项目上下文记忆匣
description: Robocon2026 R1 底盘控制固件开发全上下文：项目架构、代码细节、已知问题
type: project
---

# Robocon 2026 — R1 上层主控固件 记忆匣

## 1. 项目概述

- **项目**: Robocon 2026 赛季 R1 机器人上层主控制板固件 (自制 A 板)
- **MCU**: STM32F407VET6 (Cortex-M4F, 168MHz, 512KB Flash, 192KB SRAM)
- **硬件板**: 自制 A 板（与 R2 主控相同，非 Robomaster 开发板 A 型）
- **RTOS**: FreeRTOS V10.3.1 (CMSIS-OS v1 封装)
- **结构**: 四轮麦克纳姆轮底盘 + 机械臂 + 夹取/升降机构
- **控制方式**: SBUS 遥控器直连（与下层底盘共享接收机，Y 线并联），CH6 三段切换底盘/机械臂/夹取机构
- **IDE**: Keil MDK v5 (ARMCC V5.06)
- **工程文件**: `MDK-ARM/A_board.uvprojx`

## 2. 硬件架构关键信息

| CAN 总线 | 通路 | 用途 |
|----------|------|------|
| CAN1 (PD0/PD1) | 原生 | 机械臂 DM 电机 (`ARM_CAN=hcan1`) + YV 电磁阀刷新 |
| CAN2 (PB5/PB6) | 原生 | 底盘 4×DM4310 MIT 模式 (`chassis_hcan=hcan2`, ID 0x01-0x04) |
| hcan3 (MCP2515, SPI1+PA4) | 扩展 | 物理 SPI1 通道；当前代码用 `hcan5/grabbing_can` 绑定此通道 |
| hcan4 (MCP2515, SPI2+PB12) | 扩展 | 预留 |
| hcan5 (MCP2515, SPI1+PA4) | 扩展 | 当前 `grabbing_can`，夹取/升降机构 |

| UART | 引脚 | 参数 | 用途 |
|------|------|------|------|
| UART4 | PA0(TX)/PA1(RX) | 100kbps, 9E2, DMA | SBUS 遥控器 |
| USART1 | PA9(TX)/PA10(RX) | 115200-8N1 | 调试串口 |
| USART2 | PD5(TX)/PD6(RX) | 115200-8N1 | 通信串口 |

## 3. FreeRTOS 任务 (全部优先级 0, 时间片轮询)

| 任务 | 周期 | 栈(words) | 位置 | 功能 |
|------|------|-----------|------|------|
| `start_task` | 一次性 | 256 | main.c | 初始化 SBUS + CAN → 创建子任务 → 自销毁 |
| `sbus_task` | 10ms | 512 | main.c | SBUS 解码 + CH4 使能边沿 + CH6 模式切换 + 底盘/机械臂/夹取控制 |
| `chassis_task` | 5ms | 256 | main.c | 麦轮逆运动学 + CAN MIT 控制帧 |
| `led_task` | 100/200ms | 128 | main.c | GPIOE LED 流水灯 |

## 4. 控制流

```
SBUS 遥控器
    │ UART4 DMA
    ▼
sbus_task (10ms)
    │ CH3→set_vx, CH2→set_vy, CH0→set_vw
    │ CH4 上升沿/下降沿 → sys_enabled
    ▼
chassis_task (5ms)
    │ 麦轮逆运动学计算 (0.707 系数)
    │ motor_out[0..3] = f(vx, vy, vw)
    │ dm_mit_ctrl(CAN1, ID, pos=0, vel=motor_out[i], kp=0, kd=torque, torq=0)
    ▼
CAN1 → 4×DM4310 (MIT 速度模式)
```

## 5. 关键代码细节

### main.c 核心逻辑

**系统使能处理** (`system_enable_handler()`):
- CH4 > 1700 且上升沿 → `sys_enabled=true`, `chassis_enable()`
- CH4 < 1700 且下降沿 → `sys_enabled=false`, `chassis_disable()`, 速度归零
- SBUS 断联 >50ms → 自动失能

**SBUS 帧校验**:
```c
static inline bool sbus_frame_valid(void)
{
    return (sbus_rx_buf[23] == 0x00 && sbus_rx_buf[0] == 0x0f && sbus_rx_buf[24] == 0x00);
}
```

**速度映射** (含死区):
| 通道 | 映射范围 | 死区 | 输出范围 |
|------|----------|------|----------|
| CH3 (vx) | 240~1807 | 1022~1026 | -5.0~+5.0 m/s |
| CH2 (vy) | 250~1807 | 1022~1026 | -5.0~+5.0 m/s |
| CH0 (vw) | 242~1800 | 1022~1026 | -2.5~+2.5 rad/s |

注意：CH3(vx) 的映射范围是 240-1807，但死区是 1022-1026。这在中间附近会导致一个约 150 的跳变区域（从约 1026 到 1022），这实际上是正常的，因为 SBUS 信号在中位附近本来就有噪声。

### chassis.c 逆运动学

```c
// FR (0x02)
motor_out[1] =  0.707f * set_vx - 0.707f * set_vy + set_vw * chassis_r;
// FL (0x03)
motor_out[2] =  0.707f * set_vx + 0.707f * set_vy + set_vw * chassis_r;
// BR (0x01)
motor_out[0] = -0.707f * set_vx - 0.707f * set_vy + set_vw * chassis_r;
// BL (0x04)
motor_out[3] = -0.707f * set_vx + 0.707f * set_vy + set_vw * chassis_r;
```

每个电机依次发送，中间有 `vTaskDelay(1)` 间隔。
MIT 帧参数: `pos=0, vel=motor_out[i], kp=0, kd=chassis_torque, torq=0`

### motor_control.c MIT 协议帧格式

```
CAN data[8] 编码 (DM4310):
[0:1] pos    — 16-bit 位置 (-12.5 ~ +12.5 rad)
[2:3] vel    — 12-bit 速度 (-30.0 ~ +30.0 rad/s)
[3:4] kp     — 12-bit 刚度 (0 ~ 500 N·m/rad)
[5:6] kd     — 12-bit 阻尼 (0 ~ 5 N·m·s/rad)
[6:7] torq   — 12-bit 扭矩 (-10 ~ +10 N·m)
```

### pid.h 注意

重新声明了 `int8_t`、`uint8_t` 等标准类型，且未使用 `#define` 防护：
```c
typedef signed char int8_t;
typedef unsigned char uint8_t;
```
如果与其他包含 `<stdint.h>` 的模块混编可能产生类型冲突。

### solenoid_valves.h YV9/YV10 bug

```c
#define YV9(dat) \
    { \
        ... \
        else { can_can_send_data[0] ^= 0x01; }  \  // BUG: 应操作 [1] 而非 [0]
    }
```
YV9 和 YV10 的 flip (dat!=0 && dat!=1) 分支错误地操作了 `[0]` 而非 `[1]`。

## 6. 启动流程

```
main()
 ├─ HAL_Init()
 ├─ SystemClock_Config()
 ├─ MX_GPIO_Init()
 ├─ MX_DMA_Init()
 ├─ MX_CAN1_Init() / MX_CAN2_Init()
 ├─ MX_SPI1_Init() / MX_SPI2_Init() / MX_SPI3_Init()
 ├─ MX_UART4_Init()
 ├─ MX_USART1_UART_Init() / MX_USART2_UART_Init()
 ├─ MX_TIM1_Init()
 ├─ xTaskCreate(start_task, "start_task", 256, NULL, 0, NULL)
 ├─ MX_FREERTOS_Init()
 └─ osKernelStart()

start_task():
 ├─ sbus_rx_init()           # UART4 DMA CIRCULAR 启动
 ├─ can_filter_init()        # CAN1 滤波器
 └─ xTaskCreate(×3) + vTaskDelete(NULL)
     ├─ led_task     (128 words, prio 0)
     ├─ sbus_task    (512 words, prio 0)  // 名为 "remote_task"
     └─ chassis_task (256 words, prio 0)
```

## 7. 当前工程文件结构

```
R1/upperpart/
├── .gitignore                      # Git 忽略规则
├── A_board.ioc                     # CubeMX 引脚配置
├── README.md / CLAUDE.md           # 项目文档
├── memory_box_short.md / long.md   # 记忆匣
├── Core/Src/main.c                 # 主程序 (~591行)
├── Lib/
│   ├── chassis.c/h                 # 麦轮运动学 + 全局速度变量
│   ├── motor_control.c/h           # DM/YUN 电机 CAN 协议
│   ├── mcp2515.c/h/consts.h        # MCP2515 SPI-CAN 驱动
│   ├── can_receive.c/h             # CAN 接收 + 3508 控制 (预留)
│   ├── sbus_set.c/h                # SBUS 解码
│   ├── solenoid_valves.c/h         # 电磁阀宏 YV1-YV10
│   ├── pid.c/h                     # PID 控制器 (预留)
│   ├── bsp_can.c/h                 # CAN 滤波器
│   ├── arm.c/h                     # 机械臂关节控制 + IK
│   └── grabbing.c/h                # 夹取/升降机构控制
├── Doc/                            # 文档
├── MDK-ARM/A_board.uvprojx         # Keil 工程
└── MDK-ARM/A_board/                # 编译输出
```

## 8. 通信协议

### UART4 — SBUS 遥控器

| 参数 | 值 |
|------|-----|
| 波特率 | 100000 |
| 数据位 | 9 |
| 停止位 | 2 |
| 校验位 | Even |
| 帧长度 | 25 字节 |
| DMA | DMA1_Stream2, CIRCULAR |

16 通道 (CH0~CH15)，每通道 11-bit (0~2047)。
帧校验: `buf[0]==0x0F && buf[23]==0x00 && buf[24]==0x00`

## 9. 编译说明

- **Keil**: MDK v5, ARM Compiler V5.06 update 7
- **包**: STM32F4xx_DFP 3.1.1
- **Define**: `USE_HAL_DRIVER`, `STM32F407xx`
- **优化**: `-O4` (O3 + 自动内联)
- **C 标准**: C99 + GNU extensions
- **输出**: `MDK-ARM/A_board/A_board.hex` + `.axf`

## 10. 注意事项

- `chassis_r = 1.0` 和 `chassis_torque = 1.0` 是占位值，实车需标定
- 底盘当前为**纯开环速度控制**，无位置/速度闭环
- 4 个 DM4310 的使能和 MIT 控制依次发送，间隔 `vTaskDelay(1)`
- sbus_task 任务名为 `"remote_task"` (与函数名不一致)
- USART1/USART2 已初始化但 DMA 接收未启动 (无 `HAL_UART_Receive_DMA` 调用)
- 夹取机构 MCP2515 已通过 `grabbing_init()` 接入；`solenoid_valves` 通过 `YV_flash(&hcan1)` 刷新，但 YV9/YV10 flip bug 仍在
- HAL_TIM_PeriodElapsedCallback 由 TIM14 驱动，用作 HAL 时基 (SysTick 归 FreeRTOS)

## 11. 本次会话 (2026-05-11) 变更记录

### 变更1: 项目分析 + 文档创建
首次打开 R1 工程，进行全面分析，创建了 4 份文档：
- `README.md` — 完整的硬件/软件/协议文档
- `CLAUDE.md` — AI 助手的项目级配置文件 (自动加载)
- `memory_box_short.md` — 会话变更摘要
- `memory_box_long.md` — 详细项目上下文

### 变更2: 底盘从 CAN1 改到 CAN2
`Lib/chassis.h:8` — `#define chassis_hcan (hcan1)` → `(hcan2)`
CAN2 滤波器已在 `bsp_can.c` 中配置，DM 反馈由 `HAL_CAN_RxFifo1MsgPendingCallback` 分发。

### 变更3: 3508 代码迁移至 motor_control 库
**原因**: 统一电机控制接口到 motor_control 库，避免分散在多个文件中。

**涉及文件**:
- `Lib/motor_control.h` — 新增 `motor_measure_t` 结构体、`get_motor_measure` 宏、`CAN1/2/3_send_dat()` 声明
- `Lib/motor_control.c` — 新增 `motor_chassis[4]` / `can2_motor_chassis[4]` 全局变量、CAN 收发静态缓冲区、
  `HAL_CAN_RxFifo0MsgPendingCallback` / `HAL_CAN_RxFifo1MsgPendingCallback` (统一分发 DM + 3508)、
  `CAN1_send_dat()` / `CAN2_send_dat()` / `CAN3_send_dat_mcp2515()`
- `Lib/CAN_receive.c` / `Lib/CAN_receive.h` — 已清空，从 Keil 工程中移除

**HAL 回调架构**:
```
CAN1 RX FIFO0 → HAL_CAN_RxFifo0MsgPendingCallback
                  ├─ StdId 0x205-0x208 → 3508 (motor_chassis[i])
                  └─ 其他 → can1_rx_callback() [DM 电机]

CAN2 RX FIFO1 → HAL_CAN_RxFifo1MsgPendingCallback
                  ├─ StdId 0x205-0x208 → 3508 (can2_motor_chassis[i])
                  └─ 其他 → can2_rx_callback() [DM 电机]
```

### 变更4: 发现的工程问题清单
见上方第 10 节注意事项 + README.md 中的已知问题。

## 12. 本次会话 (2026-05-18) 变更记录

### 变更5: CH6 三段开关作为控制模式选择

`main.c` 中 `remote_mode_t` 当前为：
- `REMOTE_MODE_CHASSIS`: CH6 中位，底盘模式
- `REMOTE_MODE_ARM`: CH6 >1700，机械臂模式
- `REMOTE_MODE_GRABBING`: CH6 <1000，夹取机构模式

模式切换行为：
- 进入机械臂模式：底盘速度清零，同步 CH7 当前物理位置，再执行 `arm_back_zero()`。
- 进入夹取模式：底盘速度清零，同步 CH7 当前物理位置；如果抓取状态为 0，则记录当前 CH5 工位。
- 机械臂/夹取模式下底盘速度目标始终归零。

### 变更6: 机械臂与吸盘控制

机械臂模式 (`CH6 >1700`)：
- CH1: yaw 增量控制
- CH8: pitch 绝对映射 `[-0.7, 0.7]`
- CH9: elbow 绝对映射 `[-3.14, 0.7]`
- terminal: 固定 `0.0f`
- CH7: 每次切换取反 `suction_enabled` 并刷新 YV1

CH4 上升沿/下降沿都会将 `suction_enabled=false` 并 `YV1(0)`。

### 变更7: 夹取机构状态机

夹取模式 (`CH6 <1000`)：
- CH5 选择工位，`>1700` 为下工位，否则为上工位。
- CH7 任意边沿推进 `grabbing_step_count`。
- CH5 实时控制工位（`>1700` 下工位，否则上工位），步骤执行中随时可切换工位。
- 第 6 步后不循环回第 1 步；只有 CH4 失能会清零状态。

抓取步骤当前为 6 步：

| 次数 | 步骤宏 | 下工位函数 | 上工位函数 | 动作 |
|------|--------|------------|------------|------|
| 1 | `GRABBING_STEP_RETRACT` | `grabbing_section1()` | `grabbing_upper_section1()` | 推进收回到未伸出 |
| 2 | `GRABBING_STEP_CLAW_OPEN` | `grabbing_section2()` | `grabbing_upper_section2()` | 只打开当前工位夹爪 |
| 3 | `GRABBING_STEP_FLIP_DOWN` | `grabbing_section3()` | `grabbing_upper_section3()` | 升降到对应工位高度，推进保持收回，夹爪翻到取杆位置 |
| 4 | `GRABBING_STEP_CLAW_CLOSE` | `grabbing_section4()` | `grabbing_upper_section4()` | 只关闭当前工位夹爪 |
| 5 | `GRABBING_STEP_FLIP_BACK` | `grabbing_section5()` | `grabbing_upper_section5()` | 当前工位夹爪翻回 |
| 6 | `GRABBING_STEP_PUSH` | `grabbing_section6()` | `grabbing_upper_section6()` | 当前工位推进 |

安全重点：夹爪只能在推进未伸出时打开，所以 CH4 清零后第一下 CH7 只做 `RETRACT`，第二下 CH7 才允许 `CLAW_OPEN`。运动步骤不自动开/关夹爪。

### 变更8: 夹取机构模块接入

当前 `Lib/grabbing.c/h`：
- `grabbing_can` 宏定义为 `hcan5`
- `grabbing_init()` 调用 `mcp2515_sys_init(&hcan5, &hspi1, GPIOA, GPIO_PIN_4)`
- `grabbing_enable/disable()` 控制 ID 0x01~0x05
- `grabbing_run_step(station, step)` 统一分发上下工位动作

### 编译信息 (2026-05-18)

- Keil UV4 build 成功：0 errors, 0 warnings
- Flash: 21192 bytes, RAM: 23520 bytes
- hex: `MDK-ARM/A_board/A_board.hex`
- axf: `MDK-ARM/A_board/A_board.axf`
- log: `.embeddedskills/build/A_board-A_board-build.log`

## 13. 本次会话 (2026-06-15) 变更记录

### 变更9: 底盘解耦 — 骨架清理

上下层电源分离，底盘控制完全移交给下层 (Robomaster A 板)。上层 main.c 清理为最小骨架：

**移除的代码**:
- `chassis_task` 及 `chassis_updata()`、`YV_flash()`
- `chassis_remote_control()` / `stop_chassis_remote()`
- `arm_remote_control()` 及机械臂 IK 全局变量 (`arm_yaw`, `arm_r`, `fb_des`, `lr_des`, `ud_des`, `end_des`)
- `grabbing_remote_control()` 及夹取状态机 (`grabbing_step_count`, `push_target`, `last_ch7`)
- `REMOTE_MODE_CHASSIS/ARM/GRABBING` 模式枚举和 CH6 三段切换逻辑
- `suction_enabled` / `YV1()` 吸盘控制
- `sbus_speed_map` / `sbus_step` / `sbus_abs` 映射函数

**保留的代码**:
- LED 流水灯 (`led_task`, 128 words)
- SBUS 解码 + 50ms 断联保护 (`sbus_task`, 512 words, 10ms)
- CH4 锁车/解锁 (`system_enable_handler`)
- 启动任务 (`start_task`, 256 words, 一次性)
- CubeMX 生成的外设初始化 (CAN1/2, SPI1/2/3, UART4, USART1/2, TIM1)

**Lib 文件**: `chassis.c/h`, `arm.c/h`, `grabbing.c/h`, `solenoid_valves.c/h`, `motor_control.c/h`, `mcp2515.c/h`, `can_receive.c/h`, `pid.c/h`, `bsp_can.c/h` 全部保留在 Lib 目录，当前 main.c 不引用，待后续功能重新接入。

### 变更10: 遥控器参数修正

**问题**: 原代码 `ch_high()` 阈值 `>1700`，但实际遥控器 3 段拨杆最高位只有 1659，导致 CH4 永远无法解锁。

**遥控器实际量程** (AT9S Pro):

| 通道 | 控件 | 量程 |
|------|------|------|
| CH0~CH3 | 摇杆 | 326~1659 (中位 992) |
| CH4~CH6 | 3 段拨杆 | 326 / 992 / 1659 |
| CH7 | 2 段拨杆 | 329 / 1659 |
| CH8~CH11 | 旋钮 | 326~1659 |

**修正**:
- `ch_high()`: `>1700` → `>1300` (中位 992 和高位 1659 之间)
- 新增 `ch_low()`: `<650` (低位 326 和中位 992 之间)
- 新增 `ch_mid()`: `≥650 && ≤1300`
- `SBUS_CENTER` 从 1024 → 992 (匹配实际摇杆中位)

**硬件连接**: R1 上下层各有独立 SBUS 接收机，绑定同一遥控器。两块板之间无任何线路连接。

### 文档更新
- `README.md` — 更新遥控器规格、通道映射、锁车逻辑，新增 2026-06-15 变更记录
- `memory_box_short.md` — 重写为当前骨架状态
- `memory_box_long.md` — 追加本变更记录
- 新增 `.gitignore`，创建 `Doc/` 目录
