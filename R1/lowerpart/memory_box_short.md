# R1 底盘主控 — 短记忆匣

## 项目
STM32F407VET6 + FreeRTOS，**R1 底盘车**。赛季首台验证车，负责 DM 电机驱动库搭建和 SBUS 遥控链路验证。Keil MDK-ARM (`master_a.uvprojx`)。

## 当前状态

| 子系统 | 状态 | 说明 |
|--------|------|------|
| 电机驱动库 | ✅ 完成 | 7 种 DM 型号 + YUN 系列, MIT/POS/SPD/PSI 四模式, per-motor 参数表 |
| SBUS 遥控 | ✅ 完成 | USART1 DMA 循环, 帧校验, 50ms 超时断联 |
| CAN 滤波器 | ✅ 完成 | hcan1 + hcan2, 全通模式 |
| 底盘运动学 | ❌ 待开发 | chassis_task 空骨架, 全向轮/麦克纳姆轮运动学待实现 |
| 电机使能流程 | ❌ 待开发 | start_task 中无 dm_enable 调用 |
| CAN 反馈 | ⚠️ 待验证 | dm_rx_cbk 调用在 CAN_receive.c 中被注释掉 |
| 云台/武器 | ⚠️ 注释 | USART6 云台控制代码在 main.c 中, 当前注释状态 |

## 关键参数

| 参数 | 值 |
|------|-----|
| MCU | STM32F407VET6, 168MHz |
| SBUS | USART1 (PB6/PB7), 100kbps, 9-bit Even 2-stop |
| CH4 使能阈值 | 1600 (`ch_down()` 函数) |
| SBUS 超时 | 50ms |
| 摇杆中位死区 | 991~993 → 0 |
| CAN1/2 | PD0/PD1, PB5/PB6 |
| USART6 (预留) | PG9/PG14, 115200-8N1, DMA RX+TX |

## motor_control 模块关键设计

此模块是 R1 最重要的输出，设计经过打磨，值得后续车型参考：

- **per-motor 范围**: `motor_t` 自带 `p_max/v_max/t_max`，通过 `dm_model_t` 枚举 + `dm_range_table[]` 查找表管理
- **mode 自动偏移**: `motor->mode` → `mode_offset[]` → CAN ID 自动加 `MIT_MODE` / `POS_MODE` 等
- **统一反馈回调**: `dm_rx_cbk(motor_set, rx_data)` 自动提取 ID 分发 — 一个函数覆盖所有 CAN 路
- **命令统一**: 使能/失能/清零/清错共用一个 `dm_send_cmd()` 内部函数

### 型号表
| 型号 | P_MAX | V_MAX | T_MAX |
|------|-------|-------|-------|
| DM_4310 | 12.5 | 30 | 10 |
| DM_4310_48V | 12.5 | 45 | 18 |
| DM_4340 | 12.5 | 30 | 27 |
| DM_3519 | 12.5 | 20 | 8 |
| DM_8006 | 12.5 | 21 | 20 |
| DM_8009 | 12.5 | 18 | 40 |

## FreeRTOS 任务

| 任务 | 周期 | 栈 | 说明 |
|------|------|-----|------|
| start_task | 一次性 | 256 | 初始化, 创建子任务后自销毁 |
| remote_task | ~4ms | 128 | SBUS 解码 + CH4 使能管理 |
| chassis_task | ~2ms | 128 | 空骨架 |
| led_task | 200ms | 128 | 流水灯 |

## 已知问题

1. **dm_rx_cbk 被注释**: `CAN_receive.c` 中 CAN1/CAN2 中断回调里的 `dm_rx_cbk()` 调用被注释掉了 → DM 电机反馈数据不会被解析。可能是调试时临时注释，需恢复。
2. **chassis_task 空骨架**: 没有任何运动学代码，底盘不会动。
3. **没有电机使能流程**: `start_task` 中没有 `dm_enable` 调用 → 电机上电后不会自动使能。
4. **USART6 云台代码为注释状态**: 云台 yaw/pitch 控制和电磁阀 YV6/YV7/YV8 操作全部被注释，不含在当前工作代码中。
5. **`ch_down()` 命名误导**: 函数名暗示下降沿判断，但实际是 `>1600` 判断，语义为 ch_high。R2 中已修正为 `ch_high()`。
6. **PID 模块有独立类型定义**: `pid.h` 中重新定义了 `int8_t/uint8_t` 等标准类型，可能与 `stdint.h` 冲突（好在 `main.h` 引入了 `stm32f4xx_hal.h` 已经包含标准类型）。

## 与 R2 的关系

R1 → R2 代码迁移路径:
- `motor_control.c/h` → 被 R2 采用但简化了 (去掉了 dm_model_t 表, 改为全局硬编码)
- `sbus_set.c/h` → 直接复制到 R2
- `pid.c/h` → 直接复制到 R2
- `bsp_can.c/h` → 复制并修改了滤波器编号 (Bank 0/14 vs R1 的配置)
- `CAN_receive.c/h` → 复制并新增了 MCP2515 发送函数

R2 新增模块 (R1 没有):
- `mcp2515.c/h` — SPI-CAN 外部控制器
- `chassis.c/h` — 全向轮运动学 + 抬升
- `arm.c/h` — 双机械臂
- `imu.c/h` — IMU 姿态
- `uart_task.c/h` — 串口协议
- `solenoid_valves.c/h` — 电磁阀

## 详细上下文
见同目录下 [memory_box_long.md](memory_box_long.md)
