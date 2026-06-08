# R2 主控固件 — 短记忆匣 (2026-06-08 归档)

## 项目
STM32F407VET6 + FreeRTOS，**四45°全向轮底盘** + **4独立齿条抬升(DM4310)** + 左右双机械臂(待开发)。SBUS遥控(手动)/上位机串口(自动)双模式。Keil MDK-ARM。

## 当前状态 ✅

| 子系统 | 状态 | 说明 |
|--------|------|------|
| 底盘全向轮 | ✅ 完成 | 45°运动学, CAN1 ID 1~4, 雷达实测速度OK, 校准系数×1.9 |
| 独立抬升 | ✅ 完成 | DM4310位置-速度, CAN1 ID 5~8, 行程417mm, RACK=20mm/rad, lift_mode快慢速 |
| 串口协议 | ✅ 完成 | 30B控制帧 USART1收, 27B状态帧 USART2发 50Hz |
| 夹爪翻转 | ✅ 完成 | hcan3 ID 0x01, CH6低+CH7沿切换 0↔1.57, 串口weapon_pitch控制 |
| 夹爪气缸 | ✅ 完成 | hcan3 YV1 (CAN ID 0x300), CH6中+CH7沿切换开/关, 串口weapon_gripper控制 |
| 双机械臂 | ❌ 待开发 | arm_task 空骨架 |
| 升降机构 | ❌ 待开发 | hcan5 模块待写 |

## 关键参数

| 参数 | 值 |
|------|-----|
| 底盘 | 350×350mm, 轮径127mm, CHASSIS_R=0.245 |
| 运动学 | 45°全向轮, COS45=0.707, SPEED_SCALE=0.0001, 实测×1.9 |
| 抬升 | ID 5~8, 方向全为向下撑, 行程0~417mm, slow=1.0 fast=2.0 rad/s |
| 翻转 | hcan3 ID 0x01, slow=0.5 fast=2.0 rad/s, 0=竖起 1.57=翻下, 上电不动作 |
| 电磁阀 | hcan3 YV1 (CAN ID 0x300, data[0] bit0), 1=开/得电 0=关/失电 |
| 遥控 | CH4使能, CH5低=手动/高=自动, CH8/CH9前后抬升 |
| CH6/CH7 | CH6低(~240)=翻转 CH6中(~1024)=气缸, CH7沿=切换 |
| 协议 | VEL_SCALE=10000=1m/s, weapon_pitch→翻转 weapon_gripper→YV1 |

## 详细上下文
见同目录下 `memory_box_long.md`

## 会话变更 (2026-06-06) — 机械结构重构清理

### 1. 删除旧舵轮运动学模块
- 删除 `Lib/four_steering_wheel_ik.c` 和 `Lib/four_steering_wheel_ik.h`
- 原因: 四舵轮(3508转向串级PID + DM驱动) → 四全向轮(4×DM MIT), 运动学完全不同

### 2. 删除旧升降模块
- 删除 `Lib/upstairs.c` 和 `Lib/upstairs.h`
- 原因: 云台电机 MIT 模式 → DM 电机位置-速度模式

### 3. main.c 清理
- 移除 `#include "four_steering_wheel_ik.h"`、`#include "upstairs.h"`、`#include "arm.h"`
- `start_task` 移除 `FourSteeringWheel_Init()`、`upstairs_init()` 调用
- `chassis_task` / `up_cs_task` / `arm_task` 保留骨架，待按新机械结构重写

### 4. 删除旧机械臂模块
- 删除 `Lib/arm.c` 和 `Lib/arm.h`
- 原因: 单机械臂 → 左右双机械臂，IK/控制完全不同
- `fb_des/lr_des/ud_des/end_des` 定义迁至 `uart_task.c`
- `arm_task` 保留为空骨架

### 5. uart_task 模块变更
- 移除 `#include "four_steering_wheel_ik.h"` 和 `#include "arm.h"`
- `fb_des/lr_des/ud_des/end_des` 定义从 arm.c 迁至 uart_task.c
- 添加 `extern float fb_des, lr_des, ud_des, end_des` 到 uart_task.h

### 6. 删除旧电磁阀模块 + 协议清理
- 删除 `Lib/solenoid_valves.c` 和 `Lib/solenoid_valves.h`
- 原因: 10路气缸/吸盘 → 仅保留1个夹爪气缸 (待重新实现)
- 移除 main.c 中所有 YV1~YV10 宏调用
- 移除 uart_task.c 中电磁阀解析和 gripper_cmd/uart2_state/uart2_take
- UART1/UART2 协议字段标记为待更新
- hcan4 MCP2515 初始化内联 (保留给夹爪翻转电机 DM 位置模式)

### 7. 新增底盘模块 chassis.c/h ✅ 已验证
- **45° 全向轮**运动学 (0.707 投影, 非麦克纳姆)
- CAN1 ID: BR=1, FR=2, FL=3, BL=4 (DM MIT)
- 独立抬升: FR=5, FL=6, BL=7, BR=8 (DM 位置-速度)
- 速度缩放 `SPEED_SCALE=0.025`, 方向已修正 (Vx/Vw 负号)
- 底盘几何: 347×347mm, CHASSIS_R=0.245m, CHASSIS_TORQUE=1.0
- 方向验证: ✅ 前后/横向/旋转均正确

### 8. 文档更新
- README.md 重写机械结构章节
- memory_box 新增本条变更记录

## 新机械结构速查

| 部件 | 电机 | 总线 | 控制模式 |
|------|------|------|----------|
| 全向轮 ×4 | DM | CAN1 (ID 1~4) | MIT |
| 独立抬升 ×4 | DM4310 | CAN1 (ID 5~8) | 位置-速度 |
| 升降机构 ×2 | DM | hcan5 (MCP2515 SPI3+PA15) | ⚠️ 待定 |
| 左机械臂 ×4 | DM | CAN2 | 位置 |
| 右机械臂 ×4 | DM | ⚠️ 待定 | 位置 |
| 夹爪翻转 ×1 | DM | hcan4 (MCP2515 SPI2+PB12) | 位置 |
| 夹爪气缸 ×1 | 电磁阀 | ⚠️ 待定 | 位操作 |

---

## 会话变更 (2026-06-08) — 夹爪翻转+气缸串口接入

### 1. 抬升 fast/slow 速度
- `LIFT_VEL_SLOW=1.0` `LIFT_VEL_FAST=2.0` rad/s, `lift_update(hcan, vel)` 加 vel 参数
- 手动固定 slow, 自动按 `lift_mode` (0=slow 1=fast)

### 2. 夹爪翻转电机迁至 hcan3
- hcan3 ID 0x01 (原 hcan4), `gripper_flip_pos`: 0=竖起 1.57=翻下
- 速度: `GRIPPER_FLIP_VEL_SLOW=0.5` / `GRIPPER_FLIP_VEL_FAST=2.0`
- 上电安全: `gripper_flip_ready=false`, 首次 CH7 操作或串口非零指令后才开始控制

### 3. 夹爪气缸 YV1 接入 hcan3
- CAN ID 0x300, `YV_flash_mcp2515(&hcan3)` 每周期刷新
- `YV1(1)` 开/得电, `YV1(0)` 关/失电, `YV1(2)` 翻转
- 复用 `solenoid_valves.c/h` (CAN ID 0x300, 8bit 位掩码)

### 4. CH6/CH7 组合遥控
- CH6<632 (低档~240): CH7沿→翻转电机 0↔1.57
- CH6 632-1415 (中档~1024): CH7沿→YV1 开↔关
- CH6>1415 (高档~1807): 无操作

### 5. 串口协议武器控制
- `weapon_pitch` (offset 26): 1→翻下(1.57) 2→竖起(0) 0→保持
- `weapon_gripper` (offset 27): 1→YV1(1)开 2→YV1(0)关 0→保持
- 自动模式: 翻转 fast speed, 手动模式: 翻转 slow speed

### 6. solenoid_valves 模块恢复
- `solenoid_valves.c/h` 保留在工程中, 用于 YV1 电磁阀位操作

## 历史记录 (仅保留关键参考，多数已过时)

### 编译信息 (2026-05-15，旧工程)
UV4 rebuild: **0 errors, 4 warnings**, Flash 28KB, RAM 24KB

### 关键参数 (仍适用)
- `ch_high()` = **1700** (非1800!)
- 所有任务优先级=0，时间片轮询
- SBUS断联100ms失能
- 机械臂臂长 ⚠️ 待重新测量 (双机械臂结构可能不同)

### 已知问题
- USART3 共享总线: 若启用 IMU DMA 接收，发送期间可能丢 IMU 帧
- chassis_task / up_cs_task ✅ 已接入 chassis 模块 (全向轮+抬升)
- arm_task 待重构 (双机械臂)
- 串口协议已变更，UART1/UART2 协议文档待更新

## 详细上下文
见同目录下 `memory_box_long.md`
