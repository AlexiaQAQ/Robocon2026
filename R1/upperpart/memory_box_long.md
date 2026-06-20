---
name: Robocon R1 上层主控 记忆匣
description: R1 上层固件完整上下文：架构、状态机、遥控映射、已知问题
type: project
---

# R1 上层主控 — 长记忆匣

## 1. 项目概述

- **MCU**: STM32F407VET6, 自制A板
- **RTOS**: FreeRTOS, 6个任务, 全部优先级0
- **CAN**: CAN1=抬升+抓取+YV, CAN2=机械臂
- **SBUS**: UART4 DMA循环, 独立接收机
- **结构**: 模块化 — main.c(调度+锁车) + lift.c + arm.c + grab.c

## 2. 硬件明细

### CAN1 (hcan1)
| ID | 型号 | 模式 | 模块 | 方向 |
|----|------|------|------|------|
| 1 | DM4340 | POS | 抬升 FR | 向上为负 |
| 2 | DM4340 | POS | 抬升 FL | 向上为正 |
| 3 | DM4340 | POS | 抬升 BL | 向上为负 |
| 4 | DM4340 | POS | 抬升 BR | 向上为正 |
| 5 | DM4310 | POS | 抓取翻转 | — |
| 6 | DM2325 | POS | 抓取齿条 | — |
| — | — | — | YV_flash | CAN ID 0x300 |

### CAN2 (hcan2)
| ID | 型号 | 模式 | 关节 | 范围 |
|----|------|------|------|------|
| 1 | DM4340 | POS | 左根 | 0~-1.67, 0=水平 |
| 2 | DM4310 | POS | 左末 | 0~-1.57, 0=顺臂 |
| 3 | DM4340 | POS | 右根 | 0~1.67, 0=水平 |
| 4 | DM4310 | POS | 右末 | 0~1.57, 0=顺臂 |

## 3. 状态机

### 抬升 (lift.c)
```
IDLE → 收目标值 → 50Hz发送pos_ctrl
速度: 目标=0.2时 RETRUN_SPEED(2.0), 否则 LIFT_SPEED(4.0)
```

### 机械臂 (arm.c)
```
每臂独立: ARM_UP ↔ ARM_DOWN
CH5中位+CH11选臂+CH7边沿 → 切换状态
  UP:   root=±1.67, tip=0 (竖起)
  DOWN: root=0, tip=楼层决定
arm_task 每帧限幅夹后发送
首次进入 arm_update 仅同步CH7, 不切换
```

### 抓取 (grab.c)
```
G_IDLE(0) → G_RACK_ZERO(1) → G_CLAW_OPEN(2) → G_FLIP_DOWN(3)
→ G_CLAW_CLOSE(4) → G_FLIP_BACK(5) → G_MANUAL(6+)
CH5上拨+CH7边沿推进
CH10始终控制齿条
4310速度: 翻下FLIP_DOWN_SPEED, 翻上FLIP_UP_SPEED
```

## 4. 遥控器物理↔SBUS

物理拨杆与SBUS值反相: 物理上拨 → ch_low(<650)

| CH | 物理控件 | 模式 | ch_low | ch_mid | ch_high |
|----|---------|------|--------|--------|---------|
| 4 | 3段 | 全局 | — | 解锁+清抓取 | 解锁 |
| 5 | 3段 | 全局 | 未用 | 抬升+臂 | 抓取 |
| 6 | 3段 | 中位=抬升高度 | 19.0 | 28.8 | 29.3(3层) |
| 7 | 2段 | 中位=臂切换, 上拨=工序推进 | — | — | — |
| 8,9 | 旋钮 | 全局吸盘 | — | — | >1300=开 |
| 10 | 旋钮 | 齿条0~12 | — | — | — |
| 11 | 旋钮 | >1000左臂 | — | — | — |

## 5. 安全机制

- SBUS帧校验: buf[0]==0x0F && buf[24]==0x00
- 100ms断联 → 自动锁车(不重置last_ch4)
- failsafe标志 → 立刻锁车
- 臂关节限幅(arm_task内)
- CAN邮箱保护: arm_enable在sys_enabled=true之前完成

## 6. 变更记录

### 2026-06-19 — 模块化重构
- 拆分 lift.c/h, arm.c/h, grab.c/h, 各带状态机
- main.c 精简为锁车+SBUS调度+任务创建
- SBUS改回DMA循环+sbus_poll (同底盘方案)
- failsafe处理: 立刻锁车
- 断联保护: 不重置last_ch4, 避免重连误触发
- grab_reset改为边沿触发, 避免每帧重复

### 2026-06-15-18 — 功能开发
- 抬升控制: 4×DM4340, CH5/CH6
- 双机械臂: CAN2, CH7切换抬起/放下, CH11选臂
- 抓取机构: CAN1, 6步工序, CH10齿条, YV3
- 吸盘: CH8/YV1, CH9/YV2
- dm_motor移植自下层底盘

### 2026-06-15 — 底盘解耦
- 移除chassis/arm/grabbing旧代码
- 遥控器阈值修正 1700→1300
