# R2 主控固件 v1.0 Release Notes

**日期**: 2026-06-28  
**MCU**: STM32F407VET6 (Cortex-M4F, 168MHz)  
**RTOS**: FreeRTOS  
**提交**: `6f3b377`

---

## 已实现功能

### 底盘

- 四全向轮 DM_3519 MIT 模式驱动 (CAN1, ID 1~4)
- 45° 全向轮逆运动学 (COS45 补偿, 实际车速 = 协议值)
- 速度限幅 vx/vy ±2m/s, vw ±3.2rad/s (双重)
- MIT 阻尼 kd=7.0
- 发送顺序 BR→FR→BL→FL

### 抬升

- 四独立抬升 DM_4310 POS 模式 (CAN1, ID 5~8)
- 齿条满行程 420mm
- 编码器实时回传, 软件解卷绕 (帧间追踪, 首帧 target 锚定)
- 零位偏移校准 (LIFT_OFFSET_FR/FL/BL/BR)
- 各电机独立偏移补偿

### 双机械臂

- 左右各 3 轴 DM_4340 POS 模式 (CAN2, ID 1~6)
- 各关节独立速度 (12 个宏, 分别在 arm.h 中调校)
- 自动模式: 接收上位机 mrad 指令, 方向已适配
- 手动模式: 双机械臂回原点
- 硬限幅 ±180° (±3141 mrad)

### 末端执行器

- 夹爪翻转 DM_4310 POS (MCP2515 hcan3, ID 0x01)
- 夹爪气缸 YV1 (开/关)
- 左吸盘 YV2 (吸/松)
- 右吸盘 YV3 (吸/松)

### 遥控器 (SBUS)

| CH | 功能 |
|----|------|
| CH0 | vw 角速度 |
| CH2 | vx 前进 |
| CH3 | vy 横向 |
| CH4 | 主使能 (上升沿使能, 下降沿失能+安全释放) |
| CH5 | 手动/自动切换 |
| CH6 低 + CH7 | 翻转电机 0↔1.57 |
| CH6 中 + CH7 | 夹爪 YV1 开↔关 |
| CH6 高 + CH7 | 左右吸盘 YV2+YV3 全开/全关 |
| CH8 | 前后升降 0~420mm (CH6 高时) |

### 串口协议

- 控制帧 30B (USART1, 上位机→电控, 50Hz DMA)
- 状态帧 23B (USART2, 电控→上位机, 50Hz)
- 协议对齐 `rc26_vehicle_serial_protocol_final.md`
- 升降实际高度编码器实测回传 (非 target)
- 光电/TOF 占位 (独立 MCU 处理)

### 安全机制

1. SBUS 帧校验 + 100ms 超时断联
2. CH4 边沿检测使能/失能
3. **使能时序**: 先使能电机, 后置 sys_enabled, 防控帧先达
4. **失能安全释放**: YV1 夹紧 + YV2/YV3 松开, 先发后失能
5. 速度/升降/机械臂三重硬限幅
6. 夹爪上电保护 (gripper_flip_ready)
7. DMA 帧头尾校验失败自动复位

---

## 关键技术修正

| 修正 | 说明 |
|------|------|
| 运动学 COS45 补偿 | vx/(r×cos45°) 替代 cos45×vx/r, 车速 ×2 |
| 升降解卷绕 | POS 电机 ±12.5rad 单圈, 软件帧间追踪解卷 |
| dm_enable start_flag | enable/disable 置位标志, 状态帧据此判断有效性 |
| dm_motor 命令 ID | 统一使用基础 ID, MCP2515 与 native CAN 一致 |

---

## FreeRTOS 任务

| 任务 | 周期 | 栈 (words) | 优先级 |
|------|------|-----------|--------|
| led_task | 200ms | 128 | 2 |
| sbus_task | 10ms | 256 | 1 |
| uart_task | 20ms | 1024 | 0 |
| chassis_task | ~2ms | 512 | 0 |
| up_cs_task | 50ms | 512 | 0 |
| arm_task | 10ms | 512 | 0 |

---

## 构建产物

- Flash: 30348 bytes
- RAM: 24912 bytes (192KB 总 SRAM 的 13%)

---

## 已知限制

- 光电/TOF 传感器由独立 MCU 处理, 状态帧对应字段占位 0x00
- 状态帧仅自动模式 (CH5 低) 回传
- 右臂原点为镜像对称
- USART3 预留给串口陀螺仪, 当前未启用
- MCP2515 hcan3 仅用于夹爪翻转 + 电磁阀
