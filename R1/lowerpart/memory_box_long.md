# R1 底盘主控 — 长记忆匣

> 完整上下文记录：设计决策、代码演进、调试经验、已知问题和改进方向。

---

## 1. 项目历史

### 1.1 时间线

| 日期 | 事件 | Commit |
|------|------|--------|
| 2026-05 | R1 底盘车立项，CubeMX 初始化工程 | `fd06914` |
| 2026-05 | SBUS + CAN + 电机驱动第一版联调通过 | `7955ff0` — v0.1 底盘验证版 |
| 2026-05 | 新增左右双机械臂控制 + 末端吸盘电磁阀 | `76d4ded` — 但 R1 本身没有机械臂硬件 |
| 2026-06-06 | motor_control 重构: dm_model_t 多型号表 | 当前代码状态 |

### 1.2 R1 的定位

R1 是赛季**第一台能跑的车**，核心目标是：
1. **验证 DM 电机 CAN 协议** — DM4310/DM4340/YUN 系列的 MIT/位置/速度/PSI 模式
2. **验证 SBUS 遥控链路** — USART1 DMA 循环接收、帧校验、超时断联
3. **输出可复用的电机驱动库** — `motor_control.c/h` 作为后续 R2 等车型的依赖

R1 **不是**竞赛车 — 它没有机械臂、没有上位机通信、没有传感器融合，运动学也还没写。

---

## 2. 硬件详细配置

### 2.1 芯片

- **型号**: STM32F407VET6
- **封装**: LQFP-100
- **内核**: Cortex-M4F (单精度 FPU)
- **频率**: HSE 8MHz → PLL (M=6, N=168, P=2) → SYSCLK 168MHz
  - AHB: 168MHz (÷1)
  - APB1: 42MHz (÷4)
  - APB2: 84MHz (÷2)
- **Flash**: 512KB
- **SRAM**: 192KB (128KB + 64KB CCM)

### 2.2 完整引脚映射

#### CAN (2路)

| 外设 | RX | TX | 用途 |
|------|-----|-----|------|
| CAN1 | PD0 | PD1 | 主 CAN 总线 |
| CAN2 | PB5 | PB6 | 第二路 CAN |

#### UART (4路)

| 外设 | RX | TX | 波特率 | 配置 | 用途 |
|------|-----|-----|--------|------|------|
| USART1 | PB7 | PB6 | 100000 | 9-bit, Even, 2-stop | SBUS 遥控器 (DMA2_Stream2 RX) |
| UART7 | PE7 | PE8 | 115200 | 8N1 | 预留 |
| UART8 | PE0 | PE1 | 115200 | 8N1 | 预留 |
| USART6 | PG9 | PG14 | 115200 | 8N1 | 预留: 云台/上位机 (DMA2_Stream1 RX, Stream6 TX) |

#### SPI

| 外设 | 用途 |
|------|------|
| SPI5 | 预留扩展 |

#### 定时器

| 外设 | 用途 |
|------|------|
| TIM4 | 通用 (PWM/编码器 待分配) |
| TIM5 | 通用 (PWM/编码器 待分配) |
| TIM14 | FreeRTOS 时基 (HAL_IncTick) |

#### GPIO

| 引脚 | 模式 | 用途 |
|------|------|------|
| PG1~PG8 | 推挽输出 | LED 流水灯 (低电平点亮) |
| PH2~PH5 | 推挽输出, 下拉 | XT1~XT4 (用途待确认) |
| PF6 | 推挽输出 | 输出 (用途待确认) |
| PD12 | 输入, 下拉 | 输入 (用途待确认) |

---

## 3. 软件架构详解

### 3.1 启动流程

```
上电复位
  └─ main()
       ├─ HAL_Init()                    # SysTick 初始化 (但会被 FreeRTOS 覆盖)
       ├─ SystemClock_Config()          # HSE 8MHz → PLL → 168MHz
       ├─ MX_GPIO_Init()                # GPIO 初始化 (PG, PH, PF, PD)
       ├─ MX_DMA_Init()                 # DMA 初始化
       ├─ MX_USART1_UART_Init()         # SBUS 接口 (PB6/PB7)
       ├─ MX_CAN1_Init()                # CAN1 (PD0/PD1)
       ├─ MX_CAN2_Init()                # CAN2 (PB5/PB6)
       ├─ MX_UART7_Init()               # 预留 (PE7/PE8)
       ├─ MX_UART8_Init()               # 预留 (PE0/PE1)
       ├─ MX_USART6_UART_Init()         # 预留 (PG9/PG14, DMA)
       ├─ MX_TIM5_Init()                # 定时器5
       ├─ MX_TIM4_Init()                # 定时器4
       ├─ MX_SPI5_Init()                # SPI5
       ├─ xTaskCreate(start_task, 256)  # ★ 在调度器启动前创建初始化任务
       ├─ MX_FREERTOS_Init()            # 创建 defaultTask (空闲)
       └─ osKernelStart()              # ★ 启动调度器, 此后 main() 不再执行

start_task():
  ├─ xt1~xt4(0)                        # XT 输出全部拉低
  ├─ can_filter_init()                  # CAN1: Bank0 FIFO0 全通, CAN2: Bank14 FIFO1 全通
  ├─ sbus_rx_init()                     # HAL_UARTEx_ReceiveToIdle_DMA(&huart1, 25B)
  ├─ xTaskCreate(led_task, 128)
  ├─ xTaskCreate(remote_task, 128)
  └─ vTaskDelete(NULL)                  # ★ 任务自销毁, 释放栈空间
```

**注意**: `xTaskCreate(start_task)` 在 `MX_FREERTOS_Init()` 和 `osKernelStart()` **之前**调用。这是 FreeRTOS 标准用法 — 调度器启动前可以创建任务，它们会在 `osKernelStart()` 后统一开始调度。但 `start_task` 内部 `vTaskDelete(NULL)` 是合法的，因为调度器已经开始运行。

### 3.2 中断优先级

| 中断 | 优先级 (NVIC) | 说明 |
|------|--------------|------|
| USART1 (SBUS) | 5, 0 | 抢占优先级 5, 子优先级 0 |
| USART6 | 5, 0 | 同上 |
| CAN1 | 默认 (CubeMX) | RX FIFO0 中断 |
| CAN2 | 默认 (CubeMX) | RX FIFO1 中断 |
| TIM14 | 默认 | FreeRTOS systick |
| SVCall | 默认 | FreeRTOS 内核 |
| PendSV | 最低 | FreeRTOS 上下文切换 |

> FreeRTOS 使用 CMSIS-OS v1 封装, 所有任务优先级 = `osPriorityNormal` (0), 时间片轮询。

---

## 4. motor_control 模块深入

### 4.1 为什么 R1 的 motor_control 是赛季基础库

R1 的 `motor_control.c/h` 经过了设计打磨，与 R2 版本有本质差异：

**R1 版 (推荐参考)**:
```c
// 1. 型号枚举 + 参数表 → 每个电机可有不同范围
dm_init(&motor[0], 1, DM_MODE_MIT, DM_4310);    // T_MAX=10
dm_init(&motor[1], 2, DM_MODE_MIT, DM_4340);     // T_MAX=27

// 2. mode 字段自动计算 CAN ID 偏移
HAL_StatusTypeDef dm_enable(CAN_HandleTypeDef *hcan, motor_t *motor) {
    uint16_t id = motor->id + mode_offset[motor->mode];
    return dm_can_send(hcan, id, data, 8);
}

// 3. 统一反馈解析 + 自动 ID 分发
void dm_rx_cbk(motor_t *motor_set, uint8_t *rx_data) {
    uint8_t id = rx_data[0] & 0x0F;
    if (id >= 1 && id <= 8)
        dm_fb_parse(&motor_set[id - 1], rx_data);
}
```

**R2 版 (简化版, 有提升空间)**:
```c
// 1. 全局硬编码, 所有电机同一范围
#define P_MIN -12.5f  // 全局
dm_enable(hcan, motor_id);  // 手动传 ID

// 2. 5个独立的 canX_rx_callback, switch-case 重复
void can1_rx_callback(uint8_t *rx_data) {
    switch(rx_data[0]) {
        case 0x01: dm4310_fbdata(&dm_motor[0], rx_data); break;
        // ... 重复 ×5 路 CAN
    }
}

// 3. MCP2515 双重接口, 每个 API 两个变体
dm_enable() / dm_enable_mcp2515()
dm_mit_ctrl() / dm_mit_ctrl_mcp2515()
// ...
```

### 4.2 DM 协议细节

#### MIT 模式 CAN 帧格式 (8 bytes)
```
Byte 0-1: p_des (位置期望, uint16, map to [-p_max, +p_max])
Byte 2-3: v_des (速度期望, uint12, map to [-v_max, +v_max])
Byte 3-4: kp_des (刚度, uint12, map to [0, 500])
Byte 5-6: kd_des (阻尼, uint12, map to [0, 5])
Byte 6-7: t_ff  (前馈扭矩, uint12, map to [-t_max, +t_max])
```

#### 位置-速度模式 (8 bytes, 直接 float)
```
Byte 0-3: pos (float, 小端)
Byte 4-7: vel (float, 小端)
```

#### 反馈帧 (8 bytes, MIT 模式)
```
Byte 0:    [state:4][id:4]   — bit7-4=电机状态, bit3-0=电机ID
Byte 1-2:  position (uint16)
Byte 3-4:  velocity (uint12)
Byte 4-5:  torque   (uint12)
Byte 6:    Tmos (MOS管温度, °C)
Byte 7:    Tcoil (线圈温度, °C)
```

#### 命令字节
```
0xFC — 使能
0xFD — 失能
0xFE — 保存当前位��为零位
0xFB — 清除错误
```

#### YUN 系列协议差异
```c
// YUN 控制帧: ID = (motor_id & 0x000f) | 0x80
// 范围: P±40rad, V±40rad/s, T±40Nm, Kp[0,1023], Kd[0,51]
```

---

## 5. 代码中的注释块 (调试历史)

### 5.1 dm_rx_cbk 被注释 (CAN_receive.c:20, 33)

```c
// CAN1 中断回调中
// dm_rx_cbk(dm_motor, rx_data);  ← 被注释

// CAN2 中断回调中
// dm_rx_cbk(&dm_motor[4], rx_data);  ← 被注释
```

这很可能是调试时的临时注释 — 可能因为当时总线上没有 DM 电机，收到乱码后反馈解析会污染 `dm_motor[]` 数据。**如果要让底盘运动学闭环，必须先恢复这两行**。

### 5.2 USART6 云台控制代码 (main.c:320-369)

整个 `HAL_UART_RxCpltCallback` 被注释掉。其中包含:
- 云台 yaw/pitch 角度解�� (从 USART6 接收)
- 电磁阀 YV6/YV7/YV8 控制
- 电机角度回传 (dm_motor[0]/[1] 的 angle_pos)

这段代码是**从另一台车 (可能是云台车) 复制过来的**，R1 用不到云台功能所以注释了。

---

## 6. 调试与验证记录

### 6.1 已验证通过 (v0.1, commit `fd06914`)

| 功能 | 验证方法 | 结果 |
|------|----------|------|
| SBUS 接收 | 遥控器拨杆, 串口打印通道值 | 15 通道正确解码 |
| CH4 使能 | 拨动 CH4, 观察 sys_enabled 变化 | 上升沿使能/下降沿失能正常 |
| CAN 发送 | 逻辑分析仪抓 CAN 总线 | DM MIT/位置帧格式正确 |
| DM 电机旋转 | MIT 模式发送固定速度指令 | 电机正常旋转 |
| LED 流水灯 | 目视 | 8 个 LED 依次闪烁 |

### 6.2 未验证

- CAN 反馈解析 (dm_rx_cbk 被注释, 没有读取过 DM 电机反馈)
- 底盘运动学 (chassis_task 是空的)
- USART6 通信 (代码被注释)

---

## 7. 改进路线图

### 短期 (让车能动)
1. 恢复 `CAN_receive.c` 中的 `dm_rx_cbk` 调用
2. 在 `start_task` 中添加 `dm_enable` 使能电机
3. 实现 `chassis_task` 底盘运动学 (参考 R2 的 `chassis_update`)

### 中期 (功能完善)
4. 启用 USART6 作为上位机通信 (或新增串口)
5. 添加 IMU 姿态传感器
6. 实现位置闭环 (使用 DM 电机反馈)

### 长期 (为下赛季积累)
7. 保持 motor_control 模块的参考实现地位
8. 验证 dm_model_t 表中各型号参数准确性
9. 补充单元测试或硬件在环测试

---

## 8. R1 与 R2 的文件对应关系

| R1 文件 | R2 对应 | 变更说明 |
|---------|---------|----------|
| `Lib/motor_control.c/h` | `Lib/motor_control.c/h` | R2 简化了型号系统 + 新增 MCP2515 接口 |
| `Lib/sbus_set.c/h` | `Lib/sbus_set.c/h` | 基本一致 |
| `Lib/pid.c/h` | `Lib/pid.c/h` | 基本一致 |
| `Lib/bsp_can.c/h` | `Lib/bsp_can.c/h` | R2 增加了 Bank 号和 FIFO 配置 |
| `Lib/CAN_receive.c/h` | `Lib/CAN_receive.c/h` | R2 新增 MCP2515 发送 + 3508 接收保留 |
| (无) | `Lib/chassis.c/h` | R2 新增: 全向轮 + 抬升 |
| (无) | `Lib/arm.c/h` | R2 新增: 双机械臂 |
| (无) | `Lib/mcp2515.c/h` | R2 新增: SPI-CAN 驱动 |
| (无) | `Lib/uart_task.c/h`| R2 新增: 串口协议 |
| (无) | `Lib/imu.c/h` | R2 新增: IMU |
| (无) | `Lib/solenoid_valves.c/h` | R2 新增 (R1 有电磁阀代码但被注释) |
| `Core/Src/main.c` | `R2/Core/Src/main.c` | 完全不同 — R2 有更多任务和模式 |
