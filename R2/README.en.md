# Robocon 2026 — R2 Main Control Firmware (Board A)

[![Gitee](https://img.shields.io/badge/Gitee-robocon2026-C71D23?logo=gitee)](https://gitee.com/alexiaqaq/robocon2026)
[![MCU](https://img.shields.io/badge/MCU-STM32F407VET6-03234B?logo=stmicroelectronics)](https://www.st.com/en/microcontrollers-microprocessors/stm32f407ve.html)
[![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-6DB33F)](https://www.freertos.org/)
[![IDE](https://img.shields.io/badge/IDE-Keil%20MDK-009F4D)](https://www.keil.com/)

> 📖 [中文文档](README.md)

## Overview

Main control board firmware for the Robocon 2026 R2 robot, developed on **STM32F407VET6** (Cortex-M4F, 168MHz) with **FreeRTOS** real-time operating system.

The robot features a **four-omni-wheel chassis + lifting mechanism + dual arm (left & right)** structure. The main controller handles low-level motor and solenoid valve control, operable via SBUS remote (manual mode) or host PC serial commands (automatic mode).

> **2026-06-08 Gripper flip + cylinder serial port integration** | 2026-06-06 Major mechanical revision: four steering wheels → four omni wheels, lifting mechanism motor replacement, single arm → dual arm.

---

## Hardware Architecture

### MCU
- **MCU**: STM32F407VET6 (Cortex-M4F, 168MHz, 512KB Flash, 192KB SRAM)

### Peripheral Resources & Pin Mapping

| Peripheral | Pins | Function |
|------------|------|----------|
| CAN1 | PD0/PD1 | Chassis: 4× omni-wheel (DM MIT) + 4× independent lift (DM4310 pos-vel) |
| CAN2 | PB5/PB6 | Left arm motors (DM series, position mode) |
| SPI1 + PA4 (CS) | PA5/PA6/PA7 | MCP2515 → hcan3, gripper flip DM + solenoid YV1 |
| SPI2 + PB12 (CS) | PB13/PB14/PB15 | MCP2515 → hcan4, reserved/unassigned |
| SPI3 + PA15 (CS) | PB3/PB4/PB5 | MCP2515 → hcan5, lifting module DM motors (pos-vel mode) |
| UART4 | PA0(TX)/PA1(RX) | SBUS receiver (100kbps, 9-bit, Even, 2-stop) |
| USART1 | PA9(TX)/PA10(RX) | Host PC: chassis speed + valves + lift distance (115200-8N1) |
| USART2 | PD5(TX)/PD6(RX) | Host PC: arm position commands (115200-8N1) |
| USART3 | PD8(TX)/PD9(RX) | Sensor frame TX (blocking, 5Hz); IMU Euler angle RX (not started) |
| TIM1 | — | General-purpose timer |
| TIM14 | — | FreeRTOS system tick |
| GPIOE[0:7] | PE0~PE7 | LED chaser (open-drain output) |
| GPIOE[9] | PE9 | Front sensor (GPIO input, no pull-up/down) |
| GPIOE[11] | PE11 | Input (reserved) |
| GPIOE[13] | PE13 | Rear sensor (GPIO input, no pull-up/down) |
| KEY1 | — | Key input (pull-down) |

### Actuators

| Actuator | Qty | Bus | Motor Type / Mode | Notes |
|----------|-----|-----|-------------------|-------|
| Omni-wheel motors | 4 | CAN1 (ID 1~4) | DM MIT mode | Four omni-wheel drive (chassis_update) |
| Independent lift motors | 4 | CAN1 (ID 5~8) | DM4310 pos-vel mode | Rack lift, 4 independent heights (lift_update) |
| Lifting module motors | 2 | hcan5 (MCP2515) | ⚠️ TBD | Front/rear module extension/retraction (pending) |
| Left arm motors | 4 | CAN2 | DM series, position | Yaw/pitch/elbow/terminal (pending) |
| Right arm motors | 4 | ⚠️ TBD | DM series, position | Yaw/pitch/elbow/terminal (pending) |
| Gripper flip motor | 1 | hcan3 (MCP2515) | DM series, position | Flip: 0=upright, 1.57=folded, safe on boot |
| Gripper cylinder | 1 | hcan3 (MCP2515) | Solenoid YV1 | CAN ID 0x300, 1=open, 0=close |

---

## Software Architecture

### Directory Structure

```
R2/
├── Core/Src/               # CubeMX generated + main logic
│   ├── main.c              # Main: task creation + control logic
│   ├── freertos.c          # FreeRTOS config (statically allocated Idle task)
│   ├── can.c / dma.c / gpio.c / spi.c / tim.c / usart.c
│   ├── stm32f4xx_it.c      # ISRs (CAN1/2 RX callbacks)
│   └── stm32f4xx_hal_msp.c # HAL MSP init
├── Lib/                    # Application libraries
│   ├── sbus_set.c/h        # SBUS decoder (UART4 DMA)
│   ├── bsp_can.c/h         # CAN filter init
│   ├── mcp2515.c/h         # MCP2515 SPI-CAN driver
│   ├── mcp2515_consts.h    # MCP2515 register definitions
│   ├── CAN_receive.c/h     # CAN data receive
│   ├── motor_control.c/h   # DM/YUN motor CAN protocol (MIT/pos/vel)
│   ├── pid.c/h             # PID controller (positional/incremental)
│   ├── chassis.c/h         # [NEW] Omni-wheel kinematics + independent lift
│   ├── solenoid_valves.c/h # Solenoid YV1 bit ops (CAN ID 0x300)
│   ├── uart_task.c/h       # UART task (UART1/2 parse + USART3 sensor)
│   └── imu.c/h             # HLK-AS201 IMU Euler angle parser
├── Doc/                    # Protocol documentation
│   └── rc26_vehicle_serial_protocol_final.md
├── Drivers/                # STM32 HAL + CMSIS
├── Middlewares/            # FreeRTOS source
└── MDK-ARM/                # Keil project files
```

> **Deleted** (2026-06-06): `four_steering_wheel_ik.c/h` (4-wheel steering kinematics), `upstairs.c/h` (old lift), `arm.c/h` (old single arm)

### FreeRTOS Tasks

| Task | Period | Stack (words) | Priority | Function |
|------|--------|---------------|----------|----------|
| `start_task` | One-shot | 256 | 0 | Init peripherals & subsystems, spawn child tasks, self-delete |
| `sbus_task` | 10ms | 256 | 0 | SBUS decode + RC speed mapping + mode switch + CH4 enable edge + CH5 falling notify |
| `uart_task` | 100ms | 128 | 0 | Serial command parse (UART1 chassis/valve + UART2 arm) + USART3 sensor 5Hz |
| `chassis_task` | ~2ms | 1280 | 0 | ✅ Omni-wheel kinematics + CAN1 MIT drive (chassis_update) |
| `up_cs_task` | ~50ms | 256 | 0 | ✅ Independent lift position + gripper flip motor (lift_update) |
| `arm_task` | 10ms | 512 | 0 | ⚠️ Dual arm control (pending rewrite) |
| `led_task` | 200ms | 56 | 0 | GPIOE[0:7] LED chaser |

> ⚠️ `arm_task` is a leftover skeleton from the old single-arm code; needs rewrite for dual arms. `imu_task()` module is implemented but not created in `main.c`.

### Startup Sequence

```
main()
 ├─ HAL_Init()
 ├─ SystemClock_Config()         # HSE → PLL (168MHz)
 ├─ MX_GPIO / DMA / CAN1/2 / SPI1/2/3 / UART4 / USART1/2/3 / TIM1 _Init()
 ├─ xTaskCreate(start_task)      # Create init task
 ├─ MX_FREERTOS_Init()           # Create defaultTask (idle)
 └─ osKernelStart()              # Start scheduler

start_task():
 ├─ sbus_rx_init()               # Start UART4 DMA RX for SBUS
 ├─ can_filter_init()            # Configure CAN1/2 filters
 ├─ uart_rx_init()               # Start USART1/2 DMA RX
 ├─ YV_mcp2515_init()            # Init solenoid MCP2515
 ├─ xTaskCreate(×6)              # Spawn all child tasks
 └─ vTaskDelete(NULL)            # Self-delete
```

---

## Control Modes

The system switches modes via remote CH4/CH5/CH6 channels:

```
CH4 (Master Enable, threshold 1700)
├── OFF (<1700 or SBUS lost)
│   └── All subsystems disabled, velocities zeroed
│
└── ON  (>1700)
    ├── CH5=High, CH6=Low  →  Manual RC mode
    │   ├── CH0 → vw (angular, Map: 268~1783 → -300~300)
    │   ├── CH1 → lift chassis speed (Map: 270~1800 → -10~10)
    │   ├── CH2 → vx (forward, Map: 256~1800 → -180~180)
    │   ├── CH3 → vy (lateral, Map: 240~1780 → -180~180)
    │   ├── CH6 low(~240) + CH7 edge → Toggle flip motor (0↔1.57)
    │   ├── CH6 mid(~1024) + CH7 edge → Toggle gripper cylinder (open↔close)
    │   ├── CH8 → Front lift height (0~417mm)
    │   ├── CH9 → Rear lift height (0~417mm)
    └── CH5=Low  →  Auto mode (host PC control)
        ├── USART1 : Chassis vx/vy/vw + front/rear lift + lift_mode + dual arm pitch + suction + weapon_pitch + weapon_gripper
        ├── USART2 : Status frame (lift height/ToF/arm angles) 50Hz
        └── CH5 falling edge → Notify host PC of mode switch
```

> **Note**: `ch_high()` threshold = **1700** (intentionally lower than 1800 for remote switch compatibility).

### Mode Switch Notification

When CH5 transitions from high to low (entering auto mode), the main controller sends notification frames:
- USART1 TX: `{0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xEE}`
- USART2 TX: `{0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xEE}`
- USART3 TX: Current sensor frame (synchronous)

---

## Communication Protocols (⚠️ 2026-06-06 protocols changed; below are legacy docs pending update)

### UART4 — SBUS Remote

| Parameter | Value |
|-----------|-------|
| Baudrate | 100000 |
| Data bits | 9 |
| Stop bits | 2 |
| Parity | Even |
| Frame length | 25 bytes |
| DMA | DMA1_Stream2, circular mode |

- 15 channels (CH0~CH15), each 11-bit (0~2047)
- Frame check: `buf[0]==0x0F && buf[23]==0x00 && buf[24]==0x00`
- Check failure → auto-reset DMA, restart reception

### USART1 Frame (⚠️ Legacy 15B protocol; new 30B protocol: see rc26_vehicle_serial_protocol_final.md)

```
Header   vyH   vyL   vxH   vxL   vwH   vwL   Lift   Front  Rear   V1    V2    V4    V5    Tail
0xCC     [1]   [2]   [3]   [4]   [5]   [6]   [7]    [8]    [9]   [10]  [11]  [12]  [13]  0xEE
```

| Offset | Type | Description |
|--------|------|-------------|
| 0 | uint8 | Frame header 0xCC |
| 1-2 | int16 | vy lateral speed (clamped ±200, parsed ×0.1 → set_vy) |
| 3-4 | int16 | vx forward speed (clamped ±200, parsed ×0.1 → set_vx) |
| 5-6 | int16 | vw angular speed (clamped ±200, parsed ×0.1 → set_vw) |
| 7 | int8 | Lift chassis speed (clamped ±100, mapped to -5~5 m/s) |
| 8 | uint8 | Front module distance (0~210mm) |
| 9 | uint8 | Rear module distance (0~210mm) |
| 10 | uint8 | weapon_pitch (0=hold, 1=parallel-to-ground, 2=upright) ⚠️ migrated to new 30B protocol |
| 14 | uint8 | Frame tail 0xEE |

### USART2 Frame (⚠️ Legacy 10B protocol; new 27B status frame: see rc26_vehicle_serial_protocol_final.md)

```
Header   fbH   fbL   lrH   lrL   udH   udL   End    Grip   Tail
0xCC     [1]   [2]   [3]   [4]   [5]   [6]   [7]    [8]    0xEE
```

| Offset | Type | Description |
|--------|------|-------------|
| 0 | uint8 | Frame header 0xCC |
| 1-2 | int16 | fb forward/backward position (clamped: -800~800) |
| 3-4 | int16 | lr left/right position (clamped: -800~800) |
| 5-6 | int16 | ud up/down position (clamped: -800~800) |
| 7 | uint8 | weapon_gripper (0=hold, 1=open, 2=close) ⚠️ migrated to new 30B protocol |
| 9 | uint8 | Frame tail 0xEE |

### USART3 Sensor Frame (MCU → Host, 8 bytes, 5Hz)

```
Header  Rear   Front  0x00   0x00   0x00   0x00   Tail
0xCC    [1]    [2]    [3]    [4]    [5]    [6]    0xEE
```

- rear (PE13): 0x01=clear, 0x00=obstacle (PIN_SET→0x00, PIN_RESET→0x01)
- front (PE9): same logic as above
- Transmission: blocking `HAL_UART_Transmit`, once every 2 `uart_task` cycles (5Hz)

### IMU Data (HLK-AS201 → MCU, USART3 DMA)

- Protocol frame: `FA FB ... FC FD`, 14 bytes/frame
- Power-on config: send `FA FB 03 1F 04 23 FC FD` to subscribe Euler angles only
- Parsing: Roll/Pitch/Yaw raw int16 × 0.0054931640625° → degrees
- Yaw low-pass filter: `α = 0.9`
- `imu_task()`: 50ms period, auto-reset DMA on frame check failure; task not currently created
- Output variables: `imu_roll, imu_pitch, imu_yaw, imu_yaw_filtered, imu_ok`

---

## Safety Mechanisms

1. **SBUS frame check**: Validate byte 0 (0x0F) + byte 23 (0x00) + byte 24 (0x00); auto-reset DMA on failure
2. **SBUS timeout**: 100ms without valid frame → `sys_enabled=false` → all motors disabled + velocity zeroed
3. **CH4 edge detection**: Rising edge → enable subsystems; falling edge → disable + safe stop
4. **Range clamping**: All velocity/position inputs bounded by min/max limits
5. **DMA fault recovery**: On UART header/tail check failure, stop DMA, clear buffer, restart reception
6. **Arm safety zone** ⚠️ Pending redefinition (separate forbidden zones for left & right arms)
7. **Joint limits** ⚠️ Pending redefinition (separate limits for left & right arms)

## Known Issues

- `solenoid_valves.h` YV9/YV10 macros have an array index bug (flip operation uses `can_can_send_data[0]` instead of correct index); YV9/YV10 are currently unused — no functional impact
- USART3 shared bus: only sensor frame TX runs currently; enabling IMU DMA RX may drop IMU frames during TX
- `arm_task` is leftover single-arm skeleton; needs rewrite for dual arms (separate IK for each arm)
- Lift actual height feedback is fake data (`lift_front_actual`/`lift_back_actual` = target); needs encoder readout

---

## Related Links

| Resource | Link |
|----------|------|
| Repository | [Gitee — alexiaqaq/robocon2026](https://gitee.com/alexiaqaq/robocon2026) |
| Serial Protocol | [rc26_vehicle_serial_protocol_final.md](rc26_vehicle_serial_protocol_final.md) |
| Chinese Docs | [README.md](README.md) |

> ⚠️ This README is manually maintained; some outdated info may differ from actual code. See `memory_box_short.md` for the latest change summary, and `memory_box_long.md` for full context.
