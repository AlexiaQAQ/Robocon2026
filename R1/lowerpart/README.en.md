# Robocon 2026 — R1 Chassis Controller Firmware

[![MCU](https://img.shields.io/badge/MCU-STM32F407VET6-03234B?logo=stmicroelectronics)](https://www.st.com/en/microcontrollers-microprocessors/stm32f407ve.html)
[![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-6DB33F)](https://www.freertos.org/)
[![IDE](https://img.shields.io/badge/IDE-Keil%20MDK-009F4D)](https://www.keil.com/)

> 📖 [中文文档](README.md)

## Overview

Firmware for the **R1 chassis car** — the first validation vehicle of Robocon 2026 season. Built on **STM32F407VET6** (Cortex-M4F, 168MHz) + **FreeRTOS**.

R1 served as the platform for **motor driver library development, SBUS remote control validation, and DM-series motor CAN protocol adaptation**. Its `motor_control` module has become the foundation library for subsequent vehicles including R2.

> **2026-06-06** Motor control refactor: introduced `dm_model_t` multi-model motor support, replacing global hardcoded parameters.

---

## Hardware

### MCU
- **Chip**: STM32F407VET6 (Cortex-M4F, 168MHz, 512KB Flash, 192KB SRAM)

### Peripheral Pin Map

| Peripheral | Pins | Purpose |
|------------|------|---------|
| CAN1 | PD0/PD1 | Chassis motor CAN bus |
| CAN2 | PB5/PB6 | Secondary CAN bus |
| USART1 | PB6(TX)/PB7(RX) | SBUS receiver (100kbps, 9-bit, Even, 2-stop) |
| UART7 | PE8(TX)/PE7(RX) | Reserved (115200-8N1) |
| UART8 | PE1(TX)/PE0(RX) | Reserved (115200-8N1) |
| USART6 | PG14(TX)/PG9(RX) | Reserved: gimbal/host comm (115200-8N1, DMA RX+TX) |
| SPI5 | — | SPI expansion |
| TIM4/5 | — | General-purpose timers |
| TIM14 | — | FreeRTOS timebase |
| GPIOG[1:8] | PG1~PG8 | LED chaser (push-pull outputs) |
| GPIOH[2:5] | PH2~PH5 | XT1~XT4 outputs (push-pull, pulldown) |
| GPIOF[6] | PF6 | Output |
| GPIOD[12] | PD12 | Input (pulldown) |

---

## Software Architecture

```
R1/lowerpart/
├── Core/Src/               # CubeMX generated + main logic
│   ├── main.c              # Task creation + SBUS parsing
│   ├── freertos.c          # FreeRTOS config
│   └── can.c / dma.c / gpio.c / spi.c / tim.c / usart.c
├── Lib/                    # ★ Application libraries (high quality, reusable)
│   ├── motor_control.c/h   # DM/YUN motor driver (7 models, 4 modes)
│   ├── bsp_can.c/h         # CAN filter initialization
│   ├── CAN_receive.c/h     # CAN data reception (3508 current mode)
│   ├── sbus_set.c/h        # SBUS decoder (USART1 DMA)
│   └── pid.c/h             # PID controller
├── Drivers/                # STM32 HAL + CMSIS
├── Middlewares/            # FreeRTOS source
└── MDK-ARM/                # Keil project (master_a.uvprojx)
```

### FreeRTOS Tasks

| Task | Period | Stack | Priority | Function |
|------|--------|-------|----------|----------|
| `start_task` | once | 256 | 0 | Init peripherals, create child tasks, self-delete |
| `remote_task` | ~4ms | 128 | 0 | SBUS decode + CH4 enable edge detection + speed mapping |
| `chassis_task` | ~2ms | 128 | 0 | ⚠️ Empty skeleton — kinematics not yet implemented |
| `led_task` | 200ms | 128 | 0 | GPIOG[1:8] chaser LEDs |

---

## SBUS Remote Control

| Parameter | Value |
|-----------|-------|
| Interface | USART1 (PB6/PB7) |
| Baud | 100000 |
| Data/Stop/Parity | 9-bit / 2 / Even |
| Frame length | 25 bytes |
| DMA | DMA2_Stream2, circular |

Channel mapping when enabled (CH4 > 1600):
- CH0 → vw (angular velocity)
- CH2 → vx (forward velocity)
- CH3 → vy (lateral velocity)

Stick deadzone: values 991~993 → 0. 50ms frame timeout triggers automatic disable.

---

## Motor Control Library

R1's `motor_control` module is the season's foundation library. Key design decisions:

- **Per-motor ranges**: each `motor_t` carries its own `p_max/v_max/t_max` — different models get different ranges
- **Auto CAN ID offset**: `motor->mode` automatically computes `+MIT_MODE` / `+POS_MODE` offsets
- **Unified feedback callback**: single `dm_rx_cbk(motor_set, rx_data)` dispatches by ID, no repeated switch-case

Supported DM models: DM_4310, DM_4310_48V, DM_4340, DM_3519, DM_8006, DM_8009, DM_CUSTOM.

Control modes: MIT, Position-Velocity, Speed, Position-Speed-Current (PSI).

---

## Status

| Subsystem | Status | Notes |
|-----------|--------|-------|
| Motor control library | ✅ Complete | 7 DM models + YUN, 4 modes |
| SBUS remote | ✅ Complete | DMA circular, frame check, timeout |
| CAN filters | ✅ Complete | hcan1 + hcan2 |
| Chassis kinematics | ❌ TODO | `chassis_task` is empty |
| Motor enable flow | ❌ TODO | No `dm_enable()` in start_task |
| Gimbal/weapon | ⚠️ Commented out | USART6 code exists in main.c |

---

## Related

| Resource | Link |
|----------|------|
| Repository | [Gitee — alexiaqaq/robocon2026](https://gitee.com/alexiaqaq/robocon2026) |
| R2 Controller | [../R2/](../R2/) |
| Short memory box | [memory_box_short.md](memory_box_short.md) |
| Long memory box | [memory_box_long.md](memory_box_long.md) |
