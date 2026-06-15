# Robocon 2026

National Robocon 2026 competition — main control firmware.

## Structure

```
RC2026/
├── R1/                  # First validation robot
│   ├── lowerpart/       # Chassis controller (Robomaster Board A, STM32F427)
│   └── upperpart/       # Upper controller (Custom Board A, STM32F407)
└── R2/                  # Second robot (Custom Board A, STM32F407)
```

| Project | Board | MCU | IDE |
|---------|-------|-----|-----|
| R1 Chassis | Robomaster Board A | STM32F427IIHx | Keil MDK |
| R1 Upper | Custom Board A | STM32F407VETx | Keil MDK |
| R2 | Custom Board A | STM32F407VETx | Keil MDK |

## Build

Each subproject builds independently. See their respective `README.md`.

## License

MIT License
