# Robocon 2026

全国大学生机器人大赛 Robocon 2026 赛季主控代码。

## 结构

```
RC2026/
├── R1/                  # 第一台验证车
│   ├── lowerpart/       # 底盘主控 (Robomaster A板, STM32F427)
│   └── upperpart/       # 上层主控 (自制A板, STM32F407)
└── R2/                  # 第二台车 (自制A板, STM32F407)
```

| 项目 | 板型 | MCU | IDE |
|------|------|-----|-----|
| R1 底盘 | Robomaster A 板 | STM32F427IIHx | Keil MDK |
| R1 上层 | 自制 A 板 | STM32F407VETx | Keil MDK |
| R2 | 自制 A 板 | STM32F407VETx | Keil MDK |

## 构建

各子项目独立编译，详见各自的 `README.md`。

## 许可

MIT License
