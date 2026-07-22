# Robocon 2026 — 全国大学生机器人大赛

🏆 **全国一等奖** — 2026 赛季

## 硬件开源

自制 A 板 (STM32F407VET6) 原理图 & PCB：

[![Open Source Hardware](https://img.shields.io/badge/开源硬件-LCEDA-FF6B6B)](https://oshwhub.com/alexia0/project_krimgbaw)

## 结构

```
RC2026/
├── R1/                  # 第一台验证车
│   ├── lowerpart/       # 底盘主控 (Robomaster A板, STM32F427IIHx)
│   └── upperpart/       # 上层主控 (自制A板, STM32F407VET6)
└── R2/                  # 第二台车 (自制A板, STM32F407VET6)
```

## 算法仓库

视觉、决策、上位机等算法代码在独立的仓库中：

[https://github.com/inkccc/Rc2026](https://github.com/inkccc/Rc2026)

## 各子项目

| 项目 | 板型 | MCU | 功能 |
|------|------|-----|------|
| R1 底盘 | Robomaster A 板 | STM32F427IIHx | 四全向轮底盘 + 扩张机构 |
| R1 上层 | 自制 A 板 | STM32F407VET6 | 抬升 + 双机械臂 + 抓取 + 吸盘 + IR |
| R2 | 自制 A 板 | STM32F407VET6 | 全向底盘 + 抬升 + 双6轴臂 + 末端 |

## 构建

各子项目独立编译，详见各自的 `README.md`。

## 许可

MIT License
