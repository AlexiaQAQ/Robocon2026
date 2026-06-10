/**
 * @file    chassis.h
 * @brief   四45°全向轮底盘运动学控制
 * @author  Alexia
 * @date    2026-06-10
 *
 * CAN1 ID 映射 (俯视图, 车头朝上):
 *   [FL=3]  车头  [FR=2]
 *   [BL=4]  车尾  [BR=1]
 *
 * 45° 逆运动学:
 *   V_BR = (-cos45·Vx - cos45·Vy + Vw·R) / r
 *   V_FR = (+cos45·Vx - cos45·Vy + Vw·R) / r
 *   V_FL = (+cos45·Vx + cos45·Vy + Vw·R) / r
 *   V_BL = (-cos45·Vx + cos45·Vy + Vw·R) / r
 */

#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "motor_control.h"

/* ---- 电机在 dm_motor[] 中的索引 (CAN ID 1~4) ---- */
#define CHASSIS_MOTOR_BR  0   // 后右, CAN ID 1
#define CHASSIS_MOTOR_FR  1   // 前右, CAN ID 2
#define CHASSIS_MOTOR_FL  2   // 前左, CAN ID 3
#define CHASSIS_MOTOR_BL  3   // 后左, CAN ID 4

/* ---- 底盘物理参数 ---- */
#define CHASSIS_R        0.245f            // 轮心到几何中心距离 (m)
#define WHEEL_RADIUS     0.064f            // 全向轮半径 (m), 直径 127mm
#define COS45            0.70710678118f    // cos(45°)
#define SPEED_SCALE      0.0001f           // 速度缩放 (配合 set_vx/set_vy/set_vw 的 ±1000 范围)
#define CHASSIS_TORQUE   2.0f              // MIT 模式阻尼系数

/* ---- 外部速度变量 (定义在 main.c) ---- */
extern float set_vx;
extern float set_vy;
extern float set_vw;

/* ---- API ---- */

void chassis_init(void);
void chassis_enable(void);
void chassis_disable(void);
void chassis_update(void);

#endif
