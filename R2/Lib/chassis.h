/**
 * @file    chassis.h
 * @brief   四45°全向轮底盘 + 独立抬升控制
 * @author  Alexia
 * @date    2026-06-10
 *
 * 全部电机走 CAN1 (原生 bxCAN)。使用 motor_control 新库 motor_t* API。
 *
 * dm_motor[] 分配:
 *   [0~3] 全向轮 (DM MIT),   ID 1~4
 *   [4~7] 独立抬升 (DM POS),  ID 5~8
 */

#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "main.h"
#include "can.h"
#include "motor_control.h"

/* ---- dm_motor[] 索引 ---- */
#define CHASSIS_MOTOR_BR  0   // 全向轮后右, CAN ID 1
#define CHASSIS_MOTOR_FR  1   // 全向轮前右, CAN ID 2
#define CHASSIS_MOTOR_FL  2   // 全向轮前左, CAN ID 3
#define CHASSIS_MOTOR_BL  3   // 全向轮后左, CAN ID 4

#define LIFT_MOTOR_FR  4      // 抬升前右, CAN ID 5
#define LIFT_MOTOR_FL  5      // 抬升前左, CAN ID 6
#define LIFT_MOTOR_BL  6      // 抬升后左, CAN ID 7
#define LIFT_MOTOR_BR  7      // 抬升后右, CAN ID 8

/* 抬升方向符号: +1 = 正转向上收, -1 = 正转向下撑 (全为向下撑) */
#define LIFT_DIR_FR   (-1.0f)
#define LIFT_DIR_FL   (+1.0f)
#define LIFT_DIR_BL   (-1.0f)
#define LIFT_DIR_BR   (+1.0f)

/* ---- 底盘物理参数 ---- */
#define CHASSIS_R        0.245f            // 轮心到几何中心距离 (m)
#define WHEEL_RADIUS     0.064f            // 全向轮半径 (m), 直径 127mm
#define CHASSIS_TORQUE   3.0f              // MIT 模式阻尼系数
#define COS45            0.70710678118f
#define SPEED_SCALE      0.00001f           // 速度缩放

/* 齿条: 电机弧度 → 线性行程 (mm) */
#define RACK_MM_PER_RAD  20.0f             // 417mm / 20.85rad

/* 抬升位置模式速度 (rad/s) */
#define LIFT_VEL_SLOW    2.5f
#define LIFT_VEL_FAST    20.0f

/* ---- 外部变量 ---- */
extern float set_vx, set_vy, set_vw;
extern float lift_target_mm[4];

/* ---- API ---- */

void chassis_init(CAN_HandleTypeDef *hcan);
void chassis_enable(CAN_HandleTypeDef *hcan);
void chassis_disable(CAN_HandleTypeDef *hcan);
void chassis_update(CAN_HandleTypeDef *hcan);
void lift_update(CAN_HandleTypeDef *hcan, float vel);
void lift_set(uint8_t idx, float mm);

#endif
