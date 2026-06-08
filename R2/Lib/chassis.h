#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "main.h"
#include "can.h"
#include "motor_control.h"
#include "mcp2515.h"

/* ==================== 底盘几何参数 ==================== */
#define CHASSIS_R        0.245f    // 轮心到几何中心距离 (m), sqrt((0.347/2)^2*2)
#define WHEEL_RADIUS     0.64f      // 轮子半径(单位 米)
#define CHASSIS_TORQUE   3.0f      // DM MIT 阻尼系数

/* ==================== CAN1 电机 ID 映射 ==================== */
/*
 * 俯视图 (车头朝上 = X轴):
 *
 *   全向轮 (45° 安装):         抬升机构 (DM4310 齿条):
 *   [FL=3]  车头  [FR=2]       [FL=6]  车头  [FR=5]
 *   [BL=4]  车尾  [BR=1]       [BL=7]  车尾  [BR=8]
 *
 * 全向轮运动学: 45° 模型, 系数 0.707 = cos(45°) = sin(45°)
 * 抬升方向: FR/BL 正=向上收, FL/BR 正=向下撑
 */

/* 全向轮驱动电机 (DM MIT 模式, 45° 安装) */
#define WHEEL_BR  1   // 后右
#define WHEEL_FR  2   // 前右
#define WHEEL_FL  3   // 前左
#define WHEEL_BL  4   // 后左

/* 独立抬升电机 (DM 位置-速度模式) */
#define LIFT_FR   5   // 前右, 正=向上收
#define LIFT_FL   6   // 前左, 正=向下撑
#define LIFT_BL   7   // 后左, 正=向上收
#define LIFT_BR   8   // 后右, 正=向下撑

/* 抬升方向符号: +1 = 正转向上收, -1 = 正转向下撑 */
#define LIFT_DIR_FR   (-1.0f)   // 正=向下撑
#define LIFT_DIR_FL   (+1.0f)   // 正=向下撑
#define LIFT_DIR_BL   (-1.0f)   // 正=向下撑
#define LIFT_DIR_BR   (+1.0f)   // 正=向下撑

/* 抬升位置模式速度 (rad/s) — 协议 lift_mode 控制 */
#define LIFT_VEL_SLOW  2.5f   // normal 模式: 精确调高
#define LIFT_VEL_FAST  20.0f   // fast  模式: 快速伸出/收回

/* ==================== 全局变量 ==================== */

extern float set_vx;   // 前进速度
extern float set_vy;   // 横向速度
extern float set_vw;   // 角速度

/* 抬升目标高度 (mm), 4个独立控制 */
extern float lift_target_mm[4];

/* ==================== API ==================== */

void chassis_init(CAN_HandleTypeDef *hcan);
void chassis_enable(CAN_HandleTypeDef *hcan);
void chassis_disable(CAN_HandleTypeDef *hcan);

/**
 * @brief 45° 全向轮运动学更新 + CAN1 MIT 指令发送 (~2ms 周期)
 */
void chassis_update(CAN_HandleTypeDef *hcan);

/**
 * @brief 抬升电机位置控制更新 (~50ms 周期)
 */
void lift_update(CAN_HandleTypeDef *hcan, float vel);

/**
 * @brief 设置单个抬升电机目标高度
 * @param idx 0=FR, 1=FL, 2=BL, 3=BR
 * @param mm 目标高度 (mm)
 */
void lift_set(uint8_t idx, float mm);

#endif
