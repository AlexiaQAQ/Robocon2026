/**
 * @file    arm.h
 * @brief   左右双机械臂控制模块（3轴 DM4340, CAN2 位置模式）
 * @author  Alexia
 * @date    2026-06-08
 *
 * 机械臂布局 (俯视图):
 *   [3]                              [6]   ← 末端吸盘
 *   [2]                              [5]
 *   [1]【轮】车头【轮】 [4]                 ← 根部
 *       【轮】车尾【轮】
 *
 * 左臂: CAN2 ID 1(根) 2(中) 3(末)
 * 右臂: CAN2 ID 4(根) 5(中) 6(末) (待实现)
 *
 * 控制方式: 串口 → left_pitch1~3 (mrad) → pos_ctrl (rad) → DM4340
 * 无需 IK 解算，上位机直接下发各关节目标角度
 */

#ifndef _ARM_H_
#define _ARM_H_

#include "main.h"
#include "can.h"
#include "motor_control.h"

/* ==================== CAN2 电机 ID 映射 ==================== */

/* 左臂 (root→tip) */
#define ARM_LEFT_ID1   1   // 根部关节: 前后摆动
#define ARM_LEFT_ID2   2   // 中部关节
#define ARM_LEFT_ID3   3   // 末端关节 + 吸盘

/* 右臂 (root→tip, 待实现) */
#define ARM_RIGHT_ID1  4
#define ARM_RIGHT_ID2  5
#define ARM_RIGHT_ID3  6

/* ==================== 位置模式速度 ==================== */

#define ARM_VEL_SLOW  0.3f    // rad/s, normal 模式
#define ARM_VEL_FAST  0.5f    // rad/s, fast 模式

/* ==================== 原点位置 (机械零位, rad) ==================== */

/* 左臂原点 */
#define ARM_L_ORIGIN_P1   0.0f    // 根部
#define ARM_L_ORIGIN_P2  -2.99f   // 中部
#define ARM_L_ORIGIN_P3  -1.44f   // 末端

/* 右臂原点 (与左臂镜像对称) */
#define ARM_R_ORIGIN_P1   0.0f
#define ARM_R_ORIGIN_P2   2.99f
#define ARM_R_ORIGIN_P3   1.44f

/* 兼容旧宏 (逐步废弃) */
#define ARM_ORIGIN_P1  ARM_L_ORIGIN_P1
#define ARM_ORIGIN_P2  ARM_L_ORIGIN_P2
#define ARM_ORIGIN_P3  ARM_L_ORIGIN_P3

/* ==================== 全局变量 ==================== */

/* 左臂电机反馈数据 (CAN2 RX 回调填充) */
extern motor_t arm_left_motor[3];

/* 右臂电机反馈数据 (CAN2 RX 回调填充) */
extern motor_t arm_right_motor[3];

/* ==================== API ==================== */

/**
 * @brief 双机械臂初始化 — dm_init 注册电机型号
 *        左臂: DM_4340 POS 模式, ID 1/2/3
 *        右臂: DM_4340 POS 模式, ID 4/5/6
 */
void arm_init(void);

/**
 * @brief 左臂使能 (CAN2: dm_enable ×3, 每个间隔 5ms)
 */
void arm_left_enable(CAN_HandleTypeDef *hcan);

/**
 * @brief 左臂失能 (CAN2: dm_disable ×3)
 */
void arm_left_disable(CAN_HandleTypeDef *hcan);

/**
 * @brief 左臂直接位置控制 (手动模式/原点归位)
 * @param hcan  CAN2 handle
 * @param p1/p2/p3  目标位置 (rad)
 * @param vel       位置模式速度 (rad/s)
 */
void arm_left_set(CAN_HandleTypeDef *hcan, float p1, float p2, float p3, float vel);

/**
 * @brief 左臂位置更新 (自动模式用)
 * @param hcan  CAN2 handle
 * @param vel   位置模式速度 (rad/s)
 *
 * 从 uart_task 全局变量 left_pitch1/2/3 (mrad) 读取目标角度
 * 转换为 rad 后通过 pos_ctrl 发往 CAN2
 */
void arm_left_update(CAN_HandleTypeDef *hcan, float vel);

/* ==================== 右臂 API (镜像左臂) ==================== */

void arm_right_enable(CAN_HandleTypeDef *hcan);
void arm_right_disable(CAN_HandleTypeDef *hcan);
void arm_right_set(CAN_HandleTypeDef *hcan, float p1, float p2, float p3, float vel);

/**
 * @brief 右臂位置更新 (自动模式用)
 * 从 uart_task 全局变量 right_pitch1/2/3 (mrad) → rad → CAN2
 */
void arm_right_update(CAN_HandleTypeDef *hcan, float vel);

/**
 * @brief CAN2 RX 回调: 解析 DM4340 电机反馈
 * @param rx_data  8 字节 CAN 数据
 *
 * 由 CAN_receive.c → HAL_CAN_RxFifo1MsgPendingCallback 调用
 */
void arm_can2_rx_callback(uint8_t *rx_data);

#endif
