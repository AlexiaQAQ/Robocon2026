/**
 * @file    arm.c
 * @brief   左右双机械臂控制实现 (移植到 motor_control 新库)
 * @author  Alexia
 * @date    2026-06-10
 */

#include "arm.h"
#include "uart_task.h"
#include "cmsis_os.h"

/* 反馈数据 */
motor_t arm_left_motor[3];
motor_t arm_right_motor[3];

/* ================================================================
   初始化 — 注册电机型号 DM_4340 POS 模式
   ================================================================ */

void arm_init(void)
{
    dm_init(&arm_left_motor[0],  ARM_LEFT_ID1, DM_MODE_POS, DM_4340);
    dm_init(&arm_left_motor[1],  ARM_LEFT_ID2, DM_MODE_POS, DM_4340);
    dm_init(&arm_left_motor[2],  ARM_LEFT_ID3, DM_MODE_POS, DM_4340);

    dm_init(&arm_right_motor[0], ARM_RIGHT_ID1, DM_MODE_POS, DM_4340);
    dm_init(&arm_right_motor[1], ARM_RIGHT_ID2, DM_MODE_POS, DM_4340);
    dm_init(&arm_right_motor[2], ARM_RIGHT_ID3, DM_MODE_POS, DM_4340);
}

/* ================================================================
   左臂
   ================================================================ */

void arm_left_enable(CAN_HandleTypeDef *hcan)
{
    dm_enable(hcan, &arm_left_motor[0]);  vTaskDelay(5);
    dm_enable(hcan, &arm_left_motor[1]);  vTaskDelay(5);
    dm_enable(hcan, &arm_left_motor[2]);  vTaskDelay(5);
}

void arm_left_disable(CAN_HandleTypeDef *hcan)
{
    dm_disable(hcan, &arm_left_motor[0]);  vTaskDelay(5);
    dm_disable(hcan, &arm_left_motor[1]);  vTaskDelay(5);
    dm_disable(hcan, &arm_left_motor[2]);  vTaskDelay(5);
}

void arm_left_set(CAN_HandleTypeDef *hcan, float p1, float p2, float p3, float vel)
{
    dm_pos_ctrl(hcan, ARM_LEFT_ID1, p1, vel);  vTaskDelay(1);
    dm_pos_ctrl(hcan, ARM_LEFT_ID2, p2, vel);  vTaskDelay(1);
    dm_pos_ctrl(hcan, ARM_LEFT_ID3, p3, vel);  vTaskDelay(1);
}

void arm_left_update(CAN_HandleTypeDef *hcan, float vel)
{
    /* ID2 方向反转: 协议正=上, 电机正=下 */
    float p1 =  (float)left_pitch1 * 0.001f;
    float p2 = -(float)left_pitch2 * 0.001f;
    float p3 =  (float)left_pitch3 * 0.001f;

    dm_pos_ctrl(hcan, ARM_LEFT_ID1, p1, vel);  vTaskDelay(1);
    dm_pos_ctrl(hcan, ARM_LEFT_ID2, p2, vel);  vTaskDelay(1);
    dm_pos_ctrl(hcan, ARM_LEFT_ID3, p3, vel);  vTaskDelay(1);
}

/* ================================================================
   右臂
   ================================================================ */

void arm_right_enable(CAN_HandleTypeDef *hcan)
{
    dm_enable(hcan, &arm_right_motor[0]);  vTaskDelay(5);
    dm_enable(hcan, &arm_right_motor[1]);  vTaskDelay(5);
    dm_enable(hcan, &arm_right_motor[2]);  vTaskDelay(5);
}

void arm_right_disable(CAN_HandleTypeDef *hcan)
{
    dm_disable(hcan, &arm_right_motor[0]);  vTaskDelay(5);
    dm_disable(hcan, &arm_right_motor[1]);  vTaskDelay(5);
    dm_disable(hcan, &arm_right_motor[2]);  vTaskDelay(5);
}

void arm_right_set(CAN_HandleTypeDef *hcan, float p1, float p2, float p3, float vel)
{
    dm_pos_ctrl(hcan, ARM_RIGHT_ID1, p1, vel);  vTaskDelay(1);
    dm_pos_ctrl(hcan, ARM_RIGHT_ID2, p2, vel);  vTaskDelay(1);
    dm_pos_ctrl(hcan, ARM_RIGHT_ID3, p3, vel);  vTaskDelay(1);
}

void arm_right_update(CAN_HandleTypeDef *hcan, float vel)
{
    /* ID4/ID6 方向反转: 协议正=上, 电机正=下 */
    float p1 = -(float)right_pitch1 * 0.001f;
    float p2 =  (float)right_pitch2 * 0.001f;
    float p3 = -(float)right_pitch3 * 0.001f;

    dm_pos_ctrl(hcan, ARM_RIGHT_ID1, p1, vel);  vTaskDelay(1);
    dm_pos_ctrl(hcan, ARM_RIGHT_ID2, p2, vel);  vTaskDelay(1);
    dm_pos_ctrl(hcan, ARM_RIGHT_ID3, p3, vel);
}

/* ================================================================
   CAN2 RX 回调
   ================================================================ */

void arm_can2_rx_callback(uint8_t *rx_data)
{
    uint8_t id = rx_data[0] & 0x0F;

    switch (id)
    {
        case 1: dm_fb_parse(&arm_left_motor[0], rx_data);   break;
        case 2: dm_fb_parse(&arm_left_motor[1], rx_data);   break;
        case 3: dm_fb_parse(&arm_left_motor[2], rx_data);   break;
        case 4: dm_fb_parse(&arm_right_motor[0], rx_data);  break;
        case 5: dm_fb_parse(&arm_right_motor[1], rx_data);  break;
        case 6: dm_fb_parse(&arm_right_motor[2], rx_data);  break;
        default: break;
    }
}
