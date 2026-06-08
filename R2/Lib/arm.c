/**
 * @file    arm.c
 * @brief   左右双机械臂控制实现
 * @author  Alexia
 * @date    2026-06-08
 */

#include "arm.h"
#include "uart_task.h"
#include "cmsis_os.h"

/* ==================== 反馈数据 ==================== */

motor_t arm_left_motor[3];   // 左臂电机 1/2/3 的反馈
motor_t arm_right_motor[3];  // 右臂电机 4/5/6 的反馈

/* ==================== 左臂 ==================== */

void arm_left_enable(CAN_HandleTypeDef *hcan)
{
    dm_enable(hcan, ARM_LEFT_ID1);   vTaskDelay(5);
    dm_enable(hcan, ARM_LEFT_ID2);   vTaskDelay(5);
    dm_enable(hcan, ARM_LEFT_ID3);   vTaskDelay(5);
}

void arm_left_disable(CAN_HandleTypeDef *hcan)
{
    dm_disable(hcan, ARM_LEFT_ID1);   vTaskDelay(5);
    dm_disable(hcan, ARM_LEFT_ID2);   vTaskDelay(5);
    dm_disable(hcan, ARM_LEFT_ID3);   vTaskDelay(5);
}

void arm_left_set(CAN_HandleTypeDef *hcan, float p1, float p2, float p3, float vel)
{
    pos_ctrl(hcan, ARM_LEFT_ID1, p1, vel);  vTaskDelay(1);
    pos_ctrl(hcan, ARM_LEFT_ID2, p2, vel);  vTaskDelay(1);
    pos_ctrl(hcan, ARM_LEFT_ID3, p3, vel);  vTaskDelay(1);
}

void arm_left_update(CAN_HandleTypeDef *hcan, float vel)
{
    /* 协议 pitch2>0=肘部上折, 但电机 ID2 上=负方向, 取反 */
    float p1 = (float)left_pitch1 * 0.001f;
    float p2 = -(float)left_pitch2 * 0.001f;   // 方向反转
    float p3 = (float)left_pitch3 * 0.001f;

    pos_ctrl(hcan, ARM_LEFT_ID1, p1, vel);  vTaskDelay(1);
    pos_ctrl(hcan, ARM_LEFT_ID2, p2, vel);  vTaskDelay(1);
    pos_ctrl(hcan, ARM_LEFT_ID3, p3, vel);  vTaskDelay(1);
}

/* ==================== 右臂 ==================== */

void arm_right_enable(CAN_HandleTypeDef *hcan)
{
    dm_enable(hcan, ARM_RIGHT_ID1);  vTaskDelay(5);
    dm_enable(hcan, ARM_RIGHT_ID2);  vTaskDelay(5);
    dm_enable(hcan, ARM_RIGHT_ID3);  vTaskDelay(5);
}

void arm_right_disable(CAN_HandleTypeDef *hcan)
{
    dm_disable(hcan, ARM_RIGHT_ID1);  vTaskDelay(5);
    dm_disable(hcan, ARM_RIGHT_ID2);  vTaskDelay(5);
    dm_disable(hcan, ARM_RIGHT_ID3);  vTaskDelay(5);
}

void arm_right_set(CAN_HandleTypeDef *hcan, float p1, float p2, float p3, float vel)
{
    pos_ctrl(hcan, ARM_RIGHT_ID1, p1, vel);  vTaskDelay(1);
    pos_ctrl(hcan, ARM_RIGHT_ID2, p2, vel);  vTaskDelay(1);
    pos_ctrl(hcan, ARM_RIGHT_ID3, p3, vel);  vTaskDelay(1);
}

void arm_right_update(CAN_HandleTypeDef *hcan, float vel)
{
    /* ID4/ID6 方向反转: 协议正=上, 电机正=下; ID5 方向一致 */
    float p1 = -(float)right_pitch1 * 0.001f;  // ID4: 上=负, 取反
    float p2 =  (float)right_pitch2 * 0.001f;  // ID5: 上=正, 不变
    float p3 = -(float)right_pitch3 * 0.001f;  // ID6: 上=负, 取反

    pos_ctrl(hcan, ARM_RIGHT_ID1, p1, vel);  vTaskDelay(1);
    pos_ctrl(hcan, ARM_RIGHT_ID2, p2, vel);  vTaskDelay(1);
    pos_ctrl(hcan, ARM_RIGHT_ID3, p3, vel);
}

/* ==================== CAN2 RX 回调 ==================== */

void arm_can2_rx_callback(uint8_t *rx_data)
{
    uint8_t id = rx_data[0] & 0x0F;

    switch (id)
    {
        case 1: dm4310_fbdata(&arm_left_motor[0], rx_data);   break;
        case 2: dm4310_fbdata(&arm_left_motor[1], rx_data);   break;
        case 3: dm4310_fbdata(&arm_left_motor[2], rx_data);   break;
        case 4: dm4310_fbdata(&arm_right_motor[0], rx_data);  break;
        case 5: dm4310_fbdata(&arm_right_motor[1], rx_data);  break;
        case 6: dm4310_fbdata(&arm_right_motor[2], rx_data);  break;
        default: break;
    }
}
