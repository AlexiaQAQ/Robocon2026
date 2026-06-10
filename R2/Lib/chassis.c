/**
 * @file    chassis.c
 * @brief   四45°全向轮底盘运动学 + 独立抬升实现
 * @author  Alexia
 * @date    2026-06-10
 *
 * 移植到 motor_control 新库 motor_t* API。
 * 全向轮: dm_motor[0~3] DM_4310 MIT 模式
 * 抬升:   dm_motor[4~7] DM_4310 POS 模式
 */

#include "chassis.h"
#include "cmsis_os.h"

/* ---- 全局变量 ---- */
float lift_target_mm[4] = {0};

/* ---- 内部表 ---- */
static const float lift_dir[4] = { LIFT_DIR_FR, LIFT_DIR_FL, LIFT_DIR_BL, LIFT_DIR_BR };

/* ================================================================
   初始化 — 注册电机型号
   ================================================================ */

void chassis_init(CAN_HandleTypeDef *hcan)
{
    (void)hcan;

    /* 全向轮: DM_4310 MIT 模式 */
    dm_init(&dm_motor[0], 1, DM_MODE_MIT, DM_4310);
    dm_init(&dm_motor[1], 2, DM_MODE_MIT, DM_4310);
    dm_init(&dm_motor[2], 3, DM_MODE_MIT, DM_4310);
    dm_init(&dm_motor[3], 4, DM_MODE_MIT, DM_4310);

    /* 独立抬升: DM_4310 POS 模式 */
    dm_init(&dm_motor[4], 5, DM_MODE_POS, DM_4310);
    dm_init(&dm_motor[5], 6, DM_MODE_POS, DM_4310);
    dm_init(&dm_motor[6], 7, DM_MODE_POS, DM_4310);
    dm_init(&dm_motor[7], 8, DM_MODE_POS, DM_4310);
}

/* ================================================================
   使能 / 失能
   ================================================================ */

void chassis_enable(CAN_HandleTypeDef *hcan)
{
    for (int i = 0; i < 8; i++)
    {
        dm_enable(hcan, &dm_motor[i]);
        vTaskDelay(5);
    }
}

void chassis_disable(CAN_HandleTypeDef *hcan)
{
    for (int i = 0; i < 8; i++)
    {
        dm_disable(hcan, &dm_motor[i]);
        vTaskDelay(5);
    }
}

/* ================================================================
   45° 全向轮运动学 (~2ms 周期)
   ================================================================ */

void chassis_update(CAN_HandleTypeDef *hcan)
{
    float vx_s = -set_vx * SPEED_SCALE * 1.9f;
    float vy_s =  set_vy * SPEED_SCALE * 1.9f;
    float vw_s = -set_vw * CHASSIS_R * SPEED_SCALE * 1.9f;

    float motor_out[4];
    motor_out[0] = (-COS45 * vx_s - COS45 * vy_s + vw_s) / WHEEL_RADIUS;  // BR
    motor_out[1] = ( COS45 * vx_s - COS45 * vy_s + vw_s) / WHEEL_RADIUS;  // FR
    motor_out[2] = ( COS45 * vx_s + COS45 * vy_s + vw_s) / WHEEL_RADIUS;  // FL
    motor_out[3] = (-COS45 * vx_s + COS45 * vy_s + vw_s) / WHEEL_RADIUS;  // BL

    for (int i = 0; i < 4; i++)
    {
        dm_mit_ctrl(hcan, &dm_motor[i],
                     0.0f, motor_out[i], 0.0f, CHASSIS_TORQUE, 0.0f);
        vTaskDelay(1);
    }
}

/* ================================================================
   独立抬升 (~50ms 周期)
   ================================================================ */

void lift_update(CAN_HandleTypeDef *hcan, float vel)
{
    for (int i = 0; i < 4; i++)
    {
        float target_rad = lift_target_mm[i] / RACK_MM_PER_RAD * lift_dir[i];
        dm_pos_ctrl(hcan, 5 + (uint16_t)i, target_rad, vel);
        vTaskDelay(1);
    }
}

void lift_set(uint8_t idx, float mm)
{
    if (idx < 4)
        lift_target_mm[idx] = mm;
}
