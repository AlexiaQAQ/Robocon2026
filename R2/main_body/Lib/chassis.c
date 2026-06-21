/**
 * @file    chassis.c
 * @brief   四45°全向轮底盘运动学 + 独立抬升实现
 * @author  Alexia
 * @date    2026-06-10
 *
 * 移植到 motor_control 新库 motor_t* API。
 * 全向轮: dm_motor[0~3] DM_3519 MIT 模式
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

    /* 全向轮: DM_3519 MIT 模式 */
    dm_init(&dm_motor[0], 1, DM_MODE_MIT, DM_3519);
    dm_init(&dm_motor[1], 2, DM_MODE_MIT, DM_3519);
    dm_init(&dm_motor[2], 3, DM_MODE_MIT, DM_3519);
    dm_init(&dm_motor[3], 4, DM_MODE_MIT, DM_3519);

    /*
     * 独立抬升: DM_4310 POS 模式
     * 反馈格式固定为 uint16→±12.5rad (单圈), 超出后 p_int 回卷。
     * dm_pos_ctrl 直接发 float 可超 12.5rad, 但回读需软件解卷绕 (见 uart_task.c)。
     */
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
    /* 速度限幅: ±2 m/s, ±3.2 rad/s (安全冗余, 串口侧已有第一道限幅) */
    float vx_clamp = (set_vx >  20000.0f) ?  20000.0f : ((set_vx < -20000.0f) ? -20000.0f : set_vx);
    float vy_clamp = (set_vy >  20000.0f) ?  20000.0f : ((set_vy < -20000.0f) ? -20000.0f : set_vy);
    float vw_clamp = (set_vw >  32000.0f) ?  32000.0f : ((set_vw < -32000.0f) ? -32000.0f : set_vw);

    float vx_s = -vx_clamp * SPEED_SCALE;
    float vy_s =  vy_clamp * SPEED_SCALE;
    float vw_s = -vw_clamp * CHASSIS_R * SPEED_SCALE;

    float motor_out[4];
    /*
     * 45° 全向轮逆运动学:
     *   轮 i 对前进的贡献 = ω_i × r × cos45°
     *   所以 ω_i = vx / (r × cos45°) 才能让实际车速 = vx
     *   旧公式 ω = COS45 × vx / r 实际前进 = COS45² × vx = 0.5×vx (偏慢一半)
     * 旋转项 vw_s/r 无需补偿: 轮子径向分量=1, 无 cos45 衰减
     */
    float t_scale = 1.0f / (WHEEL_RADIUS * COS45);
    motor_out[0] = -vx_s * t_scale - vy_s * t_scale + vw_s / WHEEL_RADIUS;  // BR
    motor_out[1] =  vx_s * t_scale - vy_s * t_scale + vw_s / WHEEL_RADIUS;  // FR
    motor_out[2] =  vx_s * t_scale + vy_s * t_scale + vw_s / WHEEL_RADIUS;  // FL
    motor_out[3] = -vx_s * t_scale + vy_s * t_scale + vw_s / WHEEL_RADIUS;  // BL

    /* 发送顺序: BR(1)→FR(2)→BL(4)→FL(3) */
    static const uint8_t order[4] = {0, 1, 3, 2};
    for (int i = 0; i < 4; i++)
    {
        uint8_t idx = order[i];
        dm_mit_ctrl(hcan, &dm_motor[idx], 0.0f, motor_out[idx], 0.0f, CHASSIS_TORQUE, 0.0f);
        vTaskDelay(1);
    }
}

/* ================================================================
   独立抬升 (~50ms 周期)
   ================================================================ */

void lift_update(CAN_HandleTypeDef *hcan, float vel)
{
    static const float offset[4] = { LIFT_OFFSET_FR, LIFT_OFFSET_FL, LIFT_OFFSET_BL, LIFT_OFFSET_BR };

    for (int i = 0; i < 4; i++)
    {
        float target_rad = (lift_target_mm[i] / RACK_MM_PER_RAD + offset[i]) * lift_dir[i];
        dm_pos_ctrl(hcan, 5 + (uint16_t)i, target_rad, vel);
        vTaskDelay(1);
    }
}

void lift_set(uint8_t idx, float mm)
{
    if (idx < 4)
        lift_target_mm[idx] = mm;
}
