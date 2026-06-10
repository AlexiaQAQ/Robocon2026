/**
 * @file    chassis.c
 * @brief   四45°全向轮底盘运动学实现
 * @author  Alexia
 * @date    2026-06-10
 *
 * 移植自 R2 chassis.c，适配 R1 的 motor_t* 风格电机 API。
 * 全部电机走 CAN1 (原生 bxCAN)，不使用 MCP2515。
 */

#include "chassis.h"
#include "can.h"
#include "cmsis_os.h"

/* ---- 内部电机 ID 表 ---- */
static const uint16_t wheel_id[4] = {1, 2, 3, 4};  // BR, FR, FL, BL

/* ---- 初始化 ---- */
void chassis_init(void)
{
    dm_init(&dm_motor[0], 1, DM_MODE_MIT, DM_4310);
    dm_init(&dm_motor[1], 2, DM_MODE_MIT, DM_4310);
    dm_init(&dm_motor[2], 3, DM_MODE_MIT, DM_4310);
    dm_init(&dm_motor[3], 4, DM_MODE_MIT, DM_4310);
}

/* ---- 使能 (逐个, 间隔 5ms 防止 CAN 拥塞) ---- */
void chassis_enable(void)
{
    for (int i = 0; i < 4; i++)
    {
        dm_enable(&hcan1, &dm_motor[i]);
        vTaskDelay(5);
    }
}

/* ---- 失能 ---- */
void chassis_disable(void)
{
    for (int i = 0; i < 4; i++)
    {
        dm_disable(&hcan1, &dm_motor[i]);
        vTaskDelay(5);
    }
}

/* ---- 45° 全向轮逆运动学 + CAN1 发送 (~2ms 周期) ---- */
void chassis_update(void)
{
    /*
     * 方向修正:
     *   vx_s: -set_vx (遥控前推=正, 但电机反方向时需要取负)
     *   vy_s:  set_vy
     *   vw_s: -set_vw (遥控左转=正, 电机反方向)
     *
     * 1.9: 雷达实测校准系数, 与 R2 一致
     */

    float vx_s = -set_vx * SPEED_SCALE * 1.9f;
    float vy_s =  set_vy * SPEED_SCALE * 1.9f;
    float vw_s = -set_vw * CHASSIS_R * SPEED_SCALE * 1.9f;

    float motor_out[4];

    // BR: 45° 左前安装, 投影到 X/Y 分量为 -cos45/-cos45
    motor_out[0] = (-COS45 * vx_s - COS45 * vy_s + vw_s) / WHEEL_RADIUS;

    // FR: 45° 右前安装, 投影到 X/Y 分量为 +cos45/-cos45
    motor_out[1] = ( COS45 * vx_s - COS45 * vy_s + vw_s) / WHEEL_RADIUS;

    // FL: 45° 右后安装, 投影到 X/Y 分量为 +cos45/+cos45
    motor_out[2] = ( COS45 * vx_s + COS45 * vy_s + vw_s) / WHEEL_RADIUS;

    // BL: 45° 左后安装, 投影到 X/Y 分量为 -cos45/+cos45
    motor_out[3] = (-COS45 * vx_s + COS45 * vy_s + vw_s) / WHEEL_RADIUS;

    // 逐个发送 MIT 指令, 间隔 1ms 防止 CAN 拥塞
    for (int i = 0; i < 4; i++)
    {
        dm_mit_ctrl(&hcan1, &dm_motor[i],
                     0.0f,            // pos = 0 (速度模式)
                     motor_out[i],    // vel = 目标角速度 (rad/s)
                     0.0f,            // kp = 0
                     CHASSIS_TORQUE,  // kd = 阻尼
                     0.0f);           // torque = 0
        vTaskDelay(1);
    }
}
