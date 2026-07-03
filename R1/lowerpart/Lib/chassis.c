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

/* ---- 初始化 ---- */
void chassis_init(void)
{
    dm_init(&dm_motor[0], 1, DM_MODE_SPD, DM_3519);   // BR
    dm_init(&dm_motor[1], 2, DM_MODE_SPD, DM_3519);   // FR
    dm_init(&dm_motor[2], 4, DM_MODE_SPD, DM_3519);   // BL
    dm_init(&dm_motor[3], 3, DM_MODE_SPD, DM_3519);   // FL
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

    float vx_s = -set_vx;
    float vy_s =  set_vy;
    float vw_s = -set_vw;

    float motor_out[4];

    // BR: 45° 左前安装
    motor_out[0] = (-vx_s - vy_s + vw_s);

    // FR: 45° 右前安装
    motor_out[1] = ( vx_s - vy_s + vw_s);

    // BL: 45° 左后安装
    motor_out[2] = (-vx_s + vy_s + vw_s);

    // FL: 45° 右后安装
    motor_out[3] = ( vx_s + vy_s + vw_s);

    // SPD 速度模式: 直接 float 速度
    for (int i = 0; i < 4; i++)
    {
        dm_spd_ctrl(&hcan1, dm_motor[i].id, motor_out[i]);
        vTaskDelay(1);
    }
}
