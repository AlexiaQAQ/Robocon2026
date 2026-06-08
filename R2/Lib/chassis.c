#include "chassis.h"
#include "cmsis_os.h"

/* ==================== 全局变量 ==================== */
float lift_target_mm[4] = {0};

/* ==================== 静态常量 ==================== */

// 全向轮电机 ID (0=BR, 1=FR, 2=FL, 3=BL)
static const uint16_t wheel_id[4] = {WHEEL_BR, WHEEL_FR, WHEEL_FL, WHEEL_BL};

// 抬升电机 ID + 方向 (0=FR, 1=FL, 2=BL, 3=BR)
static const uint16_t lift_id[4]  = {LIFT_FR, LIFT_FL, LIFT_BL, LIFT_BR};
static const float    lift_dir[4] = {LIFT_DIR_FR, LIFT_DIR_FL, LIFT_DIR_BL, LIFT_DIR_BR};

// 齿条参数: 电机弧度 → 线性行程 (mm), 需实测校准
#define RACK_MM_PER_RAD  20.0f   // 417mm / 20.85rad

// 45° 投影系数
#define COS45  0.70710678118f

#define SPEED_SCALE  0.0001f

/* ==================== 初始化 / 使能 ==================== */

void chassis_init(CAN_HandleTypeDef *hcan)
{
    (void)hcan;
}

void chassis_enable(CAN_HandleTypeDef *hcan)
{
    for (int i = 1; i <= 8; i++)
    {
        dm_enable(hcan, (uint16_t)i);
        vTaskDelay(5);
    }
}

void chassis_disable(CAN_HandleTypeDef *hcan)
{
    for (int i = 1; i <= 8; i++)
    {
        dm_disable(hcan, (uint16_t)i);
        vTaskDelay(5);
    }
}

/* ==================== 45° 全向轮运动学 ==================== */

void chassis_update(CAN_HandleTypeDef *hcan)
{
    // 车头方向为 X 轴
    // 车轮: FR(0x02) FL(0x03) BR(0x01) BL(0x04)
    //
    // 45° 全向轮逆运动学:
    //   每个轮子的速度 = 底盘 Vx,Vy 在轮子切线方向的投影 + 旋转分量

    float motor_out[4];
    float vx_s = -set_vx * SPEED_SCALE * 1.9f;   // 负号: 修正前后方向
    float vy_s =  set_vy * SPEED_SCALE * 1.9f;
    float vw_s = -set_vw * CHASSIS_R * SPEED_SCALE * 1.9f;  // 负号: 修正旋转方向

    motor_out[0] = (-COS45 * vx_s - COS45 * vy_s + vw_s) / WHEEL_RADIUS; // BR
    motor_out[1] = ( COS45 * vx_s - COS45 * vy_s + vw_s) / WHEEL_RADIUS; // FR
    motor_out[2] = ( COS45 * vx_s + COS45 * vy_s + vw_s) / WHEEL_RADIUS; // FL
    motor_out[3] = (-COS45 * vx_s + COS45 * vy_s + vw_s) / WHEEL_RADIUS; // BL

    for (int i = 0; i < 4; i++)
    {
        dm_mit_ctrl(hcan, wheel_id[i], 0.0f, motor_out[i], 0.0f, CHASSIS_TORQUE, 0.0f);
        vTaskDelay(1);
    }
}

/* ==================== 抬升控制 ==================== */

void lift_update(CAN_HandleTypeDef *hcan, float vel)
{
    for (int i = 0; i < 4; i++)
    {
        float target_rad = lift_target_mm[i] / RACK_MM_PER_RAD * lift_dir[i];
        pos_ctrl(hcan, lift_id[i], target_rad, vel);
        vTaskDelay(1);
    }
}

void lift_set(uint8_t idx, float mm)
{
    if (idx < 4)
    {
        lift_target_mm[idx] = mm;
    }
}
