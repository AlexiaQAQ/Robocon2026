/**
 * @file    motor_control.h
 * @brief   达妙 (DM) 电机控制库 — MIT/位置/速度/位速流模式
 * @author  Alexia
 * @date    2026-06-10
 *
 * 移植自 R1 lowerpart，统一 native CAN + MCP2515 双总线支持。
 */

#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

#include "main.h"
#include "can.h"
#include "mcp2515.h"

/* ---- DM 电机控制模式 (叠加到 CAN ID 上) ---- */
#define MIT_MODE    0x000
#define POS_MODE    0x100
#define SPD_MODE    0x200
#define PSI_MODE    0x300

/* ---- MIT 模式 Kp/Kd 范围 (所有电机通用) ---- */
#define KP_MIN    0.0f
#define KP_MAX  500.0f
#define KD_MIN    0.0f
#define KD_MAX    5.0f

/* ---- 电机型号 ---- */
typedef enum {
    DM_4310,
    DM_4310_48V,
    DM_4340,
    DM_4340_48V,
    DM_3519,
    DM_8006,
    DM_8009,
    DM_10010L,
    DM_10010,
    DMH3510,
    DMH6215,
    DMG6220,
    DM_CUSTOM       /* 用户自行设置范围, 默认同 DM_4310 */
} dm_model_t;

/* ---- 运行模式 ---- */
typedef enum {
    DM_MODE_MIT = 0,
    DM_MODE_POS = 1,
    DM_MODE_SPD = 2,
    DM_MODE_PSI = 3
} dm_mode_t;

/* ---- 电机反馈 ---- */
typedef struct
{
    int     id;
    int     state;
    int     p_int;
    int     v_int;
    int     t_int;
    int     kp_int;
    int     kd_int;
    float   pos;            /* MIT: 位置 (rad) */
    float   vel;            /* MIT: 速度 (rad/s) */
    float   tor;            /* MIT: 扭矩 (Nm) */
    float   Kp;
    float   Kd;
    float   Tmos;           /* MOS 管温度 */
    float   Tcoil;          /* 线圈温度 */
    int16_t angle_pos;      /* 角度制位置 */
} motor_fbpara_t;

/* ---- 电机控制量 ---- */
typedef struct
{
    float   pos_set;
    float   vel_set;
    float   tor_set;
    float   cur_set;
    float   kp_set;
    float   kd_set;
} motor_ctrl_t;

/* ---- 电机总结构体 ---- */
typedef struct
{
    int8_t          id;         /* CAN 命令 ID (1~8) */
    uint8_t         mode;       /* dm_mode_t: MIT/POS/SPD/PSI */
    uint8_t         start_flag;
    float           p_max;      /* MIT 位置范围 ± (rad), 如 12.5 */
    float           v_max;      /* MIT 速度范围 ± (rad/s), 如 30.0 */
    float           t_max;      /* MIT 扭矩范围 ± (Nm), 如 10.0 */
    motor_fbpara_t  para;       /* 反馈数据 */
    motor_ctrl_t    ctrl;       /* 控制指令 */
} motor_t;

/* ---- 全局电机数组 (各模块按需分配索引) ---- */
extern motor_t dm_motor[8];

/* ================================================================
   工具函数
   ================================================================ */

float   angle_to_rads(int16_t angle);
int16_t rads_to_angle(float rads);
int16_t mabs(int16_t t);
int     float_to_uint(float x, float x_min, float x_max, int bits);
float   uint_to_float(int x_int, float x_min, float x_max, int bits);

/* ================================================================
   底层 CAN 发送
   ================================================================ */

HAL_StatusTypeDef dm_can_send(CAN_HandleTypeDef *hcan, uint16_t id,
                              uint8_t *data, uint32_t len);

/* ================================================================
   电机初始化 / 参数设置
   ================================================================ */

void    dm_init(motor_t *motor, uint8_t id, dm_mode_t mode, dm_model_t model);
void    dm_set_mit_range(motor_t *motor, float p_max, float v_max, float t_max);

/* ================================================================
   电机命令 (native CAN, 自动使用 motor->mode 计算 CAN ID 偏移)
   ================================================================ */

HAL_StatusTypeDef dm_enable(CAN_HandleTypeDef *hcan, motor_t *motor);
HAL_StatusTypeDef dm_disable(CAN_HandleTypeDef *hcan, motor_t *motor);
HAL_StatusTypeDef dm_save_zero(CAN_HandleTypeDef *hcan, motor_t *motor);
HAL_StatusTypeDef dm_clear_err(CAN_HandleTypeDef *hcan, motor_t *motor);

/* ================================================================
   MIT 模式控制 (使用 motor->p_max/v_max/t_max 做范围映射)
   ================================================================ */

HAL_StatusTypeDef dm_mit_ctrl(CAN_HandleTypeDef *hcan, motor_t *motor,
                              float pos, float vel, float kp, float kd, float torq);

/* ================================================================
   位置-速度模式 (POS_MODE, 直接发送 float, 无需范围映射)
   ================================================================ */

HAL_StatusTypeDef dm_pos_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id,
                              float pos, float vel);

/* ================================================================
   速度模式 (SPD_MODE)
   ================================================================ */

HAL_StatusTypeDef dm_spd_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float vel);

/* ================================================================
   位速流模式 (PSI_MODE)
   ================================================================ */

HAL_StatusTypeDef dm_psi_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id,
                              float pos, float vel, float cur);

/* ================================================================
   反馈解析 (MIT 模式, 使用 motor->p_max/v_max/t_max)
   ================================================================ */

void dm_fb_parse(motor_t *motor, uint8_t *rx_data);

/* ================================================================
   CAN 接收回调 — 传入电机数组基址, 自动提取 ID 分发
   ================================================================ */

void dm_rx_cbk(motor_t *motor_set, uint8_t *rx_data);

/* ================================================================
   MCP2515 总线封装 (hcan3/hcan4/hcan5)
   ================================================================ */

HAL_StatusTypeDef dm_enable_mcp2515(MCP2515_HandleTypeDef *hcan, motor_t *motor);
HAL_StatusTypeDef dm_disable_mcp2515(MCP2515_HandleTypeDef *hcan, motor_t *motor);
HAL_StatusTypeDef dm_pos_ctrl_mcp2515(MCP2515_HandleTypeDef *hcan, uint16_t motor_id,
                                      float pos, float vel);
HAL_StatusTypeDef dm_mit_ctrl_mcp2515(MCP2515_HandleTypeDef *hcan, motor_t *motor,
                                      float pos, float vel, float kp, float kd, float torq);

/* ---- 旧 API 兼容宏 (逐步废弃) ---- */
#define dm4310_fbdata(m, d)   dm_fb_parse(m, d)

#endif
