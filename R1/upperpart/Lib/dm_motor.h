#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

#include "main.h"
#include "can.h"

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

/**
 * @brief DM 电机反馈数据结构体
 * @note  由 dm_fb_parse() 从 CAN 接收数据中解析填充
 */
typedef struct
{
    int     id;             /* 电机 CAN ID (1~8) */
    int     state;          /* 电机状态码 */
    int     p_int;          /* 位置原始值 (uint16) */
    int     v_int;          /* 速度原始值 (uint12) */
    int     t_int;          /* 扭矩原始值 (uint12) */
    int     kp_int;         /* Kp 原始值 */
    int     kd_int;         /* Kd 原始值 */
    float   pos;            /* 位置 (rad) */
    float   vel;            /* 速度 (rad/s) */
    float   tor;            /* 扭矩 (Nm) */
    float   Kp;             /* 位置比例系数 */
    float   Kd;             /* 速度微分系数 */
    float   Tmos;           /* MOS 管温度 (°C) */
    float   Tcoil;          /* 线圈温度 (°C) */
    int16_t angle_pos;      /* 角度制位置 */
} motor_fbpara_t;

/**
 * @brief DM 电机控制量结构体
 */
typedef struct
{
    float   pos_set;        /* 位置给定 */
    float   vel_set;        /* 速度给定 */
    float   tor_set;        /* 扭矩给定 */
    float   cur_set;        /* 电流给定 */
    float   kp_set;         /* Kp 给定 */
    float   kd_set;         /* Kd 给定 */
} motor_ctrl_t;

/**
 * @brief DM 电机总结构体
 * @note  id + mode 自动计算 CAN ID 偏移, p_max/v_max/t_max 控制 MIT 映射范围
 */
typedef struct
{
    int8_t          id;         /* CAN 命令 ID (1~8) */
    uint8_t         mode;       /* 控制模式 (MIT / POS / SPD / PSI) */
    uint8_t         start_flag; /* 电机已使能标志 */
    float           p_max;      /* MIT 位置映射范围 ± (rad) */
    float           v_max;      /* MIT 速度映射范围 ± (rad/s) */
    float           t_max;      /* MIT 扭矩映射范围 ± (Nm) */
    motor_fbpara_t  para;       /* 反馈数据 */
    motor_ctrl_t    ctrl;       /* 控制指令 */
} motor_t;

extern motor_t dm_motor[8];

/* ================================================================
   工具函数
   ================================================================ */

float   angle_to_rads(int16_t angle);
int16_t rads_to_angle(float rads);
int16_t mabs(int16_t t);
int     float_to_uint(float x_float, float x_min, float x_max, int bits);
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
   电机命令
   ================================================================ */

HAL_StatusTypeDef dm_enable(CAN_HandleTypeDef *hcan, motor_t *motor);
HAL_StatusTypeDef dm_disable(CAN_HandleTypeDef *hcan, motor_t *motor);
HAL_StatusTypeDef dm_save_zero(CAN_HandleTypeDef *hcan, motor_t *motor);
HAL_StatusTypeDef dm_clear_err(CAN_HandleTypeDef *hcan, motor_t *motor);

/* ================================================================
   控制模式
   ================================================================ */

HAL_StatusTypeDef dm_mit_ctrl(CAN_HandleTypeDef *hcan, motor_t *motor,
                              float pos, float vel, float kp, float kd, float torq);

HAL_StatusTypeDef dm_pos_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id,
                              float pos, float vel);

HAL_StatusTypeDef dm_spd_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float vel);

HAL_StatusTypeDef dm_psi_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id,
                              float pos, float vel, float cur);

/* ================================================================
   反馈解析
   ================================================================ */

void dm_fb_parse(motor_t *motor, uint8_t *rx_data);
void dm_rx_cbk(motor_t *motor_set, uint8_t *rx_data);

#endif
