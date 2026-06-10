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
    DM_4310,        /* 额定~5Nm,  峰值10Nm,  24V空载~286rpm  PMAX=12.5 VMAX=30 TMAX=10 */
    DM_4310_48V,    /* 同上48V供电,             峰值18Nm         VMAX=45 TMAX=18 */
    DM_4340,        /* 额定~9Nm,  峰值27Nm,  24V空载~    rpm  PMAX=12.5 VMAX=30 TMAX=27 */
    DM_3519,        /* 额定3.5Nm, 峰值~8Nm,  19:1减速  空载~435rpm(电机侧) PMAX=12.5 VMAX=20 TMAX=8 */
    DM_8006,        /* 额定8Nm,   峰值20Nm,  6:1减速   24V空载~194rpm PMAX=12.5 VMAX=21 TMAX=20 */
    DM_8009,        /* 额定20Nm,  峰值40Nm,  9:1减速   24V空载~168rpm PMAX=12.5 VMAX=18 TMAX=40 */
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
   电机命令 (自动使用 motor->mode 计算 CAN ID 偏移)
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

#endif
