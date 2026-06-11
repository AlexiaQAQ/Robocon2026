#include "motor_control.h"
#include <string.h>

motor_t dm_motor[8];

/* ---- 各型号 MIT 范围表 ---- */
typedef struct {
    float p_max;
    float v_max;
    float t_max;
} dm_range_t;

static const dm_range_t dm_range_table[] = {
    [DM_4310]     = { 12.5f,  30.0f,  10.0f },
    [DM_4310_48V] = { 12.5f,  50.0f,  10.0f },
    [DM_4340]     = { 12.5f,  8.0f,   28.0f },
	[DM_4340_48V] = { 12.5f,  10.0f,  28.0f },
    [DM_3519]     = { 12.5f,  200.0f, 10.0f },
    [DM_8006]     = { 12.5f,  45.0f,  40.0f },
    [DM_8009]     = { 12.5f,  45.0f,  54.0f },
	[DM_10010L]   = { 12.5f,  25.0f, 200.0f },
	[DM_10010]    = { 12.5f,  20.0f, 200.0f },
	[DMH3510]     = { 12.5f,  280.0f,200.0f },
	[DMH6215]     = { 12.5f,  45.0f,  10.0f },
	[DMG6220]     = { 12.5f,  45.0f,  10.0f },
    [DM_CUSTOM]   = { 12.5f,  30.0f,  10.0f },  /* 默认*/
};

/* ---- 命令字节 ---- */
#define DM_CMD_ENABLE    0xFC
#define DM_CMD_DISABLE   0xFD
#define DM_CMD_SAVE_ZERO 0xFE
#define DM_CMD_CLEAR_ERR 0xFB

/* mode -> 总线 ID 偏移 */
static const uint16_t mode_offset[] = {
    [DM_MODE_MIT] = MIT_MODE,
    [DM_MODE_POS] = POS_MODE,
    [DM_MODE_SPD] = SPD_MODE,
    [DM_MODE_PSI] = PSI_MODE,
};

/* ================================================================
   工具函数
   ================================================================ */

float angle_to_rads(int16_t angle)
{
    return (float)(angle * 0.017453292f);
}

int16_t rads_to_angle(float rads)
{
    return (int16_t)(rads * 57.29578f);
}

int16_t mabs(int16_t t)
{
    if (t == INT16_MIN) return INT16_MAX;
    return (t > 0) ? t : (int16_t)(-t);
}

int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/* ================================================================
   底层 CAN 发送
   ================================================================ */

HAL_StatusTypeDef dm_can_send(CAN_HandleTypeDef *hcan, uint16_t id,
                              uint8_t *data, uint32_t len)
{
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef tx_header;

    tx_header.StdId = id;
    tx_header.ExtId = 0;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = len;

    return HAL_CAN_AddTxMessage(hcan, &tx_header, data, &send_mail_box);
}

/* ================================================================
   初始化
   ================================================================ */

void dm_init(motor_t *motor, uint8_t id, dm_mode_t mode, dm_model_t model)
{
    const dm_range_t *r = &dm_range_table[model];

    memset(motor, 0, sizeof(motor_t));
    motor->id    = id;
    motor->mode  = mode;
    motor->p_max = r->p_max;
    motor->v_max = r->v_max;
    motor->t_max = r->t_max;
}

void dm_set_mit_range(motor_t *motor, float p_max, float v_max, float t_max)
{
    motor->p_max = p_max;
    motor->v_max = v_max;
    motor->t_max = t_max;
}

/* ================================================================
   发送命令帧 (内部, 所有命令共用格式, 仅末字节不同)
   ================================================================ */

static HAL_StatusTypeDef dm_send_cmd(CAN_HandleTypeDef *hcan, motor_t *motor, uint8_t cmd)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd};
    uint16_t id = motor->id + mode_offset[motor->mode];
    return dm_can_send(hcan, id, data, 8);
}

/* ================================================================
   电机命令
   ================================================================ */

HAL_StatusTypeDef dm_enable(CAN_HandleTypeDef *hcan, motor_t *motor)
{
    return dm_send_cmd(hcan, motor, DM_CMD_ENABLE);
}

HAL_StatusTypeDef dm_disable(CAN_HandleTypeDef *hcan, motor_t *motor)
{
    return dm_send_cmd(hcan, motor, DM_CMD_DISABLE);
}

HAL_StatusTypeDef dm_save_zero(CAN_HandleTypeDef *hcan, motor_t *motor)
{
    return dm_send_cmd(hcan, motor, DM_CMD_SAVE_ZERO);
}

HAL_StatusTypeDef dm_clear_err(CAN_HandleTypeDef *hcan, motor_t *motor)
{
    return dm_send_cmd(hcan, motor, DM_CMD_CLEAR_ERR);
}

/* ================================================================
   MIT 模式控制 (使用 per-motor PMAX/VMAX/TMAX)
   ================================================================ */

HAL_StatusTypeDef dm_mit_ctrl(CAN_HandleTypeDef *hcan, motor_t *motor,
                              float pos, float vel, float kp, float kd, float torq)
{
    uint8_t data[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

    pos_tmp = float_to_uint(pos,  -motor->p_max, motor->p_max, 16);
    vel_tmp = float_to_uint(vel,  -motor->v_max, motor->v_max, 12);
    kp_tmp  = float_to_uint(kp,    KP_MIN,       KP_MAX,       12);
    kd_tmp  = float_to_uint(kd,    KD_MIN,       KD_MAX,       12);
    tor_tmp = float_to_uint(torq, -motor->t_max, motor->t_max, 12);

    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    data[7] = tor_tmp;

    return dm_can_send(hcan, motor->id + MIT_MODE, data, 8);
}

/* ================================================================
   位置-速度模式 (POS_MODE) — 直接发 float, 无需范围映射
   ================================================================ */

HAL_StatusTypeDef dm_pos_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id,
                              float pos, float vel)
{
    uint8_t data[8];
    union { float f; uint8_t b[4]; } u;

    u.f = pos;
    data[0] = u.b[0]; data[1] = u.b[1]; data[2] = u.b[2]; data[3] = u.b[3];

    u.f = vel;
    data[4] = u.b[0]; data[5] = u.b[1]; data[6] = u.b[2]; data[7] = u.b[3];

    return dm_can_send(hcan, motor_id + POS_MODE, data, 8);
}

/* ================================================================
   速度模式 (SPD_MODE)
   ================================================================ */

HAL_StatusTypeDef dm_spd_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float vel)
{
    uint8_t data[4];
    union { float f; uint8_t b[4]; } u;

    u.f = vel;
    data[0] = u.b[0]; data[1] = u.b[1]; data[2] = u.b[2]; data[3] = u.b[3];

    return dm_can_send(hcan, motor_id + SPD_MODE, data, 4);
}

/* ================================================================
   PVT模式 (PSI_MODE)
   ================================================================ */

HAL_StatusTypeDef dm_psi_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id,
                              float pos, float vel, float cur)
{
    uint8_t data[8];
    union { float f; uint8_t b[4]; } u;
    uint16_t u16_vel = (uint16_t)(vel * 100);
    uint16_t u16_cur = (uint16_t)(cur * 10000);

    u.f = pos;
    data[0] = u.b[0]; data[1] = u.b[1]; data[2] = u.b[2]; data[3] = u.b[3];

    data[4] = (uint8_t)(u16_vel);
    data[5] = (uint8_t)(u16_vel >> 8);

    data[6] = (uint8_t)(u16_cur);
    data[7] = (uint8_t)(u16_cur >> 8);

    return dm_can_send(hcan, motor_id + PSI_MODE, data, 8);
}

/* ================================================================
   反馈解析 (MIT 模式, 使用 per-motor PMAX/VMAX/TMAX)
   ================================================================ */

void dm_fb_parse(motor_t *motor, uint8_t *rx_data)
{
    motor->para.id    = (rx_data[0]) & 0x0F;
    motor->para.state = (rx_data[0]) >> 4;
    motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
    motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];

    motor->para.pos = uint_to_float(motor->para.p_int, -motor->p_max, motor->p_max, 16);
    motor->para.vel = uint_to_float(motor->para.v_int, -motor->v_max, motor->v_max, 12);
    motor->para.tor = uint_to_float(motor->para.t_int, -motor->t_max, motor->t_max, 12);

    motor->para.Tmos  = (float)(rx_data[6]);
    motor->para.Tcoil = (float)(rx_data[7]);

    motor->para.angle_pos = rads_to_angle(motor->para.pos);
}

/* ================================================================
   CAN 接收回调 — 从反馈首字节提取 ID 分发
   ================================================================ */

void dm_rx_cbk(motor_t *motor_set, uint8_t *rx_data)
{
    uint8_t id = rx_data[0] & 0x0F;

    if (id >= 1 && id <= 8)
        dm_fb_parse(&motor_set[id - 1], rx_data);
}
