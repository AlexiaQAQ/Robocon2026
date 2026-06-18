#include "dm_motor.h"
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
    [DM_4340]     = { 12.5f,  10.0f,  28.0f },
	[DM_4340_48V] = { 12.5f,  10.0f,  28.0f },
    [DM_3519]     = { 12.5f,  200.0f, 10.0f },
    [DM_8006]     = { 12.5f,  45.0f,  40.0f },
    [DM_8009]     = { 12.5f,  45.0f,  54.0f },
	[DM_10010L]   = { 12.5f,  25.0f, 200.0f },
	[DM_10010]    = { 12.5f,  20.0f, 200.0f },
	[DMH3510]     = { 12.5f,  280.0f,200.0f },
	[DMH6215]     = { 12.5f,  45.0f,  10.0f },
	[DMG6220]     = { 12.5f,  45.0f,  10.0f },
    [DM_CUSTOM]   = { 12.5f,  30.0f,  10.0f },
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

/**
************************************************************************
* @brief:      	angle_to_rads: 角度转弧度
* @param[in]:   angle: 角度值 (int16_t, 0.01°/bit)
* @retval:     	弧度值 (float)
* @details:    	将角度转换为弧度，转换系数 0.017453292 (π/180)
************************************************************************
**/
float angle_to_rads(int16_t angle)
{
    return (float)(angle * 0.017453292f);
}

/**
************************************************************************
* @brief:      	rads_to_angle: 弧度转角度
* @param[in]:   rads: 弧度值 (float)
* @retval:     	角度值 (int16_t, 0.01°/bit)
* @details:    	将弧度转换为角度，转换系数 57.29578 (180/π)
************************************************************************
**/
int16_t rads_to_angle(float rads)
{
    return (int16_t)(rads * 57.29578f);
}

/**
************************************************************************
* @brief:      	mabs: 取绝对值
* @param[in]:   t: 有符号16位整数
* @retval:     	绝对值 (int16_t)
* @details:    	返回输入值的绝对值，处理 INT16_MIN 边界情况
************************************************************************
**/
int16_t mabs(int16_t t)
{
    if (t == INT16_MIN) return INT16_MAX;
    return (t > 0) ? t : (int16_t)(-t);
}

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数
* @param[in]:   x_float: 待转换的浮点数
* @param[in]:   x_min:   范围最小值
* @param[in]:   x_max:   范围最大值
* @param[in]:   bits:    目标无符号整数的位数
* @retval:     	无符号整数结果 (int)
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，
*               映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数
* @param[in]:   x_int:  待转换的无符号整数
* @param[in]:   x_min:  范围最小值
* @param[in]:   x_max:  范围最大值
* @param[in]:   bits:   无符号整数的位数
* @retval:     	浮点数结果 (float)
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性
*               映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/* ================================================================
   底层 CAN 发送
   ================================================================ */

/**
************************************************************************
* @brief:      	dm_can_send: CAN 总线数据发送
* @param[in]:   hcan: 指向 CAN_HandleTypeDef 结构的指针
* @param[in]:   id:   标准帧 CAN ID (11-bit)
* @param[in]:   data: 指向发送数据缓冲区的指针
* @param[in]:   len:  数据长度 (DLC, 0~8)
* @retval:     	HAL 状态 (HAL_StatusTypeDef)
* @details:    	通过 HAL 库向指定 CAN 总线发送标准数据帧
************************************************************************
**/
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
   电机初始化 / 参数设置
   ================================================================ */

/**
************************************************************************
* @brief:      	dm_init: DM 电机初始化
* @param[in]:   motor: 指向 motor_t 结构的指针
* @param[in]:   id:    电机 CAN ID (1~8)
* @param[in]:   mode:  控制模式 (MIT / POS / SPD / PSI)
* @param[in]:   model: 电机型号, 决定 MIT 参数范围 (PMAX/VMAX/TMAX)
* @retval:     	void
* @details:    	清零电机结构体并设置 ID、模式和型号对应的 MIT 范围参数
************************************************************************
**/
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

/**
************************************************************************
* @brief:      	dm_set_mit_range: 设置电机 MIT 参数范围
* @param[in]:   motor: 指向 motor_t 结构的指针
* @param[in]:   p_max: 位置映射范围最大值 (rad)
* @param[in]:   v_max: 速度映射范围最大值 (rad/s)
* @param[in]:   t_max: 扭矩映射范围最大值 (Nm)
* @retval:     	void
* @details:    	覆盖电机的默认 MIT 范围参数, 用于 DM_CUSTOM 等自定义型号
************************************************************************
**/
void dm_set_mit_range(motor_t *motor, float p_max, float v_max, float t_max)
{
    motor->p_max = p_max;
    motor->v_max = v_max;
    motor->t_max = t_max;
}

/* ================================================================
   电机命令 (内部函数)
   ================================================================ */

/**
************************************************************************
* @brief:      	dm_send_cmd: 发送电机命令帧 (内部函数)
* @param[in]:   hcan:  指向 CAN_HandleTypeDef 结构的指针
* @param[in]:   motor: 指向 motor_t 结构的指针
* @param[in]:   cmd:   命令字节 (0xFC=使能, 0xFD=失能, 0xFE=保存零点, 0xFB=清除错误)
* @retval:     	HAL 状态 (HAL_StatusTypeDef)
* @details:    	根据 motor->mode 自动计算 CAN ID 偏移, 发送 8 字节命令帧
************************************************************************
**/
static HAL_StatusTypeDef dm_send_cmd(CAN_HandleTypeDef *hcan, motor_t *motor, uint8_t cmd)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd};
    uint16_t id = motor->id;
    return dm_can_send(hcan, id, data, 8);
}

/* ================================================================
   电机命令 (对外接口)
   ================================================================ */

/**
************************************************************************
* @brief:      	dm_enable: 启用电机控制模式
* @param[in]:   hcan:  指向 CAN_HandleTypeDef 结构的指针
* @param[in]:   motor: 指向 motor_t 结构的指针
* @retval:     	HAL 状态 (HAL_StatusTypeDef)
* @details:    	根据 motor->mode 自动选择对应的 CAN ID 偏移, 发送使能命令 (0xFC)
************************************************************************
**/
HAL_StatusTypeDef dm_enable(CAN_HandleTypeDef *hcan, motor_t *motor)
{
    return dm_send_cmd(hcan, motor, DM_CMD_ENABLE);
}

/**
************************************************************************
* @brief:      	dm_disable: 禁用电机控制模式
* @param[in]:   hcan:  指向 CAN_HandleTypeDef 结构的指针
* @param[in]:   motor: 指向 motor_t 结构的指针
* @retval:     	HAL 状态 (HAL_StatusTypeDef)
* @details:    	发送失能命令 (0xFD), 电机停止控制
************************************************************************
**/
HAL_StatusTypeDef dm_disable(CAN_HandleTypeDef *hcan, motor_t *motor)
{
    return dm_send_cmd(hcan, motor, DM_CMD_DISABLE);
}

/**
************************************************************************
* @brief:      	dm_save_zero: 保存当前位置为零点
* @param[in]:   hcan:  指向 CAN_HandleTypeDef 结构的指针
* @param[in]:   motor: 指向 motor_t 结构的指针
* @retval:     	HAL 状态 (HAL_StatusTypeDef)
* @details:    	发送保存零点命令 (0xFE), 电机将当前位置记录为零位
************************************************************************
**/
HAL_StatusTypeDef dm_save_zero(CAN_HandleTypeDef *hcan, motor_t *motor)
{
    return dm_send_cmd(hcan, motor, DM_CMD_SAVE_ZERO);
}

/**
************************************************************************
* @brief:      	dm_clear_err: 清除电机错误
* @param[in]:   hcan:  指向 CAN_HandleTypeDef 结构的指针
* @param[in]:   motor: 指向 motor_t 结构的指针
* @retval:     	HAL 状态 (HAL_StatusTypeDef)
* @details:    	发送清除错误命令 (0xFB), 清除电机的故障状态
************************************************************************
**/
HAL_StatusTypeDef dm_clear_err(CAN_HandleTypeDef *hcan, motor_t *motor)
{
    return dm_send_cmd(hcan, motor, DM_CMD_CLEAR_ERR);
}

/* ================================================================
   MIT 模式控制
   ================================================================ */

/**
************************************************************************
* @brief:      	dm_mit_ctrl: MIT 模式下的电机控制
* @param[in]:   hcan:  指向 CAN_HandleTypeDef 结构的指针
* @param[in]:   motor: 指向 motor_t 结构的指针 (使用 motor->p_max/v_max/t_max)
* @param[in]:   pos:   位置给定值 (rad, 映射到 ±p_max)
* @param[in]:   vel:   速度给定值 (rad/s, 映射到 ±v_max)
* @param[in]:   kp:    位置比例系数 (0 ~ 500)
* @param[in]:   kd:    速度微分系数 (0 ~ 5)
* @param[in]:   torq:  转矩前馈值 (Nm, 映射到 ±t_max)
* @retval:     	HAL 状态 (HAL_StatusTypeDef)
* @details:    	通过 CAN 总线向电机发送 MIT 模式下的控制帧 (8 字节)
*               kp=0 → 纯速度控制, kd 提供阻尼防振荡
************************************************************************
**/
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
   位置-速度模式 (POS_MODE)
   ================================================================ */

/**
************************************************************************
* @brief:      	dm_pos_ctrl: 位置速度控制
* @param[in]:   hcan:     指向 CAN_HandleTypeDef 结构的指针
* @param[in]:   motor_id: 电机 CAN ID
* @param[in]:   pos:      位置给定值 (float, rad)
* @param[in]:   vel:      速度给定值 (float, rad/s)
* @retval:     	HAL 状态 (HAL_StatusTypeDef)
* @details:    	通过 CAN 总线向电机发送位置-速度模式控制帧 (8 字节, 直接发送 float)
************************************************************************
**/
HAL_StatusTypeDef dm_pos_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id,
                              float pos, float vel)
{
    uint8_t data[8];
    uint8_t *pbuf, *vbuf;

    pbuf = (uint8_t*)&pos;
    vbuf = (uint8_t*)&vel;

    data[0] = *pbuf;     data[1] = *(pbuf+1);
    data[2] = *(pbuf+2); data[3] = *(pbuf+3);

    data[4] = *vbuf;     data[5] = *(vbuf+1);
    data[6] = *(vbuf+2); data[7] = *(vbuf+3);

    return dm_can_send(hcan, motor_id + POS_MODE, data, 8);
}

/* ================================================================
   速度模式 (SPD_MODE)
   ================================================================ */

/**
************************************************************************
* @brief:      	dm_spd_ctrl: 速度控制
* @param[in]:   hcan:     指向 CAN_HandleTypeDef 结构的指针
* @param[in]:   motor_id: 电机 CAN ID
* @param[in]:   vel:      速度给定值 (float, rad/s)
* @retval:     	HAL 状态 (HAL_StatusTypeDef)
* @details:    	通过 CAN 总线向电机发送速度模式控制帧 (4 字节, 直接发送 float)
************************************************************************
**/
HAL_StatusTypeDef dm_spd_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float vel)
{
    uint8_t data[4];
    uint8_t *vbuf = (uint8_t*)&vel;

    data[0] = *vbuf;     data[1] = *(vbuf+1);
    data[2] = *(vbuf+2); data[3] = *(vbuf+3);

    return dm_can_send(hcan, motor_id + SPD_MODE, data, 4);
}

/* ================================================================
   位速流模式 (PSI_MODE)
   ================================================================ */

/**
************************************************************************
* @brief:      	dm_psi_ctrl: 位速流控制 (混控模式)
* @param[in]:   hcan:     指向 CAN_HandleTypeDef 结构的指针
* @param[in]:   motor_id: 电机 CAN ID
* @param[in]:   pos:      位置给定值 (float, rad)
* @param[in]:   vel:      速度给定值 (float, rad/s, ×100 后发 uint16)
* @param[in]:   cur:      电流给定值 (float, A, ×10000 后发 uint16)
* @retval:     	HAL 状态 (HAL_StatusTypeDef)
* @details:    	通过 CAN 总线向电机发送位速流模式控制帧 (8 字节)
************************************************************************
**/
HAL_StatusTypeDef dm_psi_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id,
                              float pos, float vel, float cur)
{
    uint8_t data[8];
    uint8_t *pbuf = (uint8_t*)&pos;
    uint16_t u16_vel = (uint16_t)(vel * 100);
    uint16_t u16_cur = (uint16_t)(cur * 10000);

    data[0] = *pbuf;     data[1] = *(pbuf+1);
    data[2] = *(pbuf+2); data[3] = *(pbuf+3);

    data[4] = (uint8_t)(u16_vel);
    data[5] = (uint8_t)(u16_vel >> 8);

    data[6] = (uint8_t)(u16_cur);
    data[7] = (uint8_t)(u16_cur >> 8);

    return dm_can_send(hcan, motor_id + PSI_MODE, data, 8);
}

/* ================================================================
   反馈解析
   ================================================================ */

/**
************************************************************************
* @brief:      	dm_fb_parse: 解析电机反馈数据 (MIT 模式)
* @param[in]:   motor:   指向 motor_t 结构的指针 (使用 motor->p_max/v_max/t_max)
* @param[in]:   rx_data: 指向 8 字节反馈数据的指针
* @retval:     	void
* @details:    	从 CAN 接收数据中提取电机 ID、状态、位置、速度、扭矩及温度
*               pos/vel/tor 使用 per-motor 的 PMAX/VMAX/TMAX 做范围映射
************************************************************************
**/
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
   CAN 接收回调
   ================================================================ */

/**
************************************************************************
* @brief:      	dm_rx_cbk: CAN 接收回调分发
* @param[in]:   motor_set: 电机数组基址 (dm_motor 或 &dm_motor[4])
* @param[in]:   rx_data:   指向 8 字节反馈数据的指针
* @retval:     	void
* @details:    	从反馈首字节提取电机 ID (bit3~0), 自动分发到 motor_set[id-1]
************************************************************************
**/
void dm_rx_cbk(motor_t *motor_set, uint8_t *rx_data)
{
    uint8_t id = rx_data[0] & 0x0F;

    if (id >= 1 && id <= 8)
        dm_fb_parse(&motor_set[id - 1], rx_data);
}

/* ================================================================
   MCP2515 总线封装
   ================================================================ */

/**
************************************************************************
* @brief:      	dm_send_cmd_mcp2515: 通过 MCP2515 发送电机命令帧 (内部函数)
* @param[in]:   hcan:  指向 MCP2515_HandleTypeDef 结构的指针
* @param[in]:   motor: 指向 motor_t 结构的指针
* @param[in]:   cmd:   命令字节
* @retval:     	HAL 状态 (HAL_StatusTypeDef)
* @details:    	命令帧使用基础 ID (不加模式偏移)
************************************************************************
**/
static HAL_StatusTypeDef dm_send_cmd_mcp2515(MCP2515_HandleTypeDef *hcan,
                                              motor_t *motor, uint8_t cmd)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd};
    mcp2515_can_tx_data(hcan, motor->id, data, 8);
    return HAL_OK;
}

/**
************************************************************************
* @brief:      	dm_enable_mcp2515: MCP2515 总线使能电机
************************************************************************
**/
HAL_StatusTypeDef dm_enable_mcp2515(MCP2515_HandleTypeDef *hcan, motor_t *motor)
{
    return dm_send_cmd_mcp2515(hcan, motor, DM_CMD_ENABLE);
}

/**
************************************************************************
* @brief:      	dm_disable_mcp2515: MCP2515 总线失能电机
************************************************************************
**/
HAL_StatusTypeDef dm_disable_mcp2515(MCP2515_HandleTypeDef *hcan, motor_t *motor)
{
    return dm_send_cmd_mcp2515(hcan, motor, DM_CMD_DISABLE);
}

/**
************************************************************************
* @brief:      	dm_pos_ctrl_mcp2515: MCP2515 总线位置-速度控制
************************************************************************
**/
HAL_StatusTypeDef dm_pos_ctrl_mcp2515(MCP2515_HandleTypeDef *hcan, uint16_t motor_id,
                                      float pos, float vel)
{
    uint8_t data[8];
    uint8_t *pbuf, *vbuf;

    pbuf = (uint8_t*)&pos;
    vbuf = (uint8_t*)&vel;

    data[0] = *pbuf;     data[1] = *(pbuf+1);
    data[2] = *(pbuf+2); data[3] = *(pbuf+3);

    data[4] = *vbuf;     data[5] = *(vbuf+1);
    data[6] = *(vbuf+2); data[7] = *(vbuf+3);

    mcp2515_can_tx_data(hcan, motor_id + POS_MODE, data, 8);
    return HAL_OK;
}

/**
************************************************************************
* @brief:      	dm_mit_ctrl_mcp2515: MCP2515 总线 MIT 模式控制
************************************************************************
**/
HAL_StatusTypeDef dm_mit_ctrl_mcp2515(MCP2515_HandleTypeDef *hcan, motor_t *motor,
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

    mcp2515_can_tx_data(hcan, motor->id + MIT_MODE, data, 8);
    return HAL_OK;
}
