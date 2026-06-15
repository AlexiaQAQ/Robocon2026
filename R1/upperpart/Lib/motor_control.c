#include "motor_control.h"

motor_t dm_motor[4];	// DM电机数组
motor_measure_t motor_chassis[4], can2_motor_chassis[4];

static CAN_TxHeaderTypeDef  can1_tx_message;
static uint8_t              can1_can_send_data[8];
static CAN_TxHeaderTypeDef  can2_tx_message;
static uint8_t              can2_can_send_data[8];
static uint8_t              can3_can_send_data[8];

float angle_to_rads(int16_t angle)
{
	float rads=0;
	rads = (float)(angle * 0.017453292f);
	return (float)rads;
}


int16_t rads_to_angle(float rads)
{
	int16_t angle=0;
	angle = (float)(rads * 57.29578f);
	return (int16_t)angle;
}


uint16_t mabs(int16_t t)
{
	if(t>0){return t;}
	else{return -t;}
}


int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}


uint8_t canx_bsp_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{	
	uint32_t send_mail_box;
	
	CAN_TxHeaderTypeDef	tx_header;
	
	tx_header.StdId = id;
	tx_header.ExtId = 0;
	tx_header.IDE   = 0;
	tx_header.RTR   = 0;
	tx_header.DLC   = len;
	
	
	HAL_CAN_AddTxMessage(hcan, &tx_header, data,  &send_mail_box);
	
  return 0;
}

void dm_enable(CAN_HandleTypeDef* hcan, uint16_t motor_id)
{
	uint8_t data[8];
	uint16_t id = motor_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
	
	canx_bsp_send_data(hcan, id, data, 8);
}

void dm_enable_mcp2515(MCP2515_HandleTypeDef* hcan, uint16_t motor_id)
{
	uint8_t data[8];
	uint16_t id = motor_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
	
	mcp2515_can_tx_data(hcan, id, data, 8);
}

void dm_disable(CAN_HandleTypeDef* hcan, uint16_t motor_id)
{
	uint8_t data[8];
	uint16_t id = motor_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;
	
	canx_bsp_send_data(hcan, id, data, 8);
}

void dm_disable_mcp2515(MCP2515_HandleTypeDef* hcan, uint16_t motor_id)
{
	uint8_t data[8];
	uint16_t id = motor_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;
	
	mcp2515_can_tx_data(hcan, id, data, 8);
}

void dm_save_pos_zero(CAN_HandleTypeDef* hcan, uint16_t motor_id)
{
	uint8_t data[8];
	uint16_t id = motor_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFE;
	
	canx_bsp_send_data(hcan, id, data, 8);
}

void dm_clear_err(CAN_HandleTypeDef* hcan, uint16_t motor_id)
{
	uint8_t data[8];
	uint16_t id = motor_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFB;
	
	canx_bsp_send_data(hcan, id, data, 8);
}

void dm_mit_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id;

	pos_tmp = float_to_uint(pos,  P_MIN,  P_MAX,  16);
	vel_tmp = float_to_uint(vel,  V_MIN,  V_MAX,  12);
	kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(torq, T_MIN,  T_MAX,  12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	canx_bsp_send_data(hcan, id, data, 8);
}

void dm_mit_ctrl_mcp2515(MCP2515_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id;

	pos_tmp = float_to_uint(pos,  P_MIN,  P_MAX,  16);
	vel_tmp = float_to_uint(vel,  V_MIN,  V_MAX,  12);
	kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(torq, T_MIN,  T_MAX,  12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	mcp2515_can_tx_data(hcan, id, data, 8);
}

void pos_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float pos, float vel)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf;
	uint8_t data[8];
	
	id = motor_id + POS_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	data[6] = *(vbuf+2);
	data[7] = *(vbuf+3);
	
	canx_bsp_send_data(hcan, id, data, 8);
}

void pos_ctrl_mcp2515(MCP2515_HandleTypeDef* hcan,uint16_t motor_id, float pos, float vel)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf;
	uint8_t data[8];
	
	id = motor_id + POS_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	data[6] = *(vbuf+2);
	data[7] = *(vbuf+3);
	
	mcp2515_can_tx_data(hcan, id, data, 8);
}

void spd_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float vel)
{
	uint16_t id;
	uint8_t *vbuf;
	uint8_t data[4];
	
	id = motor_id + SPD_MODE;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *vbuf;
	data[1] = *(vbuf+1);
	data[2] = *(vbuf+2);
	data[3] = *(vbuf+3);
	
	canx_bsp_send_data(hcan, id, data, 4);
}

void psi_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel, float cur)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf, *ibuf;
	uint8_t data[8];
	
	uint16_t u16_vel = vel*100;
	uint16_t u16_cur  = cur*10000;
	
	id = motor_id + PSI_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&u16_vel;
	ibuf=(uint8_t*)&u16_cur;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	
	data[6] = *ibuf;
	data[7] = *(ibuf+1);
	
	canx_bsp_send_data(hcan, id, data, 8);
}

void CAN1_send_dat(int16_t canid, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    can1_tx_message.StdId = canid;
    can1_tx_message.IDE = CAN_ID_STD;
    can1_tx_message.RTR = CAN_RTR_DATA;
    can1_tx_message.DLC = 0x08;
    can1_can_send_data[0] = motor1 >> 8;
    can1_can_send_data[1] = motor1;
    can1_can_send_data[2] = motor2 >> 8;
    can1_can_send_data[3] = motor2;
    can1_can_send_data[4] = motor3 >> 8;
    can1_can_send_data[5] = motor3;
    can1_can_send_data[6] = motor4 >> 8;
    can1_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &can1_tx_message, can1_can_send_data, &send_mail_box);
}

void CAN2_send_dat(uint16_t canid, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    can2_tx_message.StdId = canid;
    can2_tx_message.IDE = CAN_ID_STD;
    can2_tx_message.RTR = CAN_RTR_DATA;
    can2_tx_message.DLC = 0x08;
    can2_can_send_data[0] = motor1 >> 8;
    can2_can_send_data[1] = motor1;
    can2_can_send_data[2] = motor2 >> 8;
    can2_can_send_data[3] = motor2;
    can2_can_send_data[4] = motor3 >> 8;
    can2_can_send_data[5] = motor3;
    can2_can_send_data[6] = motor4 >> 8;
    can2_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan2, &can2_tx_message, can2_can_send_data, &send_mail_box);
}

void CAN3_send_dat_mcp2515(int16_t canid, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    can3_can_send_data[0] = motor1 >> 8;
    can3_can_send_data[1] = motor1;
    can3_can_send_data[2] = motor2 >> 8;
    can3_can_send_data[3] = motor2;
    can3_can_send_data[4] = motor3 >> 8;
    can3_can_send_data[5] = motor3;
    can3_can_send_data[6] = motor4 >> 8;
    can3_can_send_data[7] = motor4;

    mcp2515_can_tx_data(&hcan3, canid, can3_can_send_data, 8);
}

void yun_enable(CAN_HandleTypeDef* hcan ,uint16_t motor_id)
{
	uint8_t data[8];
	uint16_t id;
	
	id = (motor_id&0x000f)|0x40;
	
	canx_bsp_send_data(hcan,id,data,0);

}

void yun_enable_mcp2515(MCP2515_HandleTypeDef* hcan ,uint16_t motor_id)
{
	uint8_t data[8];
	uint16_t id;
	
	id = (motor_id&0x000f)|0x40;
	
	mcp2515_can_tx_data(hcan, id, data, 0);

}

void yun_disable(CAN_HandleTypeDef* hcan ,uint16_t motor_id)
{
	uint8_t data[8];
	uint16_t id;
	
	id = (motor_id&0x000f)|0x20;
	
	canx_bsp_send_data(hcan,id,data,0);
}

void yun_disable_mcp2515(MCP2515_HandleTypeDef* hcan ,uint16_t motor_id)
{
	uint8_t data[8];
	uint16_t id;
	
	id = (motor_id&0x000f)|0x20;
	
	mcp2515_can_tx_data(hcan,id,data,0);
}


void yun_clearerro(CAN_HandleTypeDef* hcan ,uint16_t motor_id)
{
	uint8_t data[8];
	uint16_t id;
	
	id = (motor_id&0x000f)|0x02e0;
	
	canx_bsp_send_data(hcan,id,data,0);
}

void yun_clearerro_mcp2515(MCP2515_HandleTypeDef* hcan ,uint16_t motor_id)
{
	uint8_t data[8];
	uint16_t id;
	
	id = (motor_id&0x000f)|0x02e0;
	
	mcp2515_can_tx_data(hcan, id, data, 0);
}

void yun_mit_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id,float P , float V , float T,float Kp , float Kd)
{
	uint8_t data[8];
	uint16_t id;
	
	id = (motor_id&0x000f)|0x80;
	
	uint16_t P_t = 0;
	uint16_t V_t = 0;
	uint16_t T_t = 0;
	uint16_t Kp_t = 0;
	uint16_t Kd_t = 0;
	
	P_t = float_to_uint(P,    -40.0f,  40.0f,    16);
	V_t = float_to_uint(V,    -40.0f,  40.0f,    14);
	Kp_t  = float_to_uint(Kp,   0.0f,  1023.0f,  10);
	Kd_t  = float_to_uint(Kd,   0.0f,  51.0f,     8);
	T_t = float_to_uint(T,    -40.0f,  40.0f,    16);
	
	data[0] = P_t;
	data[1] = P_t >> 8;
	data[2] = V_t;
	data[3] = ((V_t >> 8) & 0x3f)| ((Kp_t & 0x03) << 6);
	data[4] = Kp_t >> 2;
	data[5] = Kd_t;
	data[6] = T_t;
	data[7] = T_t >> 8;
	
	canx_bsp_send_data(hcan,id,data,8);
}

void yun_mit_ctrl_mcp2515(MCP2515_HandleTypeDef* hcan, uint16_t motor_id,float P , float V , float T,float Kp , float Kd)
{
	uint8_t data[8];
	uint16_t id;
	
	id = (motor_id&0x000f)|0x80;
	
	uint16_t P_t = 0;
	uint16_t V_t = 0;
	uint16_t T_t = 0;
	uint16_t Kp_t = 0;
	uint16_t Kd_t = 0;
	
	P_t = float_to_uint(P,    -40.0f,  40.0f,    16);
	V_t = float_to_uint(V,    -40.0f,  40.0f,    14);
	Kp_t  = float_to_uint(Kp,   0.0f,  1023.0f,  10);
	Kd_t  = float_to_uint(Kd,   0.0f,  51.0f,     8);
	T_t = float_to_uint(T,    -40.0f,  40.0f,    16);
	
	data[0] = P_t;
	data[1] = P_t >> 8;
	data[2] = V_t;
	data[3] = ((V_t >> 8) & 0x3f)| ((Kp_t & 0x03) << 6);
	data[4] = Kp_t >> 2;
	data[5] = Kd_t;
	data[6] = T_t;
	data[7] = T_t >> 8;
	
	mcp2515_can_tx_data(hcan,id,data,8);
}
