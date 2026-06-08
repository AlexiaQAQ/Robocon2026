#include "motor_control.h"

motor_t dm_motor[4];	//达秒电机初始化


//	角度转弧度
float angle_to_rads(int16_t angle)
{
	float rads=0;
	rads = (float)(angle * 0.017453292f);
	return (float)rads;
}


//	弧度转角度
int16_t rads_to_angle(float rads)
{
	int16_t angle=0;
	angle = (float)(rads * 57.29578f);
	return (int16_t)angle;
}

//	绝对值函数
uint16_t mabs(int16_t t)
{
	if(t>0){return t;}
	else{return -t;}
}


/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}



//	电机发送套皮
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
	
  /*找到空的发送邮箱，把数据发送出去*/
//	if(HAL_CAN_AddTxMessage(hcan, &tx_header, data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) {
//		if(HAL_CAN_AddTxMessage(hcan, &tx_header, data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK) {
//			
//    }
//  }
  return 0;
}

//	达妙电机使能
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

//	达妙电机失能
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

//	达秒电机设置零点
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

//	达秒电机清除错误
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


/**
************************************************************************
* @brief:      	mit_ctrl: MIT模式下的电机控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   torq:			转矩给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
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

/**
************************************************************************
* @brief:      	pos_speed_ctrl: 位置速度控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   vel:			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送位置速度控制命令
************************************************************************
**/
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

/**
************************************************************************
* @brief:      	speed_ctrl: 速度控制函数
* @param[in]:   hcan: 		指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   vel: 			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送速度控制命令
************************************************************************
**/
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


/**
************************************************************************
* @brief:      	pos_speed_ctrl: 混控模式
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   i:				电流给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送位置速度控制命令
************************************************************************
**/
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

/**
************************************************************************
* @brief:      	dm4310_fbdata: 获取DM4310电机反馈数据函数
* @param[in]:   motor:    指向motor_t结构的指针，包含电机相关信息和反馈数据
* @param[in]:   rx_data:  指向包含反馈数据的数组指针
* @retval:     	void
* @details:    	从接收到的数据中提取DM4310电机的反馈信息，包括电机ID、
*               状态、位置、速度、扭矩以及相关温度参数
************************************************************************
**/
void dm4310_fbdata(motor_t *motor, uint8_t *rx_data)
{
	motor->para.id = (rx_data[0])&0x0F;
	motor->para.state = (rx_data[0])>>4;
	motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
	motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
	motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12);  // (-18.0,18.0)
	motor->para.Tmos = (float)(rx_data[6]);
	motor->para.Tcoil = (float)(rx_data[7]);
	
	motor->para.angle_pos = rads_to_angle(motor->para.pos);
}


/**
************************************************************************
* @brief:      	can1_rx_callback: CAN1接收回调函数
* @param:      	void
* @retval:     	void
* @details:    	处理CAN1接收中断回调，根据接收到的ID和数据，执行相应的处理。
*               当接收到ID为0时，调用dm4310_fbdata函数更新Motor的反馈数据。
************************************************************************
**/
void can1_rx_callback(uint8_t *rx_data)
{
	switch(rx_data[0])
	{
		case 0x01:dm4310_fbdata(&dm_motor[0], rx_data);
			break;
		case 0x02:dm4310_fbdata(&dm_motor[1], rx_data);
			break;
		case 0x03:dm4310_fbdata(&dm_motor[2], rx_data);
			break;
		case 0x04:dm4310_fbdata(&dm_motor[3], rx_data);
			break;
	}
}

//	多路can如下放在各自的回调函数中
void can2_rx_callback(uint8_t *rx_data)
{
	switch(rx_data[0])
	{
		case 0x01:dm4310_fbdata(&dm_motor[0], rx_data);
			break;
		case 0x02:dm4310_fbdata(&dm_motor[1], rx_data);
			break;
		case 0x03:dm4310_fbdata(&dm_motor[2], rx_data);
			break;
		case 0x04:dm4310_fbdata(&dm_motor[3], rx_data);
			break;
	}
}

void can3_rx_callback(uint8_t *rx_data)
{
	switch(rx_data[0])
	{
		case 0x01:dm4310_fbdata(&dm_motor[0], rx_data);
			break;
		case 0x02:dm4310_fbdata(&dm_motor[1], rx_data);
			break;
		case 0x03:dm4310_fbdata(&dm_motor[2], rx_data);
			break;
		case 0x04:dm4310_fbdata(&dm_motor[3], rx_data);
			break;
	}
}

void can4_rx_callback(uint8_t *rx_data)
{
	switch(rx_data[0])
	{
		case 0x01:dm4310_fbdata(&dm_motor[0], rx_data);
			break;
		case 0x02:dm4310_fbdata(&dm_motor[1], rx_data);
			break;
		case 0x03:dm4310_fbdata(&dm_motor[2], rx_data);
			break;
		case 0x04:dm4310_fbdata(&dm_motor[3], rx_data);
			break;
	}
}

void can5_rx_callback(uint8_t *rx_data)
{
	switch(rx_data[0])
	{
		case 0x01:dm4310_fbdata(&dm_motor[0], rx_data);
			break;
		case 0x02:dm4310_fbdata(&dm_motor[1], rx_data);
			break;
		case 0x03:dm4310_fbdata(&dm_motor[2], rx_data);
			break;
		case 0x04:dm4310_fbdata(&dm_motor[3], rx_data);
			break;
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////



// 云深处电机使能
void yun_enable(CAN_HandleTypeDef* hcan ,uint16_t motor_id)
{
	uint8_t data[8];
	uint16_t id;
	
	id = (motor_id&0x000f)|0x40;
	
	canx_bsp_send_data(hcan,id,data,0);

}

// 云深处电机使能
void yun_enable_mcp2515(MCP2515_HandleTypeDef* hcan ,uint16_t motor_id)
{
	uint8_t data[8];
	uint16_t id;
	
	id = (motor_id&0x000f)|0x40;
	
	mcp2515_can_tx_data(hcan, id, data, 0);

}

// 云深处电机失能
void yun_disable(CAN_HandleTypeDef* hcan ,uint16_t motor_id)
{
	uint8_t data[8];
	uint16_t id;
	
	id = (motor_id&0x000f)|0x20;
	
	canx_bsp_send_data(hcan,id,data,0);
}

// 云深处电机失能
void yun_disable_mcp2515(MCP2515_HandleTypeDef* hcan ,uint16_t motor_id)
{
	uint8_t data[8];
	uint16_t id;
	
	id = (motor_id&0x000f)|0x20;
	
	mcp2515_can_tx_data(hcan,id,data,0);
}


// 云深处电机清除错误
void yun_clearerro(CAN_HandleTypeDef* hcan ,uint16_t motor_id)
{
	uint8_t data[8];
	uint16_t id;
	
	id = (motor_id&0x000f)|0x02e0;
	
	canx_bsp_send_data(hcan,id,data,0);
}

// 云深处电机清除错误
void yun_clearerro_mcp2515(MCP2515_HandleTypeDef* hcan ,uint16_t motor_id)
{
	uint8_t data[8];
	uint16_t id;
	
	id = (motor_id&0x000f)|0x02e0;
	
	mcp2515_can_tx_data(hcan, id, data, 0);
}


//	P 角度     [-40rad , 40rad] 		map [0 , 65535]
//	V 角速度	 	 [-40rad/s , 40rad/s] map [0 , 16384]
//	T 扭矩		 [-40N.m , 40N.m] 		map [0 , 65535]
//	Kp 刚度		 [0 , 1023] 					map [0 , 1023]
//	Kd 阻尼		 [0.0 , 51.0] 				map [0 , 255]

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
