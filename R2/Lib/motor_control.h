#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

#include "can.h"
#include "mcp2515.h"

#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPD_MODE			0x200
#define PSI_MODE		  	0x300

//	公共函数		///////////////////////////////////////////////////////////////////////////////////////////////////////

//	角度转弧度
float angle_to_rads(int16_t angle);

//	弧度转角度
int16_t rads_to_angle(float rads);

int float_to_uint(float x_float, float x_min, float x_max, int bits);

float uint_to_float(int x_int, float x_min, float x_max, int bits);



//	达秒电机			//////////////////////////////////////////////////////////////////////////////////////////////////////

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

// 电机回传信息结构体
typedef struct 
{
	int id;
	int state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;
	int16_t angle_pos;
	float vel;
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
}motor_fbpara_t;

// 电机参数设置结构体
typedef struct 
{
	int8_t mode;
	float pos_set;
	float vel_set;
	float tor_set;
	float kp_set;
	float kd_set;
}motor_ctrl_t;

typedef struct
{
	int8_t id;
	uint8_t start_flag;
	motor_fbpara_t para;
	motor_ctrl_t ctrl;
}motor_t;

extern motor_t dm_motor[4];

//	达妙电机使能
void dm_enable(CAN_HandleTypeDef* hcan, uint16_t motor_id);
void dm_enable_mcp2515(MCP2515_HandleTypeDef* hcan, uint16_t motor_id);

//	达妙电机失能
void dm_disable(CAN_HandleTypeDef* hcan, uint16_t motor_id);
void dm_disable_mcp2515(MCP2515_HandleTypeDef* hcan, uint16_t motor_id);

//	达秒电机设置零点
void dm_save_pos_zero(CAN_HandleTypeDef* hcan, uint16_t motor_id);

//	达秒电机清除错误
void dm_clear_err(CAN_HandleTypeDef* hcan, uint16_t motor_id);

//	达秒电机mit控制
void dm_mit_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
void dm_mit_ctrl_mcp2515(MCP2515_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);

void pos_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float pos, float vel);
void pos_ctrl_mcp2515(MCP2515_HandleTypeDef* hcan,uint16_t motor_id, float pos, float vel);

//	达秒电机回调函数放在HAL_CAN_RxFifo0MsgPendingCallback屁股后面
void can1_rx_callback(uint8_t *rx_data);
void can2_rx_callback(uint8_t *rx_data);
void can3_rx_callback(uint8_t *rx_data);
void can4_rx_callback(uint8_t *rx_data);
void can5_rx_callback(uint8_t *rx_data);



//	云深处电机		//////////////////////////////////////////////////////////////////////////////////////////////////////


// 云深处电机使能
void yun_enable(CAN_HandleTypeDef* hcan ,uint16_t motor_id);
void yun_enable_mcp2515(MCP2515_HandleTypeDef* hcan ,uint16_t motor_id);

// 云深处电机失能
void yun_disable(CAN_HandleTypeDef* hcan ,uint16_t motor_id);
void yun_disable_mcp2515(MCP2515_HandleTypeDef* hcan ,uint16_t motor_id);

// 云深处电机清除错误
void yun_clearerro(CAN_HandleTypeDef* hcan ,uint16_t motor_id);
void yun_clearerro_mcp2515(MCP2515_HandleTypeDef* hcan ,uint16_t motor_id);

//	云深处电机mit控制
void yun_mit_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id,	float P , float V , float T, float Kp , float Kd);
void yun_mit_ctrl_mcp2515(MCP2515_HandleTypeDef* hcan, uint16_t motor_id,float P , float V , float T,float Kp , float Kd);

#endif

