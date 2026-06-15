#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

#include "can.h"
#include "mcp2515.h"

#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPD_MODE			0x200
#define PSI_MODE		  	0x300


float angle_to_rads(int16_t angle);

int16_t rads_to_angle(float rads);

int float_to_uint(float x_float, float x_min, float x_max, int bits);

float uint_to_float(int x_int, float x_min, float x_max, int bits);


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


void dm_enable(CAN_HandleTypeDef* hcan, uint16_t motor_id);
void dm_enable_mcp2515(MCP2515_HandleTypeDef* hcan, uint16_t motor_id);


void dm_disable(CAN_HandleTypeDef* hcan, uint16_t motor_id);
void dm_disable_mcp2515(MCP2515_HandleTypeDef* hcan, uint16_t motor_id);


void dm_save_pos_zero(CAN_HandleTypeDef* hcan, uint16_t motor_id);


void dm_clear_err(CAN_HandleTypeDef* hcan, uint16_t motor_id);


void dm_mit_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
void dm_mit_ctrl_mcp2515(MCP2515_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);

void pos_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float pos, float vel);
void pos_ctrl_mcp2515(MCP2515_HandleTypeDef* hcan,uint16_t motor_id, float pos, float vel);

void can1_rx_callback(uint8_t *rx_data);
void can2_rx_callback(uint8_t *rx_data);
void can3_rx_callback(uint8_t *rx_data);
void can4_rx_callback(uint8_t *rx_data);
void can5_rx_callback(uint8_t *rx_data);


typedef struct
{
    uint16_t ecd;
    int16_t  speed_rpm;
    int16_t  given_current;
    uint8_t  temperate;
    int16_t  last_ecd;
} motor_measure_t;

#define get_motor_measure(ptr, data)                                \
	{                                                                 \
		(ptr)->last_ecd = (ptr)->ecd;                                  	\
		(ptr)->ecd = (double)((data)[0] << 8 | (data)[1]);       \
		(ptr)->speed_rpm = (int16_t)((data)[2] << 8 | (data)[3]);     \
		(ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
		(ptr)->temperate = (data)[6];                                   \
	}

extern motor_measure_t motor_chassis[4], can2_motor_chassis[4];

void CAN1_send_dat(int16_t canid, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN2_send_dat(uint16_t canid, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN3_send_dat_mcp2515(int16_t canid, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);


void yun_enable(CAN_HandleTypeDef* hcan ,uint16_t motor_id);
void yun_enable_mcp2515(MCP2515_HandleTypeDef* hcan ,uint16_t motor_id);


void yun_disable(CAN_HandleTypeDef* hcan ,uint16_t motor_id);
void yun_disable_mcp2515(MCP2515_HandleTypeDef* hcan ,uint16_t motor_id);


void yun_clearerro(CAN_HandleTypeDef* hcan ,uint16_t motor_id);
void yun_clearerro_mcp2515(MCP2515_HandleTypeDef* hcan ,uint16_t motor_id);


void yun_mit_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id,	float P , float V , float T, float Kp , float Kd);
void yun_mit_ctrl_mcp2515(MCP2515_HandleTypeDef* hcan, uint16_t motor_id,float P , float V , float T,float Kp , float Kd);

#endif

