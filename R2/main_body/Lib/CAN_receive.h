#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"
#include "motor_control.h"
#include "mcp2515.h"

//rm motor data
typedef struct
{
	
    uint16_t ecd;            //	当前位置
    int16_t  speed_rpm;			 //	当前速度 
    int16_t  given_current;	 //	真实电流
    uint8_t  temperate;			 // 温度
		int16_t  last_ecd;			 //	上一次位置
	
} motor_measure_t;

//motor data read
// 电机数据的读取
#define get_motor_measure(ptr, data)                                \
	{                                                                 \
		(ptr)->last_ecd = (ptr)->ecd;                                  	\
		(ptr)->ecd = (double)((data)[0] << 8 | (data)[1]);       \
		(ptr)->speed_rpm = (short int)((data)[2] << 8 | (data)[3]);     \
		(ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
		(ptr)->temperate = (data)[6];                                   \
	}

extern motor_measure_t      motor_chassis[4],can2_motor_chassis[4];

extern CAN_HandleTypeDef    hcan1;    //CAN串口1
extern CAN_HandleTypeDef    hcan2;    //CAN串口2

//	3508经典电流控制方式
// 	can1发送电流函数
// 	canid 电机控制id
// 	motor1-4 电流值
void CAN1_send_dat(int16_t canid,int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4);


//	3508经典电流控制方式
// 	can2发送电流函数
// 	canid 电机控制id
// 	motor1-4 电流值
void CAN2_send_dat(uint16_t canid,int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4);




void CAN3_send_dat_mcp2515(int16_t canid,int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4);

#endif






