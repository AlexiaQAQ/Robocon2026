#include "CAN_receive.h" 

motor_measure_t      motor_chassis[4],can2_motor_chassis[4];

static CAN_TxHeaderTypeDef  can1_tx_message;
static uint8_t              can1_can_send_data[8];
static CAN_TxHeaderTypeDef  can2_tx_message;
static uint8_t              can2_can_send_data[8];


//	CAN1中断回调函数
//	CAN1配置成fifo0
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
			
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

	//dm_rx_cbk(dm_motor, rx_data);
}


//	CAN2中断回调函数
//	CAN2配置成fifo1
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
				
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);
    
	//dm_rx_cbk(&dm_motor[4], rx_data);
}


//	3508经典电流控制方式
// 	can1发送电流函数
// 	canid 电机控制id
// 	motor1-4 电流值
void CAN1_send_dat(int16_t canid,int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4)
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

//3508接收数据回调函数
void DJI3508_rx_callback(CAN_RxHeaderTypeDef rx_header, uint8_t rx_data[8])
{
	switch (rx_header.StdId)
	{
			case 0x205:
			case 0x206:
			case 0x207:
			case 0x208://1拖4
			{
				static uint8_t i = 0;
				//get motor id
				i = rx_header.StdId - 0x205;
				get_motor_measure(&motor_chassis[i], rx_data);
				break;
			}
	}
}


//	3508经典电流控制方式
// 	can2发送电流函数
// 	canid 电机控制id
// 	motor1-4 电流值
void CAN2_send_dat(uint16_t canid,int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4)
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

