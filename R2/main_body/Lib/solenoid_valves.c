#include "solenoid_valves.h"

CAN_TxHeaderTypeDef  can_tx_message;
uint8_t              can_can_send_data[8]={0};


//	电磁阀刷新控制函数
//	hcan	选择can口

void YV_flash(CAN_HandleTypeDef *hcan)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = 0x300;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
		
	HAL_CAN_AddTxMessage(hcan, &can_tx_message, can_can_send_data, &send_mail_box);
}

void YV_flash_mcp2515(MCP2515_HandleTypeDef *hcan)
{
	mcp2515_can_tx_data(hcan, 0x300, can_can_send_data, 8);
}


