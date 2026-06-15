#include "motor_control.h"

extern motor_t dm_motor[4];

void dm4310_fbdata(motor_t *motor, uint8_t *rx_data)
{
	motor->para.id = (rx_data[0])&0x0F;
	motor->para.state = (rx_data[0])>>4;
	motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16);
	motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12);
	motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12);
	motor->para.Tmos = (float)(rx_data[6]);
	motor->para.Tcoil = (float)(rx_data[7]);

	motor->para.angle_pos = rads_to_angle(motor->para.pos);
}

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

void can2_rx_callback(uint8_t *rx_data)
{
	switch(rx_data[0])
	{
		case 0x05:dm4310_fbdata(&dm_motor[0], rx_data);
			break;
		case 0x06:dm4310_fbdata(&dm_motor[1], rx_data);
			break;
		case 0x07:dm4310_fbdata(&dm_motor[2], rx_data);
			break;
		case 0x08:dm4310_fbdata(&dm_motor[3], rx_data);
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

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

//    if (rx_header.StdId >= 0x205 && rx_header.StdId <= 0x208)
//    {
//        uint8_t i = rx_header.StdId - 0x205;
//        get_motor_measure(&motor_chassis[i], rx_data);
//    }
//    else
//    {
//        can1_rx_callback(rx_data);
//    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);

//    if (rx_header.StdId >= 0x205 && rx_header.StdId <= 0x208)
//    {
//        uint8_t i = rx_header.StdId - 0x205;
//        get_motor_measure(&can2_motor_chassis[i], rx_data);
//    }
//    else
//    {
//        can2_rx_callback(rx_data);
//    }
}
