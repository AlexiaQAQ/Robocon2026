#include "sbus_set.h"

unsigned char sbus_rx_buf[25] = {0};
SBUS_t sbus_ch;
volatile bool sbus_frame_ready = false;

void sbus_rx_init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&sbus_uart, sbus_rx_buf, 25);
}

/* UART 空闲中断回调 — SBUS 帧间有 ≥4ms 空闲, 自动对齐帧边界 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance != UART4) return;

	if(Size >= 25)
	{
		/* 帧头帧尾校验 */
		if(sbus_rx_buf[0] == 0x0F && sbus_rx_buf[24] == 0x00)
		{
			sbus_ch.ch[0] =((sbus_rx_buf[2]<<8)   + (sbus_rx_buf[1])) & 0x07ff;
			sbus_ch.ch[1] =((sbus_rx_buf[3]<<5)   + (sbus_rx_buf[2]>>3)) & 0x07ff;
			sbus_ch.ch[2] =((sbus_rx_buf[5]<<10)  + (sbus_rx_buf[4]<<2) + (sbus_rx_buf[3]>>6)) & 0x07ff;
			sbus_ch.ch[3] =((sbus_rx_buf[6]<<7)   + (sbus_rx_buf[5]>>1)) & 0x07ff;
			sbus_ch.ch[4] =((sbus_rx_buf[7]<<4)   + (sbus_rx_buf[6]>>4)) & 0x07ff;
			sbus_ch.ch[5] =((sbus_rx_buf[9]<<9)   + (sbus_rx_buf[8]<<1) + (sbus_rx_buf[7]>>7)) & 0x07ff;
			sbus_ch.ch[6] =((sbus_rx_buf[10]<<6)  + (sbus_rx_buf[9]>>2)) & 0x07ff;
			sbus_ch.ch[7] =((sbus_rx_buf[11]<<3)  + (sbus_rx_buf[10]>>5)) & 0x07ff;
			sbus_ch.ch[8] =((sbus_rx_buf[13]<<8)  + (sbus_rx_buf[12])) & 0x07ff;
			sbus_ch.ch[9] =((sbus_rx_buf[14]<<5)  + (sbus_rx_buf[13]>>3)) & 0x07ff;
			sbus_ch.ch[10]=((sbus_rx_buf[16]<<10) + (sbus_rx_buf[15]<<2) + (sbus_rx_buf[14]>>6)) & 0x07ff;
			sbus_ch.ch[11]=((sbus_rx_buf[17]<<7)  + (sbus_rx_buf[16]>>1)) & 0x07ff;
//			sbus_ch.ch[12]=((sbus_rx_buf[18]<<4)  + (sbus_rx_buf[17]>>4)) & 0x07ff;
//			sbus_ch.ch[13]=((sbus_rx_buf[20]<<9)  + (sbus_rx_buf[19]<<1) + (sbus_rx_buf[18]>>7)) & 0x07ff;
//			sbus_ch.ch[14]=((sbus_rx_buf[21]<<6)  + (sbus_rx_buf[20]>>2)) & 0x07ff;
//			sbus_ch.ch[15]=((sbus_rx_buf[22]<<3)  + (sbus_rx_buf[21]>>5)) & 0x07ff;

			sbus_frame_ready = true;
		}
	}

	/* 无论帧是否有效, 都重新启动接收, IDLE 保证下次从头对齐 */
	HAL_UARTEx_ReceiveToIdle_DMA(&sbus_uart, sbus_rx_buf, 25);
}

float Map(float val, float in_min, float in_max, float out_min, float out_max)
{
	float rec = (float)(val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	if(rec > out_max) rec = out_max;
	if(rec < out_min) rec = out_min;
	return rec;
}

int16_t map(int16_t val, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max)
{
	if(in_min == in_max) return out_min;
	int32_t result = ((int32_t)(val - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min;
	if(result > out_max) result = out_max;
	if(result < out_min) result = out_min;
	if(result > INT16_MAX) result = INT16_MAX;
	if(result < INT16_MIN) result = INT16_MIN;
	return (int16_t)result;
}
