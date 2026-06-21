#include "sbus_set.h"

unsigned char sbus_rx_buf[25] = {0};
static unsigned char sbus_last_buf[25] = {0};  /* 用于检测帧更新 */
SBUS_t sbus_ch;
volatile bool sbus_frame_ready = false;

void sbus_rx_init(void)
{
    HAL_UART_Receive_DMA(&sbus_uart, sbus_rx_buf, 25);
}

/* 周期调用, 1~5ms 一次 */
void sbus_poll(void)
{
    uint8_t cnt;

    if(sbus_rx_buf[0] == 0x0F && sbus_rx_buf[24] == 0x00)
    {
        /* 帧内容未变 → DMA已停, 陈帧不处理 */
        uint8_t i, same = 1;
        for(i = 0; i < 25; i++) { if(sbus_rx_buf[i] != sbus_last_buf[i]) { same = 0; break; } }
        if(same) return;
        for(i = 0; i < 25; i++) sbus_last_buf[i] = sbus_rx_buf[i];

        sbus_ch.ch[0]  = ((sbus_rx_buf[1]     | sbus_rx_buf[2] << 8) & 0x07FF);
        sbus_ch.ch[1]  = ((sbus_rx_buf[2] >>3 | sbus_rx_buf[3] << 5) & 0x07FF);
        sbus_ch.ch[2]  = ((sbus_rx_buf[3] >>6 | sbus_rx_buf[4] << 2 | sbus_rx_buf[5] <<10) & 0x07FF);
        sbus_ch.ch[3]  = ((sbus_rx_buf[5] >>1 | sbus_rx_buf[6] << 7) & 0x07FF);
        sbus_ch.ch[4]  = ((sbus_rx_buf[6] >>4 | sbus_rx_buf[7] << 4) & 0x07FF);
        sbus_ch.ch[5]  = ((sbus_rx_buf[7] >>7 | sbus_rx_buf[8] << 1 | sbus_rx_buf[9] << 9) & 0x07FF);
        sbus_ch.ch[6]  = ((sbus_rx_buf[9] >>2 | sbus_rx_buf[10]<< 6) & 0x07FF);
        sbus_ch.ch[7]  = ((sbus_rx_buf[10]>>5 | sbus_rx_buf[11]<< 3) & 0x07FF);
        sbus_ch.ch[8]  = ((sbus_rx_buf[12]    | sbus_rx_buf[13]<< 8) & 0x07FF);
        sbus_ch.ch[9]  = ((sbus_rx_buf[13]>>3 | sbus_rx_buf[14]<< 5) & 0x07FF);
        sbus_ch.ch[10] = ((sbus_rx_buf[14]>>6 | sbus_rx_buf[15]<< 2 | sbus_rx_buf[16]<<10) & 0x07FF);
        sbus_ch.ch[11] = ((sbus_rx_buf[16]>>1 | sbus_rx_buf[17]<< 7) & 0x07FF);
        sbus_ch.ch[12] = ((sbus_rx_buf[17]>>4 | sbus_rx_buf[18]<< 4) & 0x07FF);
        sbus_ch.ch[13] = ((sbus_rx_buf[18]>>7 | sbus_rx_buf[19]<< 1 | sbus_rx_buf[20]<< 9) & 0x07FF);
        sbus_ch.ch[14] = ((sbus_rx_buf[20]>>2 | sbus_rx_buf[21]<< 6) & 0x07FF);
        sbus_ch.ch[15] = ((sbus_rx_buf[21]>>5 | sbus_rx_buf[22]<< 3) & 0x07FF);

        sbus_ch.frame_lost = (sbus_rx_buf[23] >> 2) & 0x01;
        sbus_ch.failsafe   = (sbus_rx_buf[23] >> 3) & 0x01;

        sbus_frame_ready = true;
    }
    else
    {
        /* 坏帧: 清缓冲, 重启 DMA */
        HAL_UART_DMAStop(&sbus_uart);
        for(cnt = 0; cnt < 25; cnt++) sbus_rx_buf[cnt] = 0;
        for(cnt = 0; cnt < 16; cnt++) sbus_ch.ch[cnt] = 0;
        HAL_UART_Receive_DMA(&sbus_uart, sbus_rx_buf, 25);
    }
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
