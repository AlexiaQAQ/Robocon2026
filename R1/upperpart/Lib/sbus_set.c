#include "sbus_set.h"
#include <string.h>

static uint8_t sbus_dma_buf[SBUS_DMA_BUF_LEN];
static uint8_t sbus_frame[SBUS_FRAME_LEN];

static uint8_t frame_index = 0;
static uint16_t old_pos = 0;

SBUS_t sbus_ch;
volatile bool sbus_frame_ready = false;

static void sbus_unpack(uint8_t *buf);
static void sbus_feed_byte(uint8_t ch);

/************************************************
 * 初始化
 ************************************************/
void sbus_rx_init(void)
{
    HAL_UART_Receive_DMA(&sbus_uart,
                         sbus_dma_buf,
                         SBUS_DMA_BUF_LEN);
}

/************************************************
 * 周期调用
 * 建议1ms~5ms调用一次
 ************************************************/
void sbus_poll(void)
{
    uint16_t pos;

    pos = SBUS_DMA_BUF_LEN -
          __HAL_DMA_GET_COUNTER(sbus_uart.hdmarx);

    while(old_pos != pos)
    {
        sbus_feed_byte(sbus_dma_buf[old_pos]);

        old_pos++;

        if(old_pos >= SBUS_DMA_BUF_LEN)
            old_pos = 0;
    }
}

/************************************************
 * 自动找帧头
 ************************************************/
static void sbus_feed_byte(uint8_t ch)
{
    if(frame_index == 0)
    {
        if(ch != 0x0F)
            return;
    }

    sbus_frame[frame_index++] = ch;

    if(frame_index < SBUS_FRAME_LEN)
        return;

    frame_index = 0;

    if(sbus_frame[0] != 0x0F)
        return;

    sbus_unpack(sbus_frame);

    sbus_frame_ready = true;
}

/************************************************
 * SBUS解包
 ************************************************/
static void sbus_unpack(uint8_t *buf)
{
    sbus_ch.ch[0]  = ((buf[1]      | buf[2]  << 8)) & 0x07FF;
    sbus_ch.ch[1]  = ((buf[2] >> 3 | buf[3]  << 5)) & 0x07FF;
    sbus_ch.ch[2]  = ((buf[3] >> 6 | buf[4]  << 2 | buf[5] << 10)) & 0x07FF;
    sbus_ch.ch[3]  = ((buf[5] >> 1 | buf[6]  << 7)) & 0x07FF;
    sbus_ch.ch[4]  = ((buf[6] >> 4 | buf[7]  << 4)) & 0x07FF;
    sbus_ch.ch[5]  = ((buf[7] >> 7 | buf[8]  << 1 | buf[9] << 9)) & 0x07FF;
    sbus_ch.ch[6]  = ((buf[9] >> 2 | buf[10] << 6)) & 0x07FF;
    sbus_ch.ch[7]  = ((buf[10]>> 5 | buf[11] << 3)) & 0x07FF;

    sbus_ch.ch[8]  = ((buf[12]     | buf[13] << 8)) & 0x07FF;
    sbus_ch.ch[9]  = ((buf[13]>> 3 | buf[14] << 5)) & 0x07FF;
    sbus_ch.ch[10] = ((buf[14]>> 6 | buf[15] << 2 | buf[16] << 10)) & 0x07FF;
    sbus_ch.ch[11] = ((buf[16]>> 1 | buf[17] << 7)) & 0x07FF;
    sbus_ch.ch[12] = ((buf[17]>> 4 | buf[18] << 4)) & 0x07FF;
    sbus_ch.ch[13] = ((buf[18]>> 7 | buf[19] << 1 | buf[20] << 9)) & 0x07FF;
    sbus_ch.ch[14] = ((buf[20]>> 2 | buf[21] << 6)) & 0x07FF;
    sbus_ch.ch[15] = ((buf[21]>> 5 | buf[22] << 3)) & 0x07FF;

    sbus_ch.ch17 = (buf[23] >> 0) & 0x01;
    sbus_ch.ch18 = (buf[23] >> 1) & 0x01;

    sbus_ch.frame_lost = (buf[23] >> 2) & 0x01;
    sbus_ch.failsafe   = (buf[23] >> 3) & 0x01;
}

/************************************************
 * float映射
 ************************************************/
float Map(float val,
          float in_min,
          float in_max,
          float out_min,
          float out_max)
{
    float rec;

    rec = (val - in_min) *
          (out_max - out_min) /
          (in_max - in_min) +
          out_min;

    if(rec > out_max)
        rec = out_max;

    if(rec < out_min)
        rec = out_min;

    return rec;
}

/************************************************
 * int16映射
 ************************************************/
int16_t map(int16_t val,
            int16_t in_min,
            int16_t in_max,
            int16_t out_min,
            int16_t out_max)
{
    int32_t result;

    if(in_min == in_max)
        return out_min;

    result =
        ((int32_t)(val - in_min) *
        (out_max - out_min))
        / (in_max - in_min)
        + out_min;

    if(result > out_max)
        result = out_max;

    if(result < out_min)
        result = out_min;

    return (int16_t)result;
}
