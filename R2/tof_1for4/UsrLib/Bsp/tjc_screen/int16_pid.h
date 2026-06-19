#ifndef _INT16_PID_H_
#define _INT16_PID_H_

#include "math.h"
#include <stdint.h>

typedef enum 
{
    PID_POSITION = 0,
    PID_DELTA
}PID_MODE;

typedef struct
{
    unsigned char mode;
    //PID 三参数 —— 现在它们代表 Q8 定点数（真实值 = 参数 / 256）
    int32_t Kp;
    int32_t Ki;
    int32_t Kd;

    int32_t max_out;  //最大输出（原始量纲，不缩放）
    int32_t max_iout; //最大积分输出（原始量纲，不缩放）

    int32_t set;
    int32_t fdb;

    int32_t out;
    int32_t Pout;
    int32_t Iout;
    int32_t Dout;
    int32_t Dbuf[3];  //微分项 0最新 1上一次 2上上次
    int32_t error[3]; //误差项 0最新 1上一次 2上上次

    uint8_t q_shift; // [MOD] 新增：定点数右移位数，默认 8 (即除以 256)

} PidTypeDef;


void PID_Init(PidTypeDef *pid, PID_MODE mode, const int32_t PID[3], int32_t max_out, int32_t max_iout);
int32_t PID_Calc(PidTypeDef *pid, int32_t ref, int32_t set);
void PID_clear(PidTypeDef *pid);

#endif

