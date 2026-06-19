#include "int16_pid.h"
#include <stdint.h>

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

void PID_Init(PidTypeDef *pid, PID_MODE mode, const int32_t PID[3], int32_t max_out, int32_t max_iout)
{
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0;
    pid->q_shift = 8;   // [MOD] 默认缩放 8 位，对应 256 倍
}

int32_t PID_Calc(PidTypeDef *pid, int32_t ref, int32_t set)
{
    int32_t temp;   // [MOD] 临时变量存放乘积右移结果

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;

    if (pid->mode == PID_POSITION)
    {
        // P 项：Kp * error，右移 q_shift 位
        temp = pid->Kp * pid->error[0];
        pid->Pout = temp >> pid->q_shift;

        // I 项：Ki * error，右移后累加
        temp = pid->Ki * pid->error[0];
        pid->Iout += temp >> pid->q_shift;

        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);

        // D 项：Kd * Dbuf，右移
        temp = pid->Kd * pid->Dbuf[0];
        pid->Dout = temp >> pid->q_shift;

        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        // 增量式 P
        temp = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Pout = temp >> pid->q_shift;

        // 增量式 I
        temp = pid->Ki * pid->error[0];
        pid->Iout = temp >> pid->q_shift;

        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2 * pid->error[1] + pid->error[2]);

        // 增量式 D
        temp = pid->Kd * pid->Dbuf[0];
        pid->Dout = temp >> pid->q_shift;

        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

void PID_clear(PidTypeDef *pid)
{
    pid->error[0] = pid->error[1] = pid->error[2] = 0;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0;
    pid->fdb = pid->set = 0;
}

