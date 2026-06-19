/**
  ******************************************************************************
  * @file    tjc_screen.c
  * @brief   淘晶驰 (TJC) 串口屏通信驱动实现
  * @note    协议: ASCII命令 + \xFF\xFF\xFF 结尾（3字节，参考官方样例）
  *          写文本:  comp.txt="text"\xFF\xFF\xFF
  *          写数值:  comp.val=123\xFF\xFF\xFF
  *          写浮点:  comp.txt="15.0"\xFF\xFF\xFF
  *          波形加:  add comp.id,ch,val\xFF\xFF\xFF
  *          波形清:  cle comp.id,ch\xFF\xFF\xFF
  *          切页:    page page_id\xFF\xFF\xFF
  *          通过 fputc → USART1 发送，与调试 printf 共用
  ******************************************************************************
  */
#include "tjc_screen.h"
#include "usart.h"
#include <stdio.h>

/* TJC 命令尾（3字节 0xFF） */
#define TJC_TAIL  "\xFF\xFF\xFF"

/* ------------------------------------------------------------------ */
/* 底层：printf + 3字节尾                                             */
/* ------------------------------------------------------------------ */

/**
  * @brief  向控件写文本
  * @note   格式: comp.txt="text"\xFF\xFF\xFF
  *         例: TJC_Write_Text("t6", "15.0") → t6.txt="15.0"\xFF\xFF\xFF
  */
void TJC_Write_Text(const char *comp, const char *text)
{
  printf("%s.txt=\"%s\"" TJC_TAIL, comp, text);
}

/**
  * @brief  向控件写整数值
  * @note   格式: comp.val=123\xFF\xFF\xFF
  *         例: TJC_Write_Int("n0", 500) → n0.val=500\xFF\xFF\xFF
  */
void TJC_Write_Int(const char *comp, int val)
{
  printf("%s.val=%d" TJC_TAIL, comp, val);
}

/**
  * @brief  向控件写浮点数（文本方式）
  * @note   格式: comp.txt="15.0"\xFF\xFF\xFF
  *         分三次 printf 构造格式串，避免 sprintf 额外开销
  */
void TJC_Write_Float(const char *comp, float val, int decimals)
{
  char fmt[16];
  snprintf(fmt, sizeof(fmt), "%%.%df", decimals);
  printf("%s.txt=\"", comp);
  printf(fmt, val);
  printf("\"" TJC_TAIL);
}

/**
  * @brief  切换页面
  * @note   格式: page page_id\xFF\xFF\xFF
  */
void TJC_Set_Page(int page_id)
{
  printf("page %d" TJC_TAIL, page_id);
}

/* ------------------------------------------------------------------ */
/* 应用层快捷函数                                                      */
/* ------------------------------------------------------------------ */

/**
  * @brief  更新当前高度显示 (t6)，单位 cm
  */
void TJC_Update_Height(float height_cm)
{
  TJC_Write_Float(TJC_COMP_HEIGHT, height_cm, 1);
}

/**
  * @brief  更新目标高度显示 (t7)，单位 cm
  */
void TJC_Update_Target(float target_cm)
{
  TJC_Write_Float(TJC_COMP_TARGET, target_cm, 1);
}

/**
  * @brief  更新 PWM 占空比显示 (t8)
  * @param  duty_permil: 0~1000 对应 0.0%~100.0%
  */
void TJC_Update_PWM(int duty_permil)
{
  TJC_Write_Float(TJC_COMP_PWM, duty_permil, 1);
}

/* ------------------------------------------------------------------ */
/* 波形控件 (s0) 操作                                                 */
/* ------------------------------------------------------------------ */

/**
  * @brief  向波形控件 (s0) 添加数据点
  * @param  channel: 通道号 (0=高度曲线, 1=目标参考线)
  * @param  value:   数据值（映射到波形 Y 轴）
  * @note   格式: add s0.id,ch,val\xFF\xFF\xFF
  */
void TJC_Add_WaveData(int channel, int value)
{
  printf("add %s.id,%d,%d" TJC_TAIL, TJC_COMP_WAVE, channel, value);
}

/**
  * @brief  清除指定通道的波形数据
  * @note   格式: cle s0.id,ch\xFF\xFF\xFF
  */
void TJC_Clear_Wave(int channel)
{
  printf("cle %s.id,%d" TJC_TAIL, TJC_COMP_WAVE, channel);
}
