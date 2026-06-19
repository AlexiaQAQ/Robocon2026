/**
  ******************************************************************************
  * @file    tjc_screen.h
  * @brief   淘晶驰 (TJC) 串口屏通信驱动
  * @note    协议: 命令字符串 + \xFF\xFF\xFF 结尾
  *          写文本: comp.txt="text"\xFF\xFF\xFF
  *          写数值: comp.val=123\xFF\xFF\xFF
  *          波形:   add comp.id,ch,val\xFF\xFF\xFF
  *          与调试 printf 共用 USART1 (PA9/PA10)
  ******************************************************************************
  */
#ifndef __TJC_SCREEN_H__
#define __TJC_SCREEN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* 控件 ID（与淘晶驰 UI 工程一致）------------------------------------------*/
#define TJC_COMP_HEIGHT     "t6"   /* 当前高度显示 */
#define TJC_COMP_TARGET     "t7"   /* 目标高度显示 */
#define TJC_COMP_PWM        "t8"   /* PWM 输出显示 */
#define TJC_COMP_WAVE       "s0"   /* 高度曲线 */
/* 波形通道 */
#define TJC_WAVE_CH_HEIGHT  0      /* 当前高度曲线通道 */
#define TJC_WAVE_CH_TARGET  1      /* 目标高度参考线通道 */

/* 运行状态文本 -------------------------------------------------------------*/
#define TJC_STATUS_STOP     "停止"
#define TJC_STATUS_RUNNING  "运行中"
#define TJC_STATUS_ADJUST   "调节中"
#define TJC_STATUS_STABLE   "已稳定"

/* 公开函数声明 -------------------------------------------------------------*/

/**
  * @brief  向控件写文本
  *         格式: comp.txt="text"\xFF\xFF\xFF
  */
void TJC_Write_Text(const char *comp, const char *text);

/**
  * @brief  向控件写整数值
  *         格式: comp.val=123\xFF\xFF\xFF
  */
void TJC_Write_Int(const char *comp, int val);

/**
  * @brief  向控件写浮点数（文本方式）
  *         格式: comp.txt="15.0"\xFF\xFF\xFF
  */
void TJC_Write_Float(const char *comp, float val, int decimals);

/**
  * @brief  切换页面
  *         格式: page page_id\xFF\xFF\xFF
  */
void TJC_Set_Page(int page_id);

/* ---- 应用层快捷函数 ---- */

/** 更新当前高度显示 (t6)，单位 cm */
void TJC_Update_Height(float height_cm);

/** 更新目标高度显示 (t7)，单位 cm */
void TJC_Update_Target(float target_cm);

/** 更新 PWM 占空比显示 (t8)，0~1000（对应 0%~100%）*/
void TJC_Update_PWM(int duty_permil);

/** 向波形控件 (s0) 添加数据点 */
void TJC_Add_WaveData(int channel, int value);

/** 清除指定通道的波形 */
void TJC_Clear_Wave(int channel);

#ifdef __cplusplus
}
#endif

#endif /* __TJC_SCREEN_H__ */
