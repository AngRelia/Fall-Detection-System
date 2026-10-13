#ifndef __LED_H
#define __LED_H

#include "main.h"

/*
 * TIM3 CH2/CH3/CH4 三路PWM亮度驱动
 * 建议引脚：PA7/PB0/PB1
 */

typedef enum
{
    LED_CH1 = 0,   /* TIM3_CH2 -> PA7 */
    LED_CH2,       /* TIM3_CH3 -> PB0 */
    LED_CH3,       /* TIM3_CH4 -> PB1 */
    LED_CH_MAX
} LED_Channel_t;

/* 初始化（传入已由 CubeMX 生成的 TIM3 句柄，如 &htim3） */
HAL_StatusTypeDef LED_PWM_Init(TIM_HandleTypeDef *htim_pwm);

/* 启停三路PWM */
HAL_StatusTypeDef LED_PWM_Start(void);
HAL_StatusTypeDef LED_PWM_Stop(void);

/* 单路亮度：0~100(%) */
void LED_SetBrightnessPercent(LED_Channel_t ch, uint8_t percent);

/* 直接设置比较值（0~ARR） */
void LED_SetCompare(LED_Channel_t ch, uint16_t compare);

/* 三路一起设置（常用于RGB） */
void LED_SetAllPercent(uint8_t ch1_percent, uint8_t ch2_percent, uint8_t ch3_percent);

#endif
