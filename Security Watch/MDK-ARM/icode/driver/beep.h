#ifndef __BEEP_H__
#define __BEEP_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 低电平触发蜂鸣器：
 * 0 -> 响
 * 1 -> 不响
 */

#if !defined(BEEP_Pin) || !defined(BEEP_GPIO_Port)
#error "BEEP_Pin / BEEP_GPIO_Port 未定义，请在 CubeMX 里给 PB14 设置 User Label 为 BEEP"
#endif

void BEEP_Init(void);
void BEEP_On(void);
void BEEP_Off(void);
void BEEP_Toggle(void);

/* 阻塞式短鸣（毫秒） */
void BEEP_BeepMs(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif
