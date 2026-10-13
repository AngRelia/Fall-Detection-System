#include "beep.h"

void BEEP_Init(void)
{
    /* 低电平触发，初始化默认关闭 */
    BEEP_Off();
}

void BEEP_On(void)
{
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
}

void BEEP_Off(void)
{
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
}

void BEEP_Toggle(void)
{
    HAL_GPIO_TogglePin(BEEP_GPIO_Port, BEEP_Pin);
}

void BEEP_BeepMs(uint32_t ms)
{
    BEEP_On();
    HAL_Delay(ms);
    BEEP_Off();
}
