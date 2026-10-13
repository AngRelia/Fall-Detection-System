#include "led.h"

static TIM_HandleTypeDef *s_led_tim = NULL;

static uint32_t LED_ChannelToTim(LED_Channel_t ch)
{
    switch (ch)
    {
    case LED_CH1: return TIM_CHANNEL_2;
    case LED_CH2: return TIM_CHANNEL_3;
    case LED_CH3: return TIM_CHANNEL_4;
    default:      return 0;
    }
}

HAL_StatusTypeDef LED_PWM_Init(TIM_HandleTypeDef *htim_pwm)
{
    if (htim_pwm == NULL)
    {
        return HAL_ERROR;
    }

    s_led_tim = htim_pwm;

    /* 默认全灭 */
    LED_SetAllPercent(0, 0, 0);
    return HAL_OK;
}

HAL_StatusTypeDef LED_PWM_Start(void)
{
    HAL_StatusTypeDef ret;

    if (s_led_tim == NULL)
    {
        return HAL_ERROR;
    }

    ret = HAL_TIM_PWM_Start(s_led_tim, TIM_CHANNEL_2);
    if (ret != HAL_OK) return ret;
    ret = HAL_TIM_PWM_Start(s_led_tim, TIM_CHANNEL_3);
    if (ret != HAL_OK) return ret;
    ret = HAL_TIM_PWM_Start(s_led_tim, TIM_CHANNEL_4);
    return ret;
}

HAL_StatusTypeDef LED_PWM_Stop(void)
{
    HAL_StatusTypeDef ret;

    if (s_led_tim == NULL)
    {
        return HAL_ERROR;
    }

    ret = HAL_TIM_PWM_Stop(s_led_tim, TIM_CHANNEL_2);
    if (ret != HAL_OK) return ret;
    ret = HAL_TIM_PWM_Stop(s_led_tim, TIM_CHANNEL_3);
    if (ret != HAL_OK) return ret;
    ret = HAL_TIM_PWM_Stop(s_led_tim, TIM_CHANNEL_4);
    return ret;
}

void LED_SetCompare(LED_Channel_t ch, uint16_t compare)
{
    uint32_t tim_ch;
    uint32_t arr;

    if (s_led_tim == NULL || ch >= LED_CH_MAX)
    {
        return;
    }

    tim_ch = LED_ChannelToTim(ch);
    if (tim_ch == 0)
    {
        return;
    }

    arr = __HAL_TIM_GET_AUTORELOAD(s_led_tim);
    if (compare > arr)
    {
        compare = (uint16_t)arr;
    }

    __HAL_TIM_SET_COMPARE(s_led_tim, tim_ch, compare);
}

void LED_SetBrightnessPercent(LED_Channel_t ch, uint8_t percent)
{
    uint32_t arr;
    uint32_t ccr;

    if (s_led_tim == NULL)
    {
        return;
    }

    if (percent > 100U)
    {
        percent = 100U;
    }

    arr = __HAL_TIM_GET_AUTORELOAD(s_led_tim);
    ccr = (arr * percent) / 100U;
    LED_SetCompare(ch, (uint16_t)ccr);
}

void LED_SetAllPercent(uint8_t ch1_percent, uint8_t ch2_percent, uint8_t ch3_percent)
{
    LED_SetBrightnessPercent(LED_CH1, ch1_percent);
    LED_SetBrightnessPercent(LED_CH2, ch2_percent);
    LED_SetBrightnessPercent(LED_CH3, ch3_percent);
}
