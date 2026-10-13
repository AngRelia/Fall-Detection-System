#ifndef __ALARM_TASK_H__
#define __ALARM_TASK_H__

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * alarm_task —— 本地蜂鸣告警与取消按钮任务
 *
 * 职责（对应课题要求 5，防误报核心）：
 *
 *   IDLE ──(tinyml_task 置 Fall)──▶ PENDING  蜂鸣器 1 Hz 间歇鸣叫
 *                                      │
 *                                      ├─(20 s 内按下取消按钮)─▶ CANCELLED
 *                                      │      记录误报样本（进误报库，供再训练）
 *                                      │
 *                                      └─(20 s 超时未取消)────▶ CONFIRMED
 *                                             长鸣 3 s + 置 BLE 上报请求
 *                                             120 s 冷却后回 IDLE
 *
 * 可调参数建议做成宏：ALARM_CANCEL_WINDOW_MS / ALARM_COOLDOWN_MS
 * 按键复用 icode\driver\key.c（短按/长按识别已实现）
 */

/* 任务句柄（由 _Task_Init 创建后赋值） */
extern TaskHandle_t AlarmTaskHandle;

/* 创建任务并分配优先级与栈大小，由 freertos.c 的 MX_FREERTOS_Init() 调用。
   prio  : osPriority_t 枚举值
   stack : 栈深度，单位 word（1 word = 4 字节） */
void Alarm_Task_Init(osPriority_t prio, uint32_t stack);

#ifdef __cplusplus
}
#endif

#endif /* __ALARM_TASK_H__ */
