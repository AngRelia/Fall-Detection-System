#ifndef __OLED_TASK_H__
#define __OLED_TASK_H__

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * oled_task —— OLED 显示任务
 *
 * 职责：
 *   1) 刷新运行状态：串口帧率 / 丢帧数 / CRC 错误数（联调期最有用）
 *   2) 显示当前疲劳等级与跌倒检测状态
 *   3) 告警时显示醒目提示与取消倒计时
 *
 * 设计要点：
 *   - OLED.c 是软件 I2C，刷屏耗时可观，建议 10 Hz 左右刷新，不要放在高优先级任务里
 *   - 允许多任务通过共享状态结构传数据，刷屏动作只在本任务内做（避免 I2C 总线竞争）
 *   - 与 alarm_task 的交互建议用事件标志组或共享状态变量 + 临界区保护
 */

/* 任务句柄（由 _Task_Init 创建后赋值） */
extern TaskHandle_t OLEDTaskHandle;

/* 创建任务并分配优先级与栈大小，由 freertos.c 的 MX_FREERTOS_Init() 调用。
   prio  : osPriority_t 枚举值
   stack : 栈深度，单位 word（1 word = 4 字节） */
void OLED_Task_Init(osPriority_t prio, uint32_t stack);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_TASK_H__ */
