#ifndef __FATIGUE_TASK_H__
#define __FATIGUE_TASK_H__

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * fatigue_task —— 疲劳状态判定任务
 *
 * 职责（对应课题要求 3）：
 *   1) 运动幅度方差：加速度模值去重力后做 4 s 窗口方差，增量更新（O(1)，不做 FFT）
 *   2) 步态周期：检测竖直轴峰值求步频 f_step，自相关主峰求周期 T_gait
 *      自相关只算 lag ∈ [0.5 s, 2.0 s]（步频 0.5~2 Hz），约 150 个 lag
 *   3) 步态变异系数 CV = std(T_gait) / mean(T_gait)（增强指标）
 *   4) 三级输出：正常 / 轻度疲劳 / 显著疲劳
 *
 * 基线自校准：上电静立 10 s 建立个人基线，阈值随个体调整
 * 本任务运行频率低（建议 1~2 Hz），优先级可低于 tinyml_task
 */

/* 任务句柄（由 _Task_Init 创建后赋值） */
extern TaskHandle_t FatigueTaskHandle;

/* 创建任务并分配优先级与栈大小，由 freertos.c 的 MX_FREERTOS_Init() 调用。
   prio  : osPriority_t 枚举值
   stack : 栈深度，单位 word（1 word = 4 字节） */
void Fatigue_Task_Init(osPriority_t prio, uint32_t stack);

#ifdef __cplusplus
}
#endif

#endif /* __FATIGUE_TASK_H__ */
