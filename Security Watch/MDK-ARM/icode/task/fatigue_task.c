/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fatigue_task.c
  * @brief   疲劳状态判定任务
  ******************************************************************************
  * 职责（对应课题要求 3）：
  *   1) 运动幅度方差：加速度模值去重力后做 4 s 窗口方差
  *      用滑动和/平方和增量更新，复杂度 O(1)，不做 FFT
  *   2) 步态周期：检测竖直轴峰值求步频 f_step，自相关主峰求周期 T_gait
  *      自相关只算 lag ∈ [0.5 s, 2.0 s]（步频 0.5~2 Hz），约 150 个 lag
  *   3) 步态变异系数 CV = std(T_gait) / mean(T_gait)（增强指标，CV 升高是疲劳标志）
  *   4) 三级输出：正常 / 轻度疲劳 / 显著疲劳
  *
  * 基线自校准：上电静立 10 s 建立个人基线，阈值随个体调整
  *
  * 运行频率：本任务只需 1~2 Hz，优先级可低于 tinyml_task
  * 注意：F103 无 FPU 也无硬件除法器，浮点方差统计要控制在低频调用
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "fatigue_task.h"
#include "cmsis_os.h"
#include "queue.h"

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* 疲劳等级枚举建议：
 *   typedef enum { FATIGUE_NORMAL = 0, FATIGUE_MILD, FATIGUE_SEVERE } Fatigue_Level_t;
 */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 方差窗口：4 s @ 100 Hz = 400 采样 */
/* #define FATIGUE_VAR_WINDOW      400 */
/* 步频搜索范围 */
/* #define STEP_FREQ_MIN_HZ        0.5f */
/* #define STEP_FREQ_MAX_HZ        2.0f */
/* 基线校准时长（ms） */
/* #define FATIGUE_BASELINE_MS     10000 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* 任务句柄（由 Fatigue_Task_Init 创建后赋值） */
TaskHandle_t FatigueTaskHandle = NULL;
/* 建议保存：滑动和 sum_a / sum_a2、个人基线 baseline_var / baseline_step_hz */
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* 任务入口：xTaskCreate 需要在本文件内先见到声明 */
void Fatigue_Task_Entry(void *argument);
/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* ---------------------------------------------------------------------------
 * 任务创建：优先级与栈大小由 freertos.c 统一传入
 * ------------------------------------------------------------------------- */
void Fatigue_Task_Init(osPriority_t prio, uint32_t stack)
{
  BaseType_t ret = xTaskCreate(Fatigue_Task_Entry,
                               "Fatigue",
                               (configSTACK_DEPTH_TYPE)stack,
                               NULL,
                               (UBaseType_t)prio,
                               &FatigueTaskHandle);
  /* 堆不足时 xTaskCreate 返回 pdFAIL，任务会静默不运行，必须断言拦住 */
  configASSERT(ret == pdPASS);
}

/* ---------------------------------------------------------------------------
 * 任务入口
 * ------------------------------------------------------------------------- */
void Fatigue_Task_Entry(void *argument)
{
  (void)argument;
  /* USER CODE BEGIN Fatigue_Task_Entry_Init */
  /* 任务级初始化：只执行一次 */
  /* USER CODE END Fatigue_Task_Entry_Init */

  for (;;)
  {
    /* USER CODE BEGIN Fatigue_Task_Entry_Loop */
    /* TODO: 增量方差 → 步频/自相关 → 与基线比较 → 输出疲劳等级 */

    vTaskDelay(pdMS_TO_TICKS(10));
    /* USER CODE END Fatigue_Task_Entry_Loop */
  }
}

/* USER CODE END Application */
