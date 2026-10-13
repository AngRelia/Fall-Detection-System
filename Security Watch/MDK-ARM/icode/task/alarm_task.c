/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    alarm_task.c
  * @brief   本地蜂鸣告警与取消按钮任务（防误报核心）
  ******************************************************************************
  * 状态机（对应课题要求 5）：
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
  * 设计要点：
  *   - 取消窗口 20 s、冷却 120 s 都做成宏，方便调参并写进论文
  *   - 按键复用 icode\driver\key.c（短按/长按识别已实现）
  *   - 蜂鸣器用 icode\driver\beep.c（注意其为低电平触发）
  *   - 被取消的事件不进"跌倒库"，但进"误报库"，是阈值调优的宝贵素材
  *   - 本任务需要及时响应取消按键，建议优先级不低于 tinyml_task
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "alarm_task.h"
#include "cmsis_os.h"
#include "queue.h"
#include "key.h"

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* 告警状态枚举建议：
 *   typedef enum {
 *     ALARM_IDLE = 0, ALARM_PENDING, ALARM_CANCELLED, ALARM_CONFIRMED
 *   } Alarm_State_t;
 */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 取消窗口与冷却时间（ms） */
/* #define ALARM_CANCEL_WINDOW_MS   20000 */
/* #define ALARM_COOLDOWN_MS       120000 */
/* 间歇鸣叫周期（ms）—— 1 Hz */
/* #define ALARM_BEEP_PERIOD_MS      1000 */
/* 确认后长鸣时长（ms） */
/* #define ALARM_LONG_BEEP_MS        3000 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* 任务句柄（由 Alarm_Task_Init 创建后赋值） */
TaskHandle_t AlarmTaskHandle = NULL;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* 任务入口：xTaskCreate 需要在本文件内先见到声明 */
void Alarm_Task_Entry(void *argument);
/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* ---------------------------------------------------------------------------
 * 任务创建：优先级与栈大小由 freertos.c 统一传入
 * ------------------------------------------------------------------------- */
void Alarm_Task_Init(osPriority_t prio, uint32_t stack)
{
  BaseType_t ret = xTaskCreate(Alarm_Task_Entry,
                               "Alarm",
                               (configSTACK_DEPTH_TYPE)stack,
                               NULL,
                               (UBaseType_t)prio,
                               &AlarmTaskHandle);
  /* 堆不足时 xTaskCreate 返回 pdFAIL，任务会静默不运行，必须断言拦住 */
  configASSERT(ret == pdPASS);
}

/* ---------------------------------------------------------------------------
 * 任务入口
 * ------------------------------------------------------------------------- */
void Alarm_Task_Entry(void *argument)
{
  (void)argument;
  /* USER CODE BEGIN Alarm_Task_Entry_Init */
  /* 任务级初始化：只执行一次 */
  Key_Init();          /* 清按键内部状态；引脚已由 MX_GPIO_Init() 配好（输入 + 内部上拉） */
  /* USER CODE END Alarm_Task_Entry_Init */

  for (;;)
  {
    /* USER CODE BEGIN Alarm_Task_Entry_Loop */
    /* ---- 按键扫描：全工程**只有这一处**调用（本任务恰好就是 10 ms 一拍）----
       Key_Scan() 内部做 30 ms 消抖、按下沿出短按事件、按住 500 ms 起连发、1000 ms 出长按事件。
       将来若新增 menu_task，把这一行挪进 menu_task 即可，务必保持"只有一个调用者"。 */
    Key_Scan();

    /* ---- 告警取消（题面要求 5）：返回键长按 = 取消 ----
       这里只"看"队首不"取"走：确认是给自己的，才 Key_GetEvent() 取走，
       免得把事件从菜单手里抢走（菜单用 Key_GetEvent() 消费）。 */
    if (Key_PeekEvent() == KEY_EVENT_BACK_LONG)
    {
      (void)Key_GetEvent();
      /* TODO: 若当前是 PENDING → 转 CANCELLED、停间歇鸣叫、把这段窗口记为误报样本 */
    }

    /* TODO: IDLE / PENDING / CANCELLED / CONFIRMED 状态机 */

    vTaskDelay(pdMS_TO_TICKS(10));
    /* USER CODE END Alarm_Task_Entry_Loop */
  }
}

/* USER CODE END Application */
