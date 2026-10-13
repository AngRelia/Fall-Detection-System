/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tinyml_task.c
  * @brief   TinyML 跌倒检测推理任务
  ******************************************************************************
  * 职责：
  *   1) 从环形缓冲区取 2 s @ 100 Hz 滑动窗口（6 通道：ax/ay/az/gx/gy/gz）
  *   2) 阈值预筛（|a| > 2.5 g 或 |ω| > 300 °/s）命中后，调用 int8 定点推理内核
  *   3) 输出 Fall / Near-fall / ADL 三分类，置信度 ≥ 0.8 时置告警事件
  *
  * 实现说明与资源实测见 tinyml_task.h 的文件头注释。
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "tinyml_task.h"
#include "cmsis_os.h"
#include "queue.h"
#include "ml_features.h"
#include "ml_infer.h"
#include "ml_weights.h"
#include "ml_selftest.h"
#include <math.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* 环形缓冲长度：必须比窗口多 1 帧，理由见 ml_features.h 的
   ML_ExtractFeaturesRing 说明（让生产者不会覆盖到消费者正在读的区域） */
#define ML_RING_LEN     (ML_WIN + 1)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* 任务句柄（由 TinyML_Task_Init 创建后赋值） */
TaskHandle_t TinyMLTaskHandle = NULL;

/* 采样环形缓冲：行主序连续存放，(ML_WIN+1) × 6 × int16 = 2412 字节 */
static int16_t s_ring[ML_RING_LEN][ML_CH];

/* 下一帧要写入的行号。16 位对齐读写，在 Cortex-M3 上是原子的，
   因此消费者可以直接读取而无需关中断。 */
static volatile uint16_t s_wr = 0;

/* 已积累的帧数，封顶到 ML_WIN */
static volatile uint16_t s_fill = 0;

/* 待处理的跌倒事件（单槽）。用短暂临界区保护读写。 */
static TinyML_Event_t s_event;
static volatile uint8_t s_event_pending = 0;

/* 上次上报的 tick，用于冷却抑制 */
static uint32_t s_last_report_tick = 0;

/* 运行统计 */
static TinyML_Stats_t s_stats;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* 任务入口：xTaskCreate 需要在本文件内先见到声明 */
void TinyML_Task_Entry(void *argument);

static uint8_t TinyML_Prefilter(const int16_t *ring, uint16_t start);
static void    TinyML_Evaluate(void);
/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* ---------------------------------------------------------------------------
 * 任务创建：优先级与栈大小由 freertos.c 统一传入
 * ------------------------------------------------------------------------- */
void TinyML_Task_Init(osPriority_t prio, uint32_t stack)
{
  BaseType_t ret = xTaskCreate(TinyML_Task_Entry,
                               "TinyML",
                               (configSTACK_DEPTH_TYPE)stack,
                               NULL,
                               (UBaseType_t)prio,
                               &TinyMLTaskHandle);
  /* 堆不足时 xTaskCreate 返回 pdFAIL，任务会静默不运行，必须断言拦住 */
  configASSERT(ret == pdPASS);
}

/* ---------------------------------------------------------------------------
 * 数据入口：由 imu_recv_task 每解析出一帧调用一次
 * ------------------------------------------------------------------------- */
void TinyML_PushFrame(const int16_t frame[ML_CH])
{
  /* 单帧 12 字节拷贝。这里不加临界区：本函数约定在任务上下文调用，
     而消费者只读"至少 1 帧之前"的数据（见 ML_RING_LEN 的说明）。 */
  memcpy(s_ring[s_wr], frame, sizeof(s_ring[0]));

  s_wr++;
  if (s_wr >= ML_RING_LEN) {
    s_wr = 0;
  }
  if (s_fill < ML_WIN) {
    s_fill++;
  }
  s_stats.frames_pushed++;
}

/* ---------------------------------------------------------------------------
 * 阈值预筛：代价极低，用来把绝大多数日常动作挡在网络之外
 * ------------------------------------------------------------------------- */
static uint8_t TinyML_Prefilter(const int16_t *ring, uint16_t start)
{
  float am_max = 0.0f;
  float gm_max = 0.0f;
  int i;

  for (i = 0; i < ML_WIN; i++) {
    int k = (int)start + i;
    const int16_t *s;
    float ax, ay, az, gx, gy, gz, am, gm;

    if (k >= ML_RING_LEN) {
      k -= ML_RING_LEN;
    }
    s = ring + (k * ML_CH);

    ax = (float)s[0];
    ay = (float)s[1];
    az = (float)s[2];
    gx = (float)s[3];
    gy = (float)s[4];
    gz = (float)s[5];

    /* 单位与门限保持一致：acc 用 mg、gyro 用 0.1°/s，省掉一次换算 */
    am = sqrtf(ax * ax + ay * ay + az * az);
    gm = sqrtf(gx * gx + gy * gy + gz * gz);

    if (am > am_max) am_max = am;
    if (gm > gm_max) gm_max = gm;
  }

  return (uint8_t)((am_max > (float)TINYML_PREFILTER_ACC_MG)
                   || (gm_max > (float)TINYML_PREFILTER_GYRO_DPS));
}

/* ---------------------------------------------------------------------------
 * 一次完整的窗口评估：预筛 → 特征 → 量化 → 推理 → 判定
 * ------------------------------------------------------------------------- */
static void TinyML_Evaluate(void)
{
  uint16_t start;
  float feat[ML_FEAT_DIM];
  int8_t in_q[ML_IN_DIM];
  int8_t out_q[ML_OUT_DIM];
  float prob[ML_OUT_DIM];
  int cls;
  uint32_t now;

  if (s_fill < ML_WIN) {
    return;                                 /* 数据还不够 2 s，不评 */
  }

  /* 最旧一帧的行号：下一帧要写的槽位的后一个。原子读一次即可，
     之后生产者至少 2 s 才会覆盖到我们正在读的区域。 */
  start = (uint16_t)((s_wr + 1) % ML_RING_LEN);

  s_stats.windows_evaluated++;

  if (!TinyML_Prefilter(&s_ring[0][0], start)) {
    return;                                 /* 日常动作：直接丢弃，不进网络 */
  }
  s_stats.prefilter_triggers++;

  ML_ExtractFeaturesRing(&s_ring[0][0], ML_RING_LEN, start, feat);
  ML_QuantizeFeatures(feat, in_q);
  ML_Run(in_q, out_q);
  ML_Softmax(out_q, prob);

  cls = ML_Predict(feat);
  if (cls != ML_CLASS_FALL || prob[ML_CLASS_FALL] < TINYML_CONF_THRESHOLD) {
    return;
  }

  s_stats.falls_detected++;

  /* 冷却抑制：连续多个窗都命中时只上报一次，避免连打告警 */
  now = xTaskGetTickCount();
  if (s_event_pending
      || (uint32_t)(now - s_last_report_tick) < pdMS_TO_TICKS(TINYML_COOLDOWN_MS)) {
    return;
  }
  s_last_report_tick = now;

  taskENTER_CRITICAL();
  s_event.cls   = ML_CLASS_FALL;
  s_event.margin = ML_Margin(out_q);
  s_event.tick  = now;
  memcpy(s_event.logit, out_q, sizeof(out_q));
  memcpy(s_event.conf, prob, sizeof(prob));
  memcpy(s_event.feat, feat, sizeof(feat));
  {
    int k = (int)start + ML_WIN - 1;        /* 窗口内最后一帧 = 触发时刻 */
    if (k >= ML_RING_LEN) {
      k -= ML_RING_LEN;
    }
    s_event.acc_mg[0] = s_ring[k][0];
    s_event.acc_mg[1] = s_ring[k][1];
    s_event.acc_mg[2] = s_ring[k][2];
  }
  s_event_pending = 1;
  taskEXIT_CRITICAL();
}

/* ---------------------------------------------------------------------------
 * 事件与统计的对外接口
 * ------------------------------------------------------------------------- */
uint8_t TinyML_TakeEvent(TinyML_Event_t *ev)
{
  uint8_t got = 0;

  taskENTER_CRITICAL();
  if (s_event_pending) {
    if (ev != NULL) {
      *ev = s_event;
    }
    s_event_pending = 0;
    got = 1;
  }
  taskEXIT_CRITICAL();

  return got;
}

void TinyML_GetStats(TinyML_Stats_t *st)
{
  if (st == NULL) {
    return;
  }
  taskENTER_CRITICAL();
  *st = s_stats;
  taskEXIT_CRITICAL();
}

void TinyML_ResetStats(void)
{
  taskENTER_CRITICAL();
  memset(&s_stats, 0, sizeof(s_stats));
  taskEXIT_CRITICAL();
}

/* ---------------------------------------------------------------------------
 * 任务入口
 * ------------------------------------------------------------------------- */
void TinyML_Task_Entry(void *argument)
{
  (void)argument;
  /* USER CODE BEGIN TinyML_Task_Entry_Init */
  {
    /* 上电自测：用 PC 端算好的黄金向量验证 MCU 端推理链路。
       见 ml_selftest.h —— 这是排查"模型在板子上不准"的第一道防线。 */
    ML_SelfTestResult_t r;
    uint8_t ok = (uint8_t)ML_SelfTest(&r);

    s_stats.selftest_ok        = ok;
    s_stats.selftest_class_ok  = r.class_ok;
    s_stats.selftest_total     = r.total;
    s_stats.selftest_maxdiff   = r.max_lsb_diff;
  }
  /* USER CODE END TinyML_Task_Entry_Init */

  for (;;)
  {
    /* USER CODE BEGIN TinyML_Task_Entry_Loop */
    TinyML_Evaluate();

    vTaskDelay(pdMS_TO_TICKS(TINYML_TASK_PERIOD_MS));
    /* USER CODE END TinyML_Task_Entry_Loop */
  }
}

/* USER CODE END Application */
