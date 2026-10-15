/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    menu_task.c
  * @brief   菜单/状态总览任务：把三条链路 + 系统状态汇总成一屏可读的视图
  ******************************************************************************
  * 分工（见 docs/菜单与交互设计.md）：
  *   本任务：翻页（上/下键）+ 汇总各子系统快照 → Menu_View_t（双缓冲）
  *   oled_task：读 Menu_View_t → 画屏（OLED 是软件 I2C，绘制只能有一个任务做）
  *
  * 按键：上/下用 Key_TakeNav() 的"导航锁存"，**不占用事件队列**，
  *       因此与"长按返回取消告警"那套队列消费互不干扰。
  *       上 = 上一页，下 = 下一页，循环。
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "menu_task.h"
#include "key.h"
#include "imu_recv_task.h"
#include "gps.h"
#include "hc08.h"
#include "ble_task.h"
#include <string.h>

/* Private define ------------------------------------------------------------*/
#define MENU_TASK_PERIOD_MS     20U     /* 20 ms：按键响应即时，汇总开销极小 */
#define MENU_LAST_LINE_LEN      12U     /* 只在屏幕上显示这么多字符 */

/* Private variables ---------------------------------------------------------*/
TaskHandle_t MenuTaskHandle = NULL;

/* 双缓冲：写者填 [1-idx]，填完原子翻转 idx；读者拷 [idx]。
   idx 是 8 位，写入是单指令，天然原子 ✓ */
static Menu_View_t       s_view[2];
static volatile uint8_t  s_view_idx = 0U;
static uint8_t           s_page = 0U;

/* Private function prototypes -----------------------------------------------*/
void        Menu_Task_Entry(void *argument);
static void Menu_Collect(Menu_View_t *v);

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void Menu_Task_Init(osPriority_t prio, uint32_t stack)
{
  BaseType_t ret = xTaskCreate(Menu_Task_Entry,
                               "MENU",
                               (configSTACK_DEPTH_TYPE)stack,
                               NULL,
                               (UBaseType_t)prio,
                               &MenuTaskHandle);
  /* 堆不足时 xTaskCreate 返回 pdFAIL，任务会静默不运行，必须断言拦住 */
  configASSERT(ret == pdPASS);
}

void Menu_GetView(Menu_View_t *out)
{
  if (out == NULL)
  {
    return;
  }
  /* 拷贝"当前这一份"：写者此刻正在填另一份，所以不会读到写一半的数据 */
  (void)memcpy(out, &s_view[s_view_idx], sizeof(Menu_View_t));
}

/* ---------------------------------------------------------------------------
 * 汇总各子系统状态（全部是只读快照接口）
 * ------------------------------------------------------------------------- */
static void Menu_Collect(Menu_View_t *v)
{
  IMU_Recv_Snapshot_t imu;
  GPS_Fix_t           fix;
  uint32_t            txok = 0U, txdrop = 0U;

  (void)memset(v, 0, sizeof(Menu_View_t));
  v->page = s_page;

  /* ---- 节点 A：IMU 链路 ---- */
  IMU_Recv_GetSnapshot(&imu);
  v->a_rate = imu.frame_rate;
  v->a_lost = imu.link_lost;
  v->a_ok   = imu.frames_ok;
  v->a_crc  = imu.crc_err;
  v->a_drop = imu.seq_drop;
  v->a_rsy  = imu.resync;
  v->a_rovr = imu.ring_overrun;

  /* ---- GPS ---- */
  v->g_alive = GPS_IsAlive(HAL_GetTick()) ? 1U : 0U;
  v->g_rx    = GPS_GetRxBytes();
  v->g_sen   = GPS_GetSentenceCount();
  v->g_cse   = GPS_GetCsumErrCount();
  if (GPS_GetFix(&fix) != 0U)
  {
    v->g_fix    = fix.valid;
    v->g_q      = fix.quality;
    v->g_st     = fix.sats;
    v->g_vw     = fix.sats_view;
    v->g_mx     = fix.snr_max;
    v->g_lat_e7 = fix.lat_e7;
    v->g_lon_e7 = fix.lon_e7;
  }

  /* ---- BLE / HC-08 ---- */
  v->b_conn  = HC08_IsConnected();
  v->b_txb   = HC08_GetTxBytes();
  v->b_rxb   = HC08_GetRxBytes();
  v->b_atok  = HC08_GetAtOkCount();
  v->b_aterr = HC08_GetAtErrCount();
  BLE_GetStats(&txok, &txdrop);
  v->b_send  = txok;
  v->b_drop  = txdrop;
  (void)HC08_GetLastLine(v->b_last, MENU_LAST_LINE_LEN);

  /* ---- 系统 ---- */
  v->heap     = (uint32_t)xPortGetFreeHeapSize();
  v->uptime_s = (uint32_t)(HAL_GetTick() / 1000U);
}

/* ---------------------------------------------------------------------------
 * 任务入口
 * ------------------------------------------------------------------------- */
void Menu_Task_Entry(void *argument)
{
  Key_Event_t nav;
  uint8_t     next;

  (void)argument;

  /* 初始化两份缓冲，避免上电瞬间读到未初始化内容 */
  (void)memset(&s_view[0], 0, sizeof(Menu_View_t));
  (void)memset(&s_view[1], 0, sizeof(Menu_View_t));

  for (;;)
  {
    /* ---- 1) 翻页：上=上一页，下=下一页，循环（含长按连发）---- */
    nav = Key_TakeNav();
    if ((nav == KEY_EVENT_UP) || (nav == KEY_EVENT_UP_REPEAT))
    {
      s_page = (uint8_t)((s_page + (MENU_PAGE_NUM - 1U)) % MENU_PAGE_NUM);
    }
    else if ((nav == KEY_EVENT_DOWN) || (nav == KEY_EVENT_DOWN_REPEAT))
    {
      s_page = (uint8_t)((s_page + 1U) % MENU_PAGE_NUM);
    }

    /* ---- 2) 汇总到"另一份"缓冲，然后原子翻转 ---- */
    next = (uint8_t)(s_view_idx ^ 1U);
    Menu_Collect(&s_view[next]);
    s_view_idx = next;

    vTaskDelay(pdMS_TO_TICKS(MENU_TASK_PERIOD_MS));
  }
}

/* USER CODE END Application */
