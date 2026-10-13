/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    imu_recv_task.c
  * @brief   DIY IMU（节点A）数据接收解析任务
  ******************************************************************************
  * 职责：
  *   1) 从节点A接收 32 字节定长 IMU 数据帧（帧格式见 diy_imu.h）
  *   2) 排空字节缓冲并解析，校验 CRC-16/MODBUS
  *   3) 单位换算成 mg / 0.1°/s 后喂给 tinyml_task（TinyML_PushFrame）
  *   4) 统计帧率、丢帧数、CRC 错误数
  *
  * 分层：
  *   diy_imu.c  —— 链路层：DMA 收字节、找帧头、校验 CRC、单位换算
  *   本文件     —— 任务层：排空、喂 TinyML、统计与链路健康监控
  *
  * 单位契约（与 ml_features.h / tinyml_task.h 一致）：
  *   acc  : mg      （±16 g 量程，1 g = 1000 mg）
  *   gyro : 0.1 °/s （±2000 °/s 量程，1 °/s = 10 LSB）
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "imu_recv_task.h"
#include "cmsis_os.h"
#include "queue.h"
#include "diy_imu.h"
#include "tinyml_task.h"
#include <string.h>          /* memset */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---------------------------------------------------------------------------
 * ★★★ 按实际接线改这一处 ★★★
 * 默认节点A挂 USART3（PB10/PB11, 115200, DMA1_Channel3 循环接收）。
 * 若改用别的串口，只改下面两行（并确认该串口在 CubeMX 里配好了 RX DMA）：
 *   USART3（PB10/PB11） → &huart3   ← 当前
 *   USART2（PA2/PA3）   → &huart2   （注意：本工程 USART2 是 HC-08 BLE）
 *   USART1（PB6/PB7）   → &huart1   （注意：本工程 USART1 是 GPS）
 * ------------------------------------------------------------------------- */
extern UART_HandleTypeDef huart3;
#define DIY_IMU_UART_HANDLE     (&huart3)

/* 任务节拍。5 ms 明显快于帧到达间隔（100 Hz → 10 ms），
   保证字节缓冲及时排空，不会因为排空慢而回卷丢字节。 */
#define IMU_RECV_PERIOD_MS      5U

/* 帧率统计窗口。1 s 恰好给出「每秒帧数」，读起来直观。 */
#define IMU_RATE_WINDOW_MS      1000U

/* 超过该时间没收到任何字节 → 判定链路断开（接线松/节点A掉电）。
   阈值取 1 s：远大于正常帧间隔（约 10~24 ms），不会误判。 */
#define IMU_LINK_TIMEOUT_MS     1000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* 任务句柄（由 IMU_Recv_Task_Init 创建后赋值） */
TaskHandle_t IMU_RecvTaskHandle = NULL;

/* 帧率统计：窗口内累计的帧数 + 窗口起点 */
static uint32_t s_rate_frames = 0;
static uint32_t s_rate_start  = 0;

/* 最新一次算出的快照（由本任务写、其它任务读，用临界区保护） */
static IMU_Recv_Snapshot_t s_snap;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* 任务入口：xTaskCreate 需要在本文件内先见到声明 */
void IMU_Recv_Task_Entry(void *argument);
/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* ---------------------------------------------------------------------------
 * 任务创建：优先级与栈大小由 freertos.c 统一传入
 * ------------------------------------------------------------------------- */
void IMU_Recv_Task_Init(osPriority_t prio, uint32_t stack)
{
  BaseType_t ret = xTaskCreate(IMU_Recv_Task_Entry,
                               "IMU_Recv",
                               (configSTACK_DEPTH_TYPE)stack,
                               NULL,
                               (UBaseType_t)prio,
                               &IMU_RecvTaskHandle);
  /* 堆不足时 xTaskCreate 返回 pdFAIL，任务会静默不运行，必须断言拦住 */
  configASSERT(ret == pdPASS);
}

/* ---------------------------------------------------------------------------
 * 对外：读取链路运行快照
 * ------------------------------------------------------------------------- */
void IMU_Recv_GetSnapshot(IMU_Recv_Snapshot_t *snap)
{
  if (snap == NULL) {
    return;
  }
  taskENTER_CRITICAL();
  *snap = s_snap;
  taskEXIT_CRITICAL();
}

/* ---------------------------------------------------------------------------
 * 把驱动统计刷新到快照，并计算帧率
 * ------------------------------------------------------------------------- */
static void IMU_Recv_UpdateSnapshot(uint32_t now)
{
  DIY_IMU_Stats_t st;

  DIY_IMU_GetStats(&st);

  /* 帧率：窗口满 1 s 结算一次，四舍五入到整数 Hz */
  if ((uint32_t)(now - s_rate_start) >= IMU_RATE_WINDOW_MS) {
    uint32_t elapsed = (uint32_t)(now - s_rate_start);
    uint16_t fps = 0;

    if (elapsed > 0U) {
      fps = (uint16_t)(((uint64_t)s_rate_frames * 1000ULL + elapsed / 2ULL) / elapsed);
    }

    taskENTER_CRITICAL();
    s_snap.frame_rate = fps;
    taskEXIT_CRITICAL();

    s_rate_frames = 0;
    s_rate_start  = now;
  }

  taskENTER_CRITICAL();
  s_snap.frames_ok      = st.frames_ok;
  s_snap.crc_err        = st.crc_err;
  s_snap.seq_drop       = st.seq_drop;
  s_snap.resync         = st.resync;
  s_snap.ring_overrun   = st.ring_overrun;
  s_snap.byte_ring_drop = st.byte_ring_drop;
  s_snap.inited         = st.inited;
  s_snap.link_lost      = (DIY_IMU_GetLastRxAge(now) > IMU_LINK_TIMEOUT_MS) ? 1U : 0U;
  taskEXIT_CRITICAL();
}

/* ---------------------------------------------------------------------------
 * 任务入口
 * ------------------------------------------------------------------------- */
void IMU_Recv_Task_Entry(void *argument)
{
  (void)argument;

  /* USER CODE BEGIN IMU_Recv_Task_Entry_Init */
  /* 绑定串口并启动「循环 DMA + 空闲中断」接收。
     前置条件（都在 Core/Src/usart.c 里）：USART3 已 Init、
     hdma_usart3_rx 已配为 DMA_CIRCULAR、USART3_IRQn 已使能。
     否则 HAL_UARTEx_ReceiveToIdle_DMA() 会返回错误。 */
  if (DIY_IMU_Init(DIY_IMU_UART_HANDLE) != DIY_IMU_OK) {
    /* 初始化失败通常意味着 CubeMX 侧的 DMA/中断没配对。
       这里不阻塞，让任务继续跑：链路监控会在超时后反复尝试重新武装，
       屏幕上表现为 link_lost=1、帧率 0，便于定位。 */
  }

  memset(&s_snap, 0, sizeof(s_snap));
  s_rate_frames = 0;
  s_rate_start  = xTaskGetTickCount();
  /* USER CODE END IMU_Recv_Task_Entry_Init */

  for (;;)
  {
    /* USER CODE BEGIN IMU_Recv_Task_Entry_Loop */
    uint32_t now = xTaskGetTickCount();

    /* 1) 排空字节缓冲并解析出帧 */
    (void)DIY_IMU_Process();

    /* 2) 取帧 → 喂 TinyML。
       TinyML_PushFrame 只做一次 12 字节 memcpy，耗时约 1 us，
       所以可以放心地在一次循环里把队列里的帧全部喂完。 */
    {
      DIY_IMU_Frame_t fr;

      while (DIY_IMU_TakeFrame(&fr)) {
        int16_t frame[ML_CH];

        /* 顺序必须是 ax, ay, az, gx, gy, gz —— 见 ml_features.h 的输入约定。
           单位是 mg 与 0.1°/s，换算已在 diy_imu.c 里以整数精确完成。 */
        frame[0] = fr.acc_mg[0];
        frame[1] = fr.acc_mg[1];
        frame[2] = fr.acc_mg[2];
        frame[3] = fr.gyro_01dps[0];
        frame[4] = fr.gyro_01dps[1];
        frame[5] = fr.gyro_01dps[2];

        TinyML_PushFrame(frame);

        s_rate_frames++;
      }
    }

    /* 3) 刷新统计并按需重新武装接收 */
    IMU_Recv_UpdateSnapshot(now);

    if (DIY_IMU_GetLastRxAge(now) > IMU_LINK_TIMEOUT_MS) {
      /* 链路断了：节点A没上电 / 接线松 / 串口被复位。
         DIY_IMU_Reinit() 内部先 Abort 再启动，重复调用是安全的。
         注意：不能在这里做任何阻塞等待，否则会拖慢其它任务。 */
      DIY_IMU_Reinit();
    }
    /* USER CODE END IMU_Recv_Task_Entry_Loop */

    vTaskDelay(pdMS_TO_TICKS(IMU_RECV_PERIOD_MS));
  }
}

/* USER CODE END Application */
