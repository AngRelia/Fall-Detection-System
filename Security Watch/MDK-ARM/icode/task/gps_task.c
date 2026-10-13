#include "gps_task.h"
#include "cmsis_os.h"
#include "queue.h"
#include "gps.h"

/* ===========================================================================
 * gps_task.c —— GNSS 定位任务
 * 设计说明见 gps_task.h
 * ========================================================================= */

/* ---------------------------------------------------------------------------
 * ★★★ 按实际接线改这一处 ★★★
 * 默认按《GPS模块接入方案》§4.3 的分配：GPS 挂 USART1（重映射 PB6/PB7）。
 * 若你改用别的串口（例如 GPS 放 USART3），只改下面两行即可：
 *   USART3（PB10/PB11） → extern UART_HandleTypeDef huart3;  (&huart3)
 *   USART2（PA2/PA3）   → extern UART_HandleTypeDef huart2;  (&huart2)
 * ------------------------------------------------------------------------- */
extern UART_HandleTypeDef huart1;
#define GPS_UART_HANDLE     (&huart1)

/* 任务节拍：20 ms 足以排空环形缓冲（默认语句集约 620 B/s） */
#define GPS_TASK_PERIOD_MS      20U
/* 上电后等模块稳定，再发 $PCA 配置命令 */
#define GPS_CONFIG_DELAY_MS     2000U
/* 超过该时间一个字节都没收到 → 判定链路异常并重新武装接收 */
#define GPS_RX_TIMEOUT_MS       5000U

/* ★ 首次联调请保持 0：**只收不发**。
   理由：模块默认就在 1 Hz 持续吐 NMEA，先纯接收能把"没数据"的原因收敛到
   接线/供电/波特率三件事上；一旦发配置命令，就多了一个"命令写坏导致模块
   不说话"的变量。确认 GPS 接收页能看到语句（RX 在涨、LST 有 $GNRMC）之后，
   再改成 1 把输出精简成只留 GGA+RMC（约 620 B/s → 150 B/s）。 */
#define GPS_SEND_PCAS_CONFIG    0

/* Private variables ---------------------------------------------------------*/
/* 任务句柄（由 GPS_Task_Init 创建后赋值） */
TaskHandle_t GPSTaskHandle = NULL;

/* Private function prototypes -----------------------------------------------*/
/* 任务入口：xTaskCreate 需要在本文件内先见到声明 */
void GPS_Task_Entry(void *argument);

/* Private application code --------------------------------------------------*/

/* ---------------------------------------------------------------------------
 * 任务创建：优先级与栈大小由 freertos.c 统一传入
 * ------------------------------------------------------------------------- */
void GPS_Task_Init(osPriority_t prio, uint32_t stack)
{
  BaseType_t ret = xTaskCreate(GPS_Task_Entry,
                               "GPS",
                               (configSTACK_DEPTH_TYPE)stack,
                               NULL,
                               (UBaseType_t)prio,
                               &GPSTaskHandle);
  /* 堆不足时 xTaskCreate 返回 pdFAIL，任务会静默不运行，必须断言拦住 */
  configASSERT(ret == pdPASS);
}

/* ---------------------------------------------------------------------------
 * 任务入口
 * ------------------------------------------------------------------------- */
void GPS_Task_Entry(void *argument)
{
  (void)argument;

  /* USER CODE BEGIN GPS_Task_Entry_Init */
  /* 绑定串口并启动「循环 DMA + 空闲中断」接收。
     必须在 CubeMX 里给该串口配好 RX DMA（Circular）与全局中断，
     否则 HAL_UARTEx_ReceiveToIdle_DMA() 会返回 HAL_ERROR。 */
  GPS_Init(GPS_UART_HANDLE);
  /* USER CODE END GPS_Task_Entry_Init */

  /* 等模块上电稳定后再发配置命令（GPS_SEND_PCAS_CONFIG=0 时整段不发，
     只安静接收 —— 见文件顶部该开关的说明） */
#if GPS_SEND_PCAS_CONFIG
  vTaskDelay(pdMS_TO_TICKS(GPS_CONFIG_DELAY_MS));
  GPS_ConfigMinimal();
  vTaskDelay(pdMS_TO_TICKS(200));
  GPS_ConfigMinimal();      /* 再发一次：模块上电初期可能丢首条命令 */
#else
  (void)GPS_CONFIG_DELAY_MS;
#endif

  /* -------------------------------------------------------------------------
   * 语句集自动修复
   * -------------------------------------------------------------------------
   * 现象：串口有字节进来（RX 在涨）、校验也没错，但**一条 GGA/RMC 都没有**
   *       —— 第 1 页的语句普查里 RMC/GGA 恒为 0，而经纬度恰恰只在这两种语句里。
   * 原因：模块的输出语句集被改过（多半是之前在 PC 上用 sscom 发过 $PCAS03），
   *       这类配置保存在模块的备电 RAM 里，断电重上也不一定恢复默认。
   * 处理：检测到这种状态就自动发一次"全部语句打开"，最多试 3 次、每次间隔 5 s。
   *       发完不影响别的：若模块本来就正常，这个分支永远不会进。
   * ------------------------------------------------------------------------- */
  {
    uint8_t  tries = 0U;
    uint32_t t_last = HAL_GetTick();

    for (;;)
    {
      /* USER CODE BEGIN GPS_Task_Entry_Loop */
      GPS_Process();

      if (GPS_GetLastRxAge(HAL_GetTick()) > GPS_RX_TIMEOUT_MS)
      {
        /* 模块没上电 / 接线松 / 串口被复位：重新武装一次接收。
           GPS_Reinit() 内部先 Abort 再启动，重复调用是安全的。 */
        GPS_Reinit();
      }

      /* ---- 语句集自动修复（见上面的说明）---- */
      if ((tries < 3U) &&
          (GPS_GetRxBytes() > 0U) &&
          (GPS_GetTypeCount(GPS_TYPE_RMC) == 0U) &&
          (GPS_GetTypeCount(GPS_TYPE_GGA) == 0U) &&
          ((uint32_t)(HAL_GetTick() - t_last) > 5000U))
      {
        GPS_ConfigAll();      /* $PCAS03 全部开启 → 恢复 GGA/RMC/GSV/GSA */
        tries++;
        t_last = HAL_GetTick();
      }

      /* 需要时在此刷新 OLED 状态页或链路质量统计：
       *   GPS_FormatPosition(line, sizeof(line), HAL_GetTick());
       *   GPS_GetSentenceCount() / GPS_GetCsumErrCount() / GPS_GetOverrunCount()
       * 当前故意不直接刷屏，避免与 oled_task 争软件 I2C。 */
      /* USER CODE END GPS_Task_Entry_Loop */

      vTaskDelay(pdMS_TO_TICKS(GPS_TASK_PERIOD_MS));
    }
  }
}
