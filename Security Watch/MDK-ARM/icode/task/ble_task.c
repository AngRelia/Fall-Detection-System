/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ble_task.c
  * @brief   BLE 上报任务（HC-08 透传）
  ******************************************************************************
  * 数据流：
  *   其它模块 --BLE_Report(json)--> 单槽暂存 --本任务--> HC08_SendLine() --USART2--> HC-08 --> 手机
  *   手机 --> HC-08 --USART2(空闲中断)--> hc08 驱动环形缓冲 --本任务--> 命令解析
  *
  * 设计取舍：
  *   1) 待发报文用**单槽暂存**而不是队列：告警是低频事件（20 s 确认窗口 + 120 s 冷却），
  *      两帧同时到达的概率极低；本板静态 RAM 只剩不到 1 KB，多槽会白占 RAM。
  *      槽满时丢弃并计数（BLE_GetStats 的 drop），不会阻塞告警任务。
  *   2) 发送阻塞 200 B 约 210 ms（9600 波特率）：低频事件可接受，免去一套发送环形缓冲。
  *   3) 不判断"手机是否已连接"再发：没有 STATE 脚时无从判断，且未连接时数据只是进空气，
  *      不会造成危害；接了 STATE 脚时以 HC08_IsConnected() 显示在屏幕上。
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "ble_task.h"
#include "hc08.h"
#include "gps.h"
#include <string.h>

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 绑定的串口：USART2（PA2=TX / PA3=RX），CubeMX 里已配 9600 8N1 + NVIC */
extern UART_HandleTypeDef huart2;
#define BLE_UART_HANDLE         (&huart2)

/* 任务节拍：BLE 数据率低，20 ms 排空一次接收缓冲绰绰有余 */
#define BLE_TASK_PERIOD_MS      20U

/* 上电后等模块稳定再发 AT（模块上电初始化需要一点时间） */
#define BLE_AT_DELAY_MS         1500U

/* ★ 上电是否发 AT 探测（`AT` → 期望回 `OK`）。
   这是判断"MCU↔HC-08 串口链路通不通"的最直接手段：
   屏幕上 `ATOK` 计数从 0 变 1 就说明通了。首次联调保持 1。 */
#define BLE_SEND_AT_PROBE       1

/* ★ 是否在探测后改蓝牙名（`AT+NAME...`）。默认 0：
   改名会写入模块内部配置，先确认链路通了再改，避免一次改错后要连 PC 恢复。 */
#define BLE_SET_NAME            0
#define BLE_DEV_NAME            "JBX-01"

/* 上电是否发一条 BOOT 报文（手机 App 里能立刻看到，最直观的"通了"证据） */
#define BLE_SEND_BOOT_MSG       1

/* 是否把手机发来的每一行回显给手机（联调时验证双向；量产可关掉省流量） */
#define BLE_ECHO_RX             1

/* 待发报文暂存槽大小：完整告警报文约 232 字节
   （{"t":..,"dev":..,"ev":"FALL","conf":..,"acc":[..],"rpy":[..],"step":..,"seq":..,
     "gps":{...109 字节...}} —— 实测按 256 留余量，不够会被计入 drop 计数） */
#define BLE_TX_STAGE_SIZE       256U
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
TaskHandle_t BLETaskHandle = NULL;

/* 待发报文暂存（单槽 + 待发标志）：生产者是其它任务，消费者是本任务 */
static char             s_tx_stage[BLE_TX_STAGE_SIZE];
static volatile uint8_t s_tx_pending = 0U;
static uint32_t         s_tx_ok = 0U;
static uint32_t         s_tx_drop = 0U;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void BLE_Task_Entry(void *argument);
static void BLE_HandleCommand(const char *cmd);
/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* ---------------------------------------------------------------------------
 * 任务创建：优先级与栈大小由 freertos.c 统一传入
 * ------------------------------------------------------------------------- */
void BLE_Task_Init(osPriority_t prio, uint32_t stack)
{
  BaseType_t ret = xTaskCreate(BLE_Task_Entry,
                               "BLE",
                               (configSTACK_DEPTH_TYPE)stack,
                               NULL,
                               (UBaseType_t)prio,
                               &BLETaskHandle);
  /* 堆不足时 xTaskCreate 返回 pdFAIL，任务会静默不运行，必须断言拦住 */
  configASSERT(ret == pdPASS);
}

/* ---------------------------------------------------------------------------
 * 对外发送接口（供 alarm_task 等调用）
 * ------------------------------------------------------------------------- */
uint8_t BLE_Report(const char *line)
{
  uint16_t n;

  if (line == NULL)
  {
    return 0U;
  }
  if (s_tx_pending != 0U)
  {
    s_tx_drop++;                      /* 上一帧还没发出去 */
    return 0U;
  }

  n = (uint16_t)strlen(line);
  if ((n == 0U) || (n >= BLE_TX_STAGE_SIZE))
  {
    s_tx_drop++;                      /* 空串或太长（槽放不下） */
    return 0U;
  }

  (void)memcpy(s_tx_stage, line, (size_t)n + 1U);
  __DMB();                            /* 保证数据先于标志对消费者可见 */
  s_tx_pending = 1U;
  return 1U;
}

void BLE_GetStats(uint32_t *ok, uint32_t *drop)
{
  if (ok != NULL)   { *ok = s_tx_ok; }
  if (drop != NULL) { *drop = s_tx_drop; }
}

/* ---------------------------------------------------------------------------
 * 手机命令解析（透传模式下模块不解析，字节直接到 MCU）
 *   指令集刻意做得极简，后续按需扩展（协议文档见 docs/HC-08接入方案.md）
 * ------------------------------------------------------------------------- */
static void BLE_HandleCommand(const char *cmd)
{
  /* 拼接缓冲用 file-static：本任务栈只有 192 word（768 B），放栈上太奢侈。
     160 字节 = 头部 17 + GPS 片段约 109 + 收尾，够用。 */
  static char s_json[160];

#if (BLE_ECHO_RX == 1)
  /* 先回显：联调时在手机端就能看到"MCU 确实收到了什么" */
  (void)HC08_SendLine(cmd);
#endif

  if (strcmp(cmd, "PING") == 0)
  {
    (void)HC08_SendLine("{\"ev\":\"PONG\"}");
  }
  else if (strcmp(cmd, "STAT") == 0)
  {
    /* 设备状态：复用 GPS_BuildJson()，与告警报文里的 gps 片段保持同一格式 */
    const char *head = "{\"ev\":\"STAT\",\"gps\":";
    uint16_t    m = (uint16_t)strlen(head);

    (void)memcpy(s_json, head, (size_t)m);
    m = (uint16_t)(m + GPS_BuildJson(&s_json[m], (uint16_t)(sizeof(s_json) - m - 1U), HAL_GetTick()));
    if (m < (uint16_t)(sizeof(s_json) - 1U))
    {
      s_json[m] = '}';
      s_json[m + 1U] = '\0';
    }
    (void)HC08_SendLine(s_json);
  }
  else if ((cmd[0] == 'A') && (cmd[1] == 'T'))
  {
    /* 允许从手机侧转发 AT 指令（仅在模块未连接时会生效，便于台架调试） */
    (void)HC08_SendAT(cmd);
  }
  /* 其它命令：忽略（后续协议扩展位） */
}

/* ---------------------------------------------------------------------------
 * 任务入口
 * ------------------------------------------------------------------------- */
void BLE_Task_Entry(void *argument)
{
  char cmd[HC08_LINE_MAX];

  (void)argument;

  /* USER CODE BEGIN BLE_Task_Entry_Init */
  HC08_Init(BLE_UART_HANDLE);

  /* 等模块上电稳定 */
  vTaskDelay(pdMS_TO_TICKS(BLE_AT_DELAY_MS));

#if (BLE_SEND_AT_PROBE == 1)
  (void)HC08_SendAT("AT");          /* 期望回 "OK" → 屏幕 ATOK 计数 +1 */
  vTaskDelay(pdMS_TO_TICKS(200));
#endif

#if (BLE_SET_NAME == 1)
  (void)HC08_SendAT("AT+NAME" BLE_DEV_NAME);   /* 写入模块配置，确认链路后再开 */
  vTaskDelay(pdMS_TO_TICKS(200));
#endif

#if (BLE_SEND_BOOT_MSG == 1)
  (void)HC08_SendLine("{\"dev\":\"JBX-01\",\"ev\":\"BOOT\"}");
#endif
  /* USER CODE END BLE_Task_Entry_Init */

  for (;;)
  {
    /* USER CODE BEGIN BLE_Task_Entry_Loop */
    /* 1) 排空接收缓冲、拼行 */
    HC08_Process();

    /* 2) 有待发报文就发出去（不阻塞其它任务：这里只花发送时间） */
    if (s_tx_pending != 0U)
    {
      if (HC08_SendLine(s_tx_stage) != 0U)
      {
        s_tx_ok++;
      }
      else
      {
        s_tx_drop++;
      }
      s_tx_pending = 0U;
    }

    /* 3) 处理手机下发的命令 */
    if (HC08_TakeLine(cmd, sizeof(cmd)) != 0U)
    {
      BLE_HandleCommand(cmd);
    }

    /* 4) 串口异常自愈：长时间一个字节都没收到且模块也未连接时重新武装接收。
          注意：手机没连接时本来就不该有数据，所以这里只做"极端长时间无数据"的兜底，
          不会干扰正常收发。 */
    /* （HC08_Init 里的 ReceiveToIdle_IT 由回调自行续接，通常不需要重装；
        若现场发现偶发收不到，可在此加计数并调用 HC08_Init(BLE_UART_HANDLE) 重装。） */
    /* USER CODE END BLE_Task_Entry_Loop */

    vTaskDelay(pdMS_TO_TICKS(BLE_TASK_PERIOD_MS));
  }
}

/* USER CODE END Application */
