/* USER CODE BEGIN Header */
#include "OLED.h"
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * 本文件只负责：
  *   1) 集中定义各任务的优先级与栈大小
  *   2) 在 MX_FREERTOS_Init() 里调用各任务的 xxx_Task_Init()
  *
  * 任务的具体实现与创建都在 icode\task\ 下，本文件不写任何任务逻辑。
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imu_recv_task.h"
#include "tinyml_task.h"
#include "fatigue_task.h"
#include "alarm_task.h"
#include "ble_task.h"
#include "oled_task.h"
#include "gps_task.h"
#include "menu_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ===========================================================================
 *                        任务优先级与栈大小（集中配置）
 *
 * 优先级取 osPriority_t 枚举，数值越大优先级越高：
 *   osPriorityLow=8   osPriorityBelowNormal=16  osPriorityNormal=24
 *   osPriorityAboveNormal=32   osPriorityHigh=40   osPriorityRealtime=48
 *
 * 当前分配理由：
 *   IMU_Recv  AboveNormal  数据先落地，优先级低了会丢帧
 *   Alarm     AboveNormal  取消按键必须响应及时，晚了就误报
 *   TinyML    Normal       推理耗时可控，单次目标 <20ms
 *   BLE       Normal       上报允许有几十毫秒延迟
 *   GPS       BelowNormal  1 Hz NMEA 流，20 ms 排空一次缓冲即可，不急
 *   Fatigue   BelowNormal  1~2Hz 低频任务，不急
 *   OLED      Low          软件I2C刷屏最慢，绝不能抢 CPU
 * ========================================================================= */
#define IMU_RECV_TASK_PRIO      osPriorityAboveNormal
#define ALARM_TASK_PRIO         osPriorityAboveNormal
#define TINYML_TASK_PRIO        osPriorityNormal
#define BLE_TASK_PRIO           osPriorityNormal
#define GPS_TASK_PRIO           osPriorityBelowNormal
#define FATIGUE_TASK_PRIO       osPriorityBelowNormal
#define OLED_TASK_PRIO          osPriorityLow

/* 栈深度，单位 word（1 word = 4 字节）
   实测经验：纯逻辑任务 256 word 够用；OLED 刷屏缓冲大，给 512 word。
   改大栈要同步调大 FreeRTOSConfig.h 的 configTOTAL_HEAP_SIZE。

   TINYML 给 384 word（1536 字节）而非 256，原因：TinyML 任务调用链里
   ML_ExtractFeatures 的两遍扫描局部变量 + 29 维特征数组(116B) + int8 输入
   输出缓冲，合计约 500 字节；256 word 虽勉强够，但余量太小，一旦以后加了
   队列接收或浮点打印就会溢出，而栈溢出在 FreeRTOS 上表现为莫名其妙的
   硬件错误，极难排查。多这 512 字节换确定性，值得。
   注意：窗口环形缓冲(2412B)是 file-static，不占任务栈。 */
#define IMU_RECV_TASK_STACK     128
#define TINYML_TASK_STACK       384
/* ★ 下面三个栈由 256 → 192 word，是【恢复 GPS 任务】腾堆用的（-768 B）：
   fatigue/alarm/ble 三个任务当前都不做重活（alarm 只跑 Key_Scan + 状态机，
   fatigue 与 ble 仍是桩），192 word = 768 B 余量充足。
   注意：这三项与 IMU 链路无关，IMU 的栈/驱动一行未改。 */
#define FATIGUE_TASK_STACK      192
#define ALARM_TASK_STACK        192
#define BLE_TASK_STACK          192
/* GPS 任务只做环形缓冲排空 + 少量整数解析，96 字节的语句缓冲是 file-static，
   不占任务栈，128 word（512 B）够用。 */
#define GPS_TASK_STACK          128
/* 菜单任务：只做翻页 + 汇总各子系统快照（双缓冲约 180 B 静态），
   全是整数运算与 memcpy，192 word（768 B）足够。
   优先级放在 OLED(Low) 之上、告警/BLE 之下，保证菜单状态比屏幕更新。 */
#define MENU_TASK_PRIO          osPriorityBelowNormal
#define MENU_TASK_STACK         192
/* OLED 显存缓冲是 OLED.c 里的 file-static 数组，不吃任务栈，
   刷屏本身不递归、不放大局部变量；GPS 接收页的语句原文与定位快照也放 file-static，
   128 word（512 B）经实测够用 */
#define OLED_TASK_STACK         128
/* ---------------------------------------------------------------------------
 * ⚠️ 下面三个栈大小是被【RAM 硬约束】压下来的，不是随手写的数
 * ---------------------------------------------------------------------------
 * 实测 (armlink map)：
 *   RAM 总量 20480 B，启动栈预留 1024 B → 可用 19456 B。
 *   未加 DIY IMU 模块前已用约 19400 B，几乎顶满。
 *   加入 diy_imu 驱动后总量 20312 B，超出 856 B。
 *
 * FreeRTOS 堆 10240 B 不能再砍：9 个任务(7 应用 + IDLE + Tmr Svc)的
 *   栈+TCB 已用 9712 B，仅余 528 B。再砍会静默创建任务失败，
 *   而 configASSERT 是关中断死循环，屏幕上不会有任何提示。
 *
 * 所以只能从任务栈里省，三项合计 856 B：
 *   IMU_Recv  256 -> 128 word   该任务只做「取帧 + 12 字节拷贝 + 计数」，
 *                               调用链浅且不递归，128 word 有余量
 *   OLED      256 -> 128 word   刷屏不递归；显存与帧/快照缓冲都在 file-static
 *                               或本文件栈上，128 word 经实测够用
 *   Tmr Svc   128 ->  64 word   在 FreeRTOSConfig.h，本工程无任何软件定时器
 *
 * 改动依据与复现方式见 docs/DIY_IMU接入说明.md。
 * 若将来再往上加任务或缓冲，务必先重算这份账，别直接调大。
 * ------------------------------------------------------------------------- */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  OLED_Init();
  OLED_ShowString(1, 1, "OS Loading...", OLED_6X8);
  OLED_Update();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* 说明：CubeMX 默认生成的 defaultTask（只做 osDelay(1000) 空转）已删除，
     省下 512 B 任务栈 + 约 90 B TCB —— configTOTAL_HEAP_SIZE 只有 12288 B，
     而 7 个应用任务 + 空闲/定时器任务已经占掉约 11 KB，这 600 B 很关键。 */

  /* USER CODE BEGIN RTOS_THREADS */
  IMU_Recv_Task_Init(IMU_RECV_TASK_PRIO, IMU_RECV_TASK_STACK);
  TinyML_Task_Init(TINYML_TASK_PRIO, TINYML_TASK_STACK);
  Fatigue_Task_Init(FATIGUE_TASK_PRIO, FATIGUE_TASK_STACK);
  Alarm_Task_Init(ALARM_TASK_PRIO, ALARM_TASK_STACK);
  BLE_Task_Init(BLE_TASK_PRIO, BLE_TASK_STACK);
  /* ---------------------------------------------------------------------
   * ★ GPS 任务已恢复创建（2026-10-13）
   * ---------------------------------------------------------------------
   * 背景：本板三路串口分工已定 —— USART1=GPS、USART2=HC-08、USART3=DIY IMU，
   *       三个外设都要用，所以 GPS 任务必须真的跑起来（此前为腾 RAM 被注释掉）。
   *
   * 腾堆方式（**完全没有动 IMU 那一侧**：imu_recv_task.c / diy_imu.c / USART3
   * 的配置与栈大小一行未改）：
   *     FATIGUE_TASK_STACK  256 -> 192 word   -256 B
   *     ALARM_TASK_STACK    256 -> 192 word   -256 B
   *     BLE_TASK_STACK      256 -> 192 word   -256 B
   *     configTIMER_QUEUE_LENGTH 10 -> 4      -96 B（本工程无任何软件定时器）
   *   合计腾出约 864 B；GPS 任务本身只要 512 B 栈 + 约 100 B TCB（约 612 B），
   *   所以恢复后堆占用比恢复前还低约 250 B。
   *
   * ⚠ 另外说明一处历史记账：注释里曾写"9 个任务已占 9712 B"。按本工程配置，
   *   空闲任务与定时器任务的栈/TCB 是 cmsis_os2.c 里的【静态数组】
   *   （从 armlink map 可验证），**不占 FreeRTOS 堆**；堆里只有 7 个应用任务的
   *   栈(6144 B) + 7 个 TCB(约 700 B) + 定时器队列(约 250 B) ≈ 7.1 KB。
   *   真实余量以开机时 OLED 上的 FreeHeap 数字为准（那一行就是为此加的）。
   * --------------------------------------------------------------------- */
  GPS_Task_Init(GPS_TASK_PRIO, GPS_TASK_STACK);
  /* 菜单/状态总览任务：上/下键翻页，汇总三条链路 + 系统状态供 OLED 显示。
     必须在 OLED 之前创建——OLED 首帧就会去读菜单视图。 */
  Menu_Task_Init(MENU_TASK_PRIO, MENU_TASK_STACK);
  OLED_Task_Init(OLED_TASK_PRIO, OLED_TASK_STACK);

  /* 堆余量自检：任务栈全部从 configTOTAL_HEAP_SIZE 动态分配。
     若某个任务"创建了却从不运行"，先看这个数字——接近 0 就是堆不够。
     注意：xTaskCreate 失败只返回 pdFAIL，而当前 configASSERT 是关中断死循环，
     屏幕上不会给出任何提示，所以这个数字是排查堆问题的第一手依据。 */
  OLED_ShowString(0, 16, "FreeHeap:", OLED_6X8);
  OLED_ShowNum(54, 16, (uint32_t)xPortGetFreeHeapSize(), 5, OLED_6X8);
  OLED_Update();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* 任务实现已移至 MDK-ARM/icode/task 各模块。
   CubeMX 原先自动生成的 StartDefaultTask 已删除（空转任务，纯粹浪费 512 B 栈）。 */
/* USER CODE END Application */
