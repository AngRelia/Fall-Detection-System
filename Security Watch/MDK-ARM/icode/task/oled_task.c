/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    oled_task.c
  * @brief   OLED 显示任务 —— GPS 接收页（当前）/ 按键检测页 / DIY IMU 链路状态页
  ******************************************************************************
  * ★ 本文件里同时存在三个屏幕，用下面的 OLED_SCREEN_SEL 开关切换：
  *     OLED_SCREEN_IMU(0)     = DIY IMU 链路状态页（三页轮换：原始量/姿态角/物理量）
  *     OLED_SCREEN_KEYTEST(1) = 按键检测页（验证 4 个按键与接线）
  *     OLED_SCREEN_GPS(2)     = GPS 接收页（**当前**，验证 NMEA 接收与解析）
  *   另外两套绘图代码都用 #if 包着、未被删改，改回对应编号即可恢复。
  *
  * 按键检测页布局（128x64，6x8 字体，每行 21 字符）：
  *   y=0   KEY TEST        EV B-LNG      最近一次事件名
  *   y=8   1 UP   [ ]  N 0000  L 00      KEY_1：按下框 / 按下次数 / 长按次数
  *   y=16  2 DOWN [ ]  N 0000  L 00
  *   y=24  3 OK   [ ]  N 0000  L 00
  *   y=32  4 BACK [ ]  N 0000  L 00
  *   y=40  RAW 1 1 1 1    HLD 0000ms   接线检查：1=松开 0=键按下；右侧当前按住时长
  *   y=48  SCAN 0000000   EVT 000      扫描调用次数（应约 100/s 增长）/ 累计事件数
  *   y=56  HEAP 0000000              系统可用堆（字节）
  *
  * 为什么 SCAN 那一行最重要：它直接反映"Key_Scan() 的调用者是否在跑"。
  * Key_Scan() 由 alarm_task 每 10 ms 调一次；这个数字不动，说明扫描方没运行，
  * 屏幕上其它计数当然也不会变 —— 一眼就能分辨"按键坏了"还是"扫描没跑"。
  *
  * IMU 页职责（开关置 0 时生效）：
  *   1) 显示 IMU 链路实测帧率、CRC 错误、丢帧数（联调期最有用）
  *   2) 显示最近一帧的加速度 / 角速度（原始计数 + 工程量对照）
  *   3) 显示 flags 状态位，便于判断节点A的融合状态
  *
  * 设计要点：
  *   - OLED.c 是软件 I2C，整屏刷新耗时可观，所以 10 Hz 刷新足够
  *   - 本任务优先级最低（osPriorityLow），绝不抢 imu_recv_task 的 CPU
  *   - 刷屏只在本任务内做，避免多任务竞争 I2C 总线
  *   - ★ 刻意【不用】OLED_Printf：它内部走 vsnprintf，调用链深。
  *     改用 OLED_ShowNum + 定点整数拆分，调用链浅，栈占用可预测。
  *
  * IMU 页画面布局（开关置 0 时）：
  *   row0  IMU  100Hz  OK            帧率 / 链路状态
  *   row1  A -2  395 1993            加速度原始计数
  *   row2    -1  193  973   mg       换算后的 mg
  *   row3  G 21 -11 -10              角速度原始计数
  *   row4     13  -7  -6  0.1d/s     换算后的 0.1°/s
  *   row5  |a| 992mg                 合加速度
  *   row6  CRC 0  DROP 0             诊断
  *   row7  RS 0  BR 0  RO 0          重同步 / 字节环丢 / 帧环覆盖
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "oled_task.h"
#include "cmsis_os.h"
#include "queue.h"
#include "OLED.h"
#include "imu_recv_task.h"
#include "diy_imu.h"
#include "key.h"
#include "gps.h"
#include "hc08.h"
#include "ble_task.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ★ 屏幕选择（三个屏幕共用同一套任务和 10 Hz 刷新节拍）
     0 = DIY IMU 链路状态页（三页轮换：原始量 / 姿态角 / 物理量）
     1 = 按键检测页
     2 = GPS 接收页（当前）★
   切换只改下面这个宏；另外两套绘图代码用 #if 包着、未被删改，改回对应编号即可。 */
#define OLED_SCREEN_IMU         0
#define OLED_SCREEN_KEYTEST     1
#define OLED_SCREEN_GPS         2
#define OLED_SCREEN_HC08        3
/* 当前默认显示 HC-08 蓝牙页（正在联调蓝牙）。
   要看 GPS 页就把下面改成 OLED_SCREEN_GPS，要看按键页改成 OLED_SCREEN_KEYTEST。 */
#define OLED_SCREEN_SEL         OLED_SCREEN_HC08

/* 目标刷新率。整屏 OLED_Update() 在软件 I2C 下约 10~30 ms，
   10 Hz 已足够看清，且不会长时间占住 CPU。 */
#define OLED_TASK_PERIOD_MS     100U

/* 诊断数字显示宽度（固定宽度避免残影）。
   注意不能用 str 拼接，OLED_ShowNum 是定长右对齐补零的，
   所以宽度取够即可，"0" 会显示成 "  0"（前置零）—— 为可读性
   这里对可能为 0 的项用较小宽度并靠 OLED_Clear 兜底。 */
#define W_CNT                   4U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* 任务句柄（由 OLED_Task_Init 创建后赋值） */
TaskHandle_t OLEDTaskHandle = NULL;

/* 只保留分页计数器。帧与统计快照改为任务入口里的局部变量 ——
   本任务 P4 优先级每轮跑完就睡眠, 局部完全够用, 且能省下 .bss。 */

/* 显示页: 每 3 s 在 原始量 / 姿态角 / 物理量(g) 之间轮换。
   分页是为了让每屏信息更聚焦 —— 128x64 塞不下所有量还能看清。
   （仅 IMU 页用，其它页下不编译，避免"定义了但没用到"的告警） */
#if (OLED_SCREEN_SEL == OLED_SCREEN_IMU)
static uint8_t             s_page = 0;
static uint32_t            s_page_tick = 0;
#endif
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* 任务入口：xTaskCreate 需要在本文件内先见到声明 */
void OLED_Task_Entry(void *argument);

#if (OLED_SCREEN_SEL == OLED_SCREEN_KEYTEST)
/* 按键检测页：读 key.c 的只读自检接口画一屏（不消费事件，不影响菜单/告警） */
static void OLED_KeyTest_Page(void);
#elif (OLED_SCREEN_SEL != OLED_SCREEN_KEYTEST)
/* 定点显示：IMU 页与 GPS 页都用它
   value : 已放大的整数（例如 mg 值；或 0.1dps 值）
   scale : 放大倍数（1000 / 100 / 10）
   frac  : 要显示的小数位数
   例：OLED_ShowScaled(x, y, 193, 1000, 3) -> "  0.193"
       OLED_ShowScaled(x, y, -1, 1000, 3)  -> " -0.001" */
static void OLED_ShowScaled(int16_t X, int16_t Y, int32_t value,
                            int32_t scale, uint8_t frac, uint8_t width);
#endif

#if (OLED_SCREEN_SEL == OLED_SCREEN_GPS)
/* GPS 接收页 */
static void OLED_GPS_Page(void);
#endif

/* GPS 页与 HC-08 页都要用「定长写字符串」（不足补空格，避免长短变化留残影） */
#if ((OLED_SCREEN_SEL == OLED_SCREEN_GPS) || (OLED_SCREEN_SEL == OLED_SCREEN_HC08))
static void OLED_ShowChars(int16_t X, int16_t Y, const char *s, uint8_t n);
#endif

#if (OLED_SCREEN_SEL == OLED_SCREEN_HC08)
/* HC-08 蓝牙页（两页：链路 / 统计） */
static void OLED_HC08_Page(void);
#endif
/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* ---------------------------------------------------------------------------
 * 任务创建：优先级与栈大小由 freertos.c 统一传入
 * ------------------------------------------------------------------------- */
void OLED_Task_Init(osPriority_t prio, uint32_t stack)
{
  BaseType_t ret = xTaskCreate(OLED_Task_Entry,
                               "OLED",
                               (configSTACK_DEPTH_TYPE)stack,
                               NULL,
                               (UBaseType_t)prio,
                               &OLEDTaskHandle);
  /* 堆不足时 xTaskCreate 返回 pdFAIL，任务会静默不运行，必须断言拦住 */
  configASSERT(ret == pdPASS);
}

/* ---------------------------------------------------------------------------
 * 按键检测页（联调工具）
 *
 * 读的都是 key.c 的只读自检接口，**不消费事件队列**，所以菜单/告警的事件流不受影响。
 * 这一页要看四件事：
 *   1) 接线：RAW 那 4 位按下去必须变 0（内部上拉没接错、线没接反）
 *   2) 消抖与边沿：按下次数 N 是否一次按一下（抖动会乱跳）
 *   3) 长按/连发：按住返回键 1 s 看 L 计数与 EV 是否变 B-LNG；上/下键看是否连发
 *   4) 扫描是否在跑：SCAN 是否以 ~100/s 增长（不动就是 Key_Scan() 的调用者没运行）
 * ------------------------------------------------------------------------- */
#if (OLED_SCREEN_SEL == OLED_SCREEN_KEYTEST)
static void OLED_KeyTest_Page(void)
{
  /* 键名固定 4 字符：直接覆盖 4 个字符位，不会残留上一次的字 */
  static const char *kname[4] = { "UP  ", "DOWN", "OK  ", "BACK" };
  uint8_t  raw;
  uint8_t  i;
  uint16_t hold_max = 0U;
  uint16_t h;

  /* ---- 标题行 + 最近一次事件 ---- */
  OLED_ShowString(0, 0, "KEY TEST", OLED_6X8);
  OLED_ShowString(66, 0, "EV", OLED_6X8);
  OLED_ShowString(84, 0, Key_EventName(Key_GetLastEvent()), OLED_6X8);

  /* ---- 四个键：按下框 / 按下次数 / 长按次数 ---- */
  raw = Key_GetRawMask();
  for (i = 0U; i < 4U; i++)
  {
    int16_t y = (int16_t)(8 + i * 8);

    OLED_ShowNum(0, y, (uint32_t)(i + 1U), 1, OLED_6X8);          /* 键号 */
    OLED_ShowString(6, y, (char *)kname[i], OLED_6X8);            /* 键名 */
    /* 消抖后的状态用方框表示：按住 = [#]，松开 = [ ] */
    OLED_ShowString(36, y, ((Key_GetState() == (uint8_t)(i + 1U)) ? "[#]" : "[ ]"), OLED_6X8);
    OLED_ShowString(60, y, "N", OLED_6X8);
    OLED_ShowNum(72, y, Key_GetPressCount(i), 4, OLED_6X8);       /* 按下次数 */
    OLED_ShowString(102, y, "L", OLED_6X8);
    OLED_ShowNum(114, y, Key_GetLongCount(i), 2, OLED_6X8);       /* 长按次数 */
  }

  /* ---- RAW：接线检查。1 = 引脚为高（松开），0 = 被拉低（按下） ---- */
  OLED_ShowString(0, 40, "RAW", OLED_6X8);
  for (i = 0U; i < 4U; i++)
  {
    char c = (((raw >> i) & 0x01U) != 0U) ? '0' : '1';   /* 按下时显示 0 更符合"低有效"直觉 */
    OLED_ShowChar((int16_t)(24 + i * 12), 40, c, OLED_6X8);
  }

  /* ---- 当前按住时长（取四个键里最大的那个） ---- */
  for (i = 0U; i < 4U; i++)
  {
    h = Key_GetHoldMs(i);
    if (h > hold_max)
    {
      hold_max = h;
    }
  }
  OLED_ShowString(66, 40, "HLD", OLED_6X8);
  OLED_ShowNum(84, 40, hold_max, 4, OLED_6X8);
  OLED_ShowString(108, 40, "ms", OLED_6X8);

  /* ---- 扫描节拍 + 累计事件数 ---- */
  OLED_ShowString(0, 48, "SCAN", OLED_6X8);
  OLED_ShowNum(30, 48, Key_GetScanCount(), 7, OLED_6X8);
  OLED_ShowString(78, 48, "EVT", OLED_6X8);
  OLED_ShowNum(102, 48, Key_GetEventCount(), 4, OLED_6X8);

  /* ---- 系统可用堆：任务创建失败往往是堆不够，放这一行便于现场判断 ---- */
  OLED_ShowString(0, 56, "HEAP", OLED_6X8);
  OLED_ShowNum(30, 56, (uint32_t)xPortGetFreeHeapSize(), 7, OLED_6X8);
}
#elif (OLED_SCREEN_SEL != OLED_SCREEN_KEYTEST)
/* ---------------------------------------------------------------------------
 * 定点显示实现（不引 printf，调用链浅）
 *
 * 思路：把放大后的整数拆成【整数部分】和【小数部分】分别用 OLED_ShowNum 画，
 *       中间补一个小数点字符。全程只用到整型除法与取模。
 *       局部变量很少，对 256 word 的栈很友好。
 * ------------------------------------------------------------------------- */
static void OLED_ShowScaled(int16_t X, int16_t Y, int32_t value,
                            int32_t scale, uint8_t frac, uint8_t width)
{
  int32_t int_part;
  int32_t frac_part;
  int32_t div;
  uint8_t i;
  uint8_t int_w;
  uint8_t neg = 0;

  if (scale <= 0) {
    return;
  }

  if (value < 0) {
    neg = 1U;
    value = -value;
  }

  int_part  = value / scale;
  frac_part = value % scale;

  /* 整数部分占的字符数：本身位数 + 1 个符号位（正数留空以对齐） */
  int_w = (uint8_t)(width > frac ? (width - frac - 1U) : 1U);

  if (neg) {
    OLED_ShowChar(X, Y, '-', OLED_6X8);
  } else {
    OLED_ShowChar(X, Y, ' ', OLED_6X8);
  }

  OLED_ShowNum((int16_t)(X + 6), Y, (uint32_t)int_part, int_w, OLED_6X8);

  if (frac > 0U) {
    OLED_ShowChar((int16_t)(X + 6 + int_w * 6), Y, '.', OLED_6X8);

    /* 小数部分按位逐个取出，避免再用一次除法构造前导零 */
    div = 1;
    for (i = 1; i < frac; i++) {
      div *= 10;
    }
    for (i = 0; i < frac; i++) {
      uint8_t digit = (uint8_t)((frac_part / div) % 10);
      OLED_ShowChar((int16_t)(X + 6 + int_w * 6 + 6 + i * 6), Y,
                    (char)('0' + digit), OLED_6X8);
      div /= 10;
    }
  }
}

#endif /* OLED_SCREEN_SEL != OLED_SCREEN_KEYTEST */

/* ---------------------------------------------------------------------------
 * GPS 接收页（联调工具）
 *
 * 全部数据来自 gps.c 的只读接口，**不消费任何东西**，与 gps_task 互不干扰。
 * 这一页解决"到底有没有在收 GPS 数据"这个最耗时的问题，看四层：
 *   1) RX   —— 累计收到字节数：**它是 0 就别看别的了**，问题在接线/供电/波特率
 *   2) SEN/CSE/OVR —— 通过校验的语句数 / 校验错数 / 环形溢出次数
 *              CSE 一直涨 = 波特率或电平不对（收到的是噪声，凑不出合法校验和）
 *   3) FIX/Q/SAT/AGE —— 解析结果：status=A 且有卫星才算定位有效；AGE 是最后一帧定位的秒数
 *   4) LST  —— 最近一条语句原文，直接肉眼确认是不是标准 NMEA（$GNRMC / $GNGGA ...）
 *
 * 画面布局（128x64，6x8 字体，每行 21 字符）：
 *   y=0   GPS OK RX 015558 A025s     链路 / 累计字节数 / 距上次有效定位秒数
 *   y=8   SEN 00036 CSE000 OVR00     通过校验语句数 / 校验错 / 环形溢出
 *   y=16  NOF Q0 ST00 VW08 MX42      定位状态/质量/参与定位星数/可见星数/最强信噪比
 *   y=24  LAT  39.9876543N           纬度
 *   y=32  LON 116.3123456E           经度
 *   y=40  ALT    43.5m  HD  0.95     海拔 m / HDOP
 *   y=48  $GNRMC,073028.60           最近语句原文（前 21 字符）
 *   y=56  0,A,2236.40101,N,1134      后 21 字符
 *
 * 室内为什么会 NOF：GNSS 信号约 -130 dBm，穿不过混凝土/钢筋/金属镀膜玻璃。
 *   所以"在室内定不上"是正常的物理现象，不是模块或代码问题。
 *   判据看 VW/MX：室内通常 VW=0、MX=0；拿到窗外/室外 1~3 分钟后
 *   VW 会变 5~12、MX 20~45，ST≥4 且 Q=1 时 FIX 出现。
 *
 * 栈占用控制：语句原文与定位快照都用 **file-static**，不占任务栈
 * （本任务栈在 freertos.c 里只有 128 word = 512 B，必须省着用）。
 * ------------------------------------------------------------------------- */
#if ((OLED_SCREEN_SEL == OLED_SCREEN_GPS) || (OLED_SCREEN_SEL == OLED_SCREEN_HC08))
/* 定长打印 n 个字符，不足补空格；避免 OLED_ShowString 画到屏外把缓冲写乱 */
static void OLED_ShowChars(int16_t X, int16_t Y, const char *s, uint8_t n)
{
  uint8_t i;

  for (i = 0U; i < n; i++)
  {
    char c = s[i];
    if (c == '\0')
    {
      c = ' ';
    }
    OLED_ShowChar((int16_t)(X + (int16_t)i * 6), Y, c, OLED_6X8);
  }
}
#endif

#if (OLED_SCREEN_SEL == OLED_SCREEN_GPS)
static void OLED_GPS_Page(void)
{
  static GPS_Fix_t s_fix;        /* 快照（static：省任务栈） */
  static char      s_nmea[48];   /* 最近一条语句原文 */
  static uint8_t   s_page = 0U;  /* 当前页 0/1/2，用上/下键切换 */
  static uint8_t   s_page_drawn = 0xFFU;   /* 已经画在屏上的页；0xFF = 还没画过 */
  uint32_t age_ms;
  uint32_t age_s;
  uint8_t  have;
  uint8_t  has_rmc;              /* 收到过 RMC（用累计语句计数判断，可靠） */
  uint8_t  has_gga;              /* 收到过 GGA */
  Key_Event_t nav;

  /* ---- 翻页：取"导航事件"锁存（与告警的队列消费互不干扰，见 key.h）----
     上 = 上一页，下 = 下一页，循环；长按连发事件同样受理，可以快速翻。 */
  nav = Key_TakeNav();
  if ((nav == KEY_EVENT_UP) || (nav == KEY_EVENT_UP_REPEAT))
  {
    s_page = (uint8_t)((s_page + 2U) % 3U);
  }
  else if ((nav == KEY_EVENT_DOWN) || (nav == KEY_EVENT_DOWN_REPEAT))
  {
    s_page = (uint8_t)((s_page + 1U) % 3U);
  }

  /* ---- 换页时清一次整屏（关键）----
     三页每行画的字符串长度并不相同：例如第 1 页 y=48 是 21 字符的语句原文，
     第 2 页同一行是 "no RF signal"（12 字符），不整屏清就会出现"上一页的尾巴"
     残留在右半边。清的是 OLED 显存缓冲（OLED_Clear），随后统一 OLED_Update() 推屏；
     只在换页那一拍做一次，不影响 10 Hz 刷新的耗时。 */
  if (s_page != s_page_drawn)
  {
    OLED_Clear();
    s_page_drawn = s_page;
  }

  /* ---- 取数据（全部只读；gps_task 是唯一写者） ---- */
  /* ⚠ 时间基准必须用 HAL_GetTick()：gps.c 内部的时间戳/超时全部基于它。
     虽然 HAL tick 与 FreeRTOS tick 同由 SysTick 驱动，但两者起点不同
     （HAL 从 HAL_Init 起算、RTOS 从 osKernelStart 起算），混用会让年龄计算整体偏移。 */
  have   = GPS_GetFix(&s_fix);
  GPS_GetLastSentence(s_nmea, sizeof(s_nmea));
  age_ms = HAL_GetTick();
  age_s  = (have != 0U) ? ((age_ms - s_fix.stamp_ms) / 1000U) : 0U;
  if (age_s > 9999U)
  {
    age_s = 9999U;
  }
  has_rmc = (uint8_t)(GPS_GetTypeCount(GPS_TYPE_RMC) > 0U);
  has_gga = (uint8_t)(GPS_GetTypeCount(GPS_TYPE_GGA) > 0U);

  if (s_page == 0U)
  {
    /* ========= 第 1 页：链路 + 语句类型普查（LK） =========
       语句普查是关键：经纬度只在 GGA/RMC 里，若这两项恒为 0，
       说明模块的输出语句集被改乱了（gps_task 会自动发 $PCAS03 修复）。 */
    OLED_ShowString(0, 0, "GPS LINK", OLED_6X8);
    OLED_ShowString(78, 0, GPS_IsAlive(age_ms) ? "ALIVE" : "LOST ", OLED_6X8);

    OLED_ShowString(0, 8, "RX", OLED_6X8);
    OLED_ShowNum(18, 8, GPS_GetRxBytes(), 8, OLED_6X8);

    OLED_ShowString(0, 16, "RMC", OLED_6X8);
    OLED_ShowNum(24, 16, GPS_GetTypeCount(GPS_TYPE_RMC), 5, OLED_6X8);
    OLED_ShowString(60, 16, "GGA", OLED_6X8);
    OLED_ShowNum(84, 16, GPS_GetTypeCount(GPS_TYPE_GGA), 5, OLED_6X8);

    OLED_ShowString(0, 24, "GSV", OLED_6X8);
    OLED_ShowNum(24, 24, GPS_GetTypeCount(GPS_TYPE_GSV), 5, OLED_6X8);
    OLED_ShowString(60, 24, "GSA", OLED_6X8);
    OLED_ShowNum(84, 24, GPS_GetTypeCount(GPS_TYPE_GSA), 5, OLED_6X8);

    OLED_ShowString(0, 32, "OTH", OLED_6X8);
    OLED_ShowNum(24, 32, GPS_GetTypeCount(GPS_TYPE_OTH), 5, OLED_6X8);
    OLED_ShowString(60, 32, "CSE", OLED_6X8);
    OLED_ShowNum(84, 32, GPS_GetCsumErrCount(), 4, OLED_6X8);

    OLED_ShowString(0, 40, "LAST POS SENT", OLED_6X8);
    /* 用"最近一条 RMC/GGA"而不是"最近一条任意语句"：GPTXT 实测占了 1478 条里的
       632 条，用后者排查字段问题时永远看不到定位语句。 */
    GPS_GetLastPosSentence(s_nmea, sizeof(s_nmea));
    OLED_ShowChars(0, 48, s_nmea, 21U);
  }
  else if (s_page == 1U)
  {
    /* ============ 第 2 页：卫星 / 天线（SAT） ============ */
    OLED_ShowString(0, 0, "GPS SATELLITE", OLED_6X8);

    OLED_ShowString(0, 8, "FIX", OLED_6X8);
    OLED_ShowString(30, 8, ((have != 0U) && (s_fix.valid != 0U)) ? "YES" : "NO ", OLED_6X8);
    OLED_ShowString(60, 8, "Q", OLED_6X8);
    OLED_ShowNum(66, 8, (uint32_t)(have ? s_fix.quality : 0U), 1, OLED_6X8);

    /* ST：参与定位的卫星数（GGA）；VW：可见卫星数（GSV）；MX：最强信噪比 dBHz */
    OLED_ShowString(0, 16, "ST", OLED_6X8);
    OLED_ShowNum(24, 16, (uint32_t)(have ? s_fix.sats : 0U), 2, OLED_6X8);
    OLED_ShowString(42, 16, "used", OLED_6X8);

    OLED_ShowString(0, 24, "VW", OLED_6X8);
    OLED_ShowNum(24, 24, (uint32_t)(have ? s_fix.sats_view : 0U), 2, OLED_6X8);
    OLED_ShowString(42, 24, "view", OLED_6X8);

    OLED_ShowString(0, 32, "MX", OLED_6X8);
    OLED_ShowNum(24, 32, (uint32_t)(have ? s_fix.snr_max : 0U), 2, OLED_6X8);
    OLED_ShowString(42, 32, "dBHz", OLED_6X8);

    OLED_ShowString(0, 40, "AGE", OLED_6X8);
    OLED_ShowNum(24, 40, age_s, 4, OLED_6X8);
    OLED_ShowString(54, 40, "s", OLED_6X8);

    /* y=48：原始定位字段（排查时最有用的一行）
       RMC 后的字符 = RMC 的 status 字段：A 有效 / V 无效 / - 说明**根本没收到 RMC**
       GGA 后的数字 = GGA 的 quality 字段：0 无效 / 1 单点 / 2 差分 / - 说明没收到 GGA
       定位是否有效看的是这两者之一（见 gps.c 的 GPS_FixCommit 注释）。 */
    OLED_ShowString(0, 48, "RMC", OLED_6X8);
    OLED_ShowChar(24, 48, (has_rmc != 0U) ? ((s_fix.status != 0U) ? 'A' : 'V') : '-', OLED_6X8);
    OLED_ShowString(60, 48, "GGA", OLED_6X8);
    if (has_gga != 0U)
    {
      OLED_ShowNum(84, 48, (uint32_t)s_fix.quality, 1, OLED_6X8);
    }
    else
    {
      OLED_ShowChar(84, 48, '-', OLED_6X8);
    }
  }
  else
  {
    /* ================= 第 3 页：位置（POS） =================
       ⚠ 注意 GPS_GetFix() 的语义：它只管"seqlock 读到一致快照"，开机时 seq=0
         就已满足，所以**它一开始就返回 1**。若不加判断，没数据时这一页会显示
         0000.0000000，看起来像"定位到 0°0′"，极易误判。
       因此这里用**语句计数**来判断"位置数据到底来没来过"：
         经纬度只存在于 GGA/RMC，GSV/GSA 里没有 —— 两个计数都是 0 就明确显示
         "no RMC/GGA"，而不是给一堆 0。 */
    uint8_t has_pos = (uint8_t)((GPS_GetTypeCount(GPS_TYPE_RMC) > 0U) ||
                                (GPS_GetTypeCount(GPS_TYPE_GGA) > 0U));

    OLED_ShowString(0, 0, "GPS POSITION", OLED_6X8);

    OLED_ShowString(0, 8, "LAT", OLED_6X8);
    OLED_ShowString(0, 16, "LON", OLED_6X8);
    if (has_pos == 0U)
    {
      /* 定长提示，之后来了数字会完整覆盖（数字更长），不会留残影 */
      OLED_ShowString(36, 8, "no RMC/GGA", OLED_6X8);
      OLED_ShowString(36, 16, "waiting...", OLED_6X8);
    }
    else
    {
      OLED_ShowScaled(36, 8, s_fix.lat_e7, 10000000, 7, 12);
      OLED_ShowString(114, 8, (s_fix.lat_e7 < 0) ? "S" : "N", OLED_6X8);
      OLED_ShowScaled(36, 16, s_fix.lon_e7, 10000000, 7, 12);
      OLED_ShowString(114, 16, (s_fix.lon_e7 < 0) ? "W" : "E", OLED_6X8);
    }

    OLED_ShowString(0, 24, "ALT", OLED_6X8);
    OLED_ShowScaled(36, 24, has_pos ? s_fix.alt_cm : 0, 100, 1, 7);
    OLED_ShowString(84, 24, "m", OLED_6X8);

    OLED_ShowString(0, 32, "HDOP", OLED_6X8);
    OLED_ShowScaled(36, 32, has_pos ? (int32_t)s_fix.hdop_x100 : 0, 100, 2, 6);

    OLED_ShowString(0, 40, "SPD", OLED_6X8);
    /* cm/s → km/h，保留 1 位小数：km/h = cm/s × 36 / 1000，再 ×10 定点 */
    OLED_ShowScaled(36, 40, has_pos ? (int32_t)(((uint32_t)s_fix.speed_cms * 36U) / 100U) : 0, 10, 1, 6);
    OLED_ShowString(84, 40, "km/h", OLED_6X8);

    OLED_ShowString(0, 48, "UTC", OLED_6X8);
    if ((has_pos == 0U) || (s_fix.utc_ms == 0U))
    {
      OLED_ShowString(30, 48, "no time", OLED_6X8);
    }
    else
    {
      OLED_ShowNum(30, 48, (s_fix.utc_ms / 3600000U), 2, OLED_6X8);
      OLED_ShowChar(42, 48, ':', OLED_6X8);
      OLED_ShowNum(48, 48, ((s_fix.utc_ms / 60000U) % 60U), 2, OLED_6X8);
      OLED_ShowChar(60, 48, ':', OLED_6X8);
      OLED_ShowNum(66, 48, ((s_fix.utc_ms / 1000U) % 60U), 2, OLED_6X8);
    }
  }

  /* ---- 最后一行：翻页提示 + 页码（三页共用） ---- */
  OLED_ShowString(0, 56, "UP/DN=PAGE", OLED_6X8);
  OLED_ShowNum(78, 56, (uint32_t)(s_page + 1U), 1, OLED_6X8);
  OLED_ShowString(84, 56, "/3", OLED_6X8);
}
#endif /* OLED_SCREEN_SEL == OLED_SCREEN_GPS */

/* ---------------------------------------------------------------------------
 * HC-08 蓝牙页（联调工具，两页，用上/下键切换）
 *
 * 全部数据来自 hc08.c / ble_task.c 的只读接口，不消费任何东西。
 * 这一页解决三个问题：
 *   1) MCU ↔ HC-08 的串口通不通？ → 看 `AT` 探测的应答：`ATOK` 从 0 变 1 就通了
 *      （模块收到 `AT` 会回 `OK`；`ATERR` 涨说明回了 ERROR/FAIL）
 *   2) 手机连上了没有？ → `LINK CONN/NOT CONN`（需要接 STATE 引脚，见接线表）
 *   3) 报文发出去了没有？ → `TXB` 字节数 + `SEND`/`DROP` 计数，以及最近收到的行
 *
 * 画面布局（128x64，6x8，每行 21 字符）：
 *   第 1 页（链路）                      第 2 页（统计）
 *   HC-08 BLE       1/2                  HC-08 STAT      2/2
 *   LINK  NOT CONN                       RXB    00000000
 *   TXB   00000000                       ATOK   0000
 *   AT    0000                           ATERR  0000
 *   LAST RX LINE                         OVR    0000
 *   OK                                   SEND   0000
 *   UP/DN=PAGE 1/2                       DROP   0000
 * ------------------------------------------------------------------------- */
#if (OLED_SCREEN_SEL == OLED_SCREEN_HC08)
static void OLED_HC08_Page(void)
{
  static char s_line[HC08_LINE_MAX + 1];   /* 最近收到的一行（static：省任务栈） */
  static uint8_t s_page = 0U;
  static uint8_t s_page_drawn = 0xFFU;
  uint32_t tx_ok = 0U, tx_drop = 0U;
  Key_Event_t nav;

  /* 翻页：与 GPS 页同一套导航锁存（不占用告警的事件队列） */
  nav = Key_TakeNav();
  if ((nav == KEY_EVENT_UP) || (nav == KEY_EVENT_UP_REPEAT))
  {
    s_page = (uint8_t)((s_page + 1U) % 2U);
  }
  else if ((nav == KEY_EVENT_DOWN) || (nav == KEY_EVENT_DOWN_REPEAT))
  {
    s_page = (uint8_t)((s_page + 1U) % 2U);
  }

  /* 换页清屏：两页各行长度不同，不清会留上一页的尾巴 */
  if (s_page != s_page_drawn)
  {
    OLED_Clear();
    s_page_drawn = s_page;
  }

  BLE_GetStats(&tx_ok, &tx_drop);

  if (s_page == 0U)
  {
    OLED_ShowString(0, 0, "HC-08 BLE", OLED_6X8);
    OLED_ShowString(72, 0, "1/2", OLED_6X8);

    OLED_ShowString(0, 8, "LINK", OLED_6X8);
    OLED_ShowString(36, 8, (HC08_IsConnected() != 0U) ? "CONN" : "NOT CONN", OLED_6X8);

    OLED_ShowString(0, 16, "TXB", OLED_6X8);
    OLED_ShowNum(36, 16, HC08_GetTxBytes(), 8, OLED_6X8);

    OLED_ShowString(0, 24, "AT", OLED_6X8);
    OLED_ShowNum(36, 24, HC08_GetAtTxCount(), 4, OLED_6X8);
    OLED_ShowString(72, 24, "ATOK", OLED_6X8);
    OLED_ShowNum(108, 24, HC08_GetAtOkCount(), 3, OLED_6X8);

    OLED_ShowString(0, 32, "LAST RX LINE", OLED_6X8);
    (void)HC08_GetLastLine(s_line, sizeof(s_line));
    OLED_ShowChars(0, 40, s_line, 21U);
  }
  else
  {
    OLED_ShowString(0, 0, "HC-08 STAT", OLED_6X8);
    OLED_ShowString(72, 0, "2/2", OLED_6X8);

    OLED_ShowString(0, 8, "RXB", OLED_6X8);
    OLED_ShowNum(36, 8, HC08_GetRxBytes(), 8, OLED_6X8);

    OLED_ShowString(0, 16, "ATOK", OLED_6X8);
    OLED_ShowNum(36, 16, HC08_GetAtOkCount(), 4, OLED_6X8);

    OLED_ShowString(0, 24, "ATERR", OLED_6X8);
    OLED_ShowNum(36, 24, HC08_GetAtErrCount(), 4, OLED_6X8);

    OLED_ShowString(0, 32, "OVR", OLED_6X8);
    OLED_ShowNum(36, 32, HC08_GetOverrunCount(), 4, OLED_6X8);

    OLED_ShowString(0, 40, "SEND", OLED_6X8);
    OLED_ShowNum(36, 40, tx_ok, 5, OLED_6X8);
    OLED_ShowString(72, 40, "DROP", OLED_6X8);
    OLED_ShowNum(108, 40, tx_drop, 3, OLED_6X8);
  }

  OLED_ShowString(0, 56, "UP/DN=PAGE", OLED_6X8);
  OLED_ShowNum(78, 56, (uint32_t)(s_page + 1U), 1, OLED_6X8);
  OLED_ShowString(84, 56, "/2", OLED_6X8);
}
#endif /* OLED_SCREEN_SEL == OLED_SCREEN_HC08 */


/* ---------------------------------------------------------------------------
 * 任务入口
 * ------------------------------------------------------------------------- */
void OLED_Task_Entry(void *argument)
{
  (void)argument;

  /* USER CODE BEGIN OLED_Task_Entry_Init */
  /* 清掉 MX_FREERTOS_Init() 里的 "OS Loading..." 启动提示 */
  OLED_Clear();
#if (OLED_SCREEN_SEL == OLED_SCREEN_KEYTEST)
  OLED_ShowString(0, 0, "KEY TEST", OLED_6X8);
#elif (OLED_SCREEN_SEL == OLED_SCREEN_GPS)
  OLED_ShowString(0, 0, "GPS receiving...", OLED_6X8);
#elif (OLED_SCREEN_SEL == OLED_SCREEN_HC08)
  OLED_ShowString(0, 0, "HC-08 BLE...", OLED_6X8);
#else
  OLED_ShowString(0, 0, "IMU link monitoring", OLED_6X8);
#endif
  OLED_Update();

  /* USER CODE END OLED_Task_Entry_Init */

  for (;;)
  {
    /* USER CODE BEGIN OLED_Task_Entry_Loop */
#if (OLED_SCREEN_SEL == OLED_SCREEN_KEYTEST)
    /* ---------------- 按键检测页 ---------------- */
    OLED_KeyTest_Page();
#elif (OLED_SCREEN_SEL == OLED_SCREEN_GPS)
    /* ---------------- GPS 接收页 ---------------- */
    OLED_GPS_Page();
#elif (OLED_SCREEN_SEL == OLED_SCREEN_HC08)
    /* ---------------- HC-08 蓝牙页 ---------------- */
    OLED_HC08_Page();
#else
    /* 帧与统计快照放栈上: 本任务每轮跑完即睡, 无需跨轮保持。
       have_frame 用 static 保存"曾经收到过帧"这一状态, 避免
       偶尔一轮没取到新帧就让屏幕闪成 waiting。 */
    DIY_IMU_Display_t   frame;
    IMU_Recv_Snapshot_t snap;
    static uint8_t      have_frame = 0;
    /* 取链路统计与最新一帧。
       两者都是【只读快照】接口：
         IMU_Recv_GetSnapshot —— 统计量
         DIY_IMU_GetLatest    —— 最近一帧（不消费队列）
       ★ 刻意不用 DIY_IMU_TakeFrame()：那个队列的消费者是 imu_recv_task
         （负责喂 TinyML）。两个任务抢同一队列会让 TinyML 随机丢帧，
         且丢帧时机取决于刷屏节奏，极难排查。 */
    IMU_Recv_GetSnapshot(&snap);

    if (DIY_IMU_GetLatest(&frame)) {
      have_frame = 1U;
    }

    /* 显示页每 3 s 轮换：0=原始量+状态  1=姿态角  2=物理量(g) */
    if ((uint32_t)(xTaskGetTickCount() - s_page_tick) >= pdMS_TO_TICKS(3000)) {
      s_page_tick = xTaskGetTickCount();
      s_page = (uint8_t)((s_page + 1U) % 3U);
    }

    /* ---- 状态栏 ---- */
    OLED_ShowString(0, 0, "IMU", OLED_6X8);
    OLED_ShowNum(24, 0, snap.frame_rate, 3, OLED_6X8);
    OLED_ShowString(42, 0, "Hz", OLED_6X8);

    if (snap.link_lost) {
      OLED_ShowString(66, 0, "LOST  ", OLED_6X8);
    } else if (snap.inited == 0U) {
      OLED_ShowString(66, 0, "NOINIT", OLED_6X8);
    } else {
      OLED_ShowString(66, 0, "OK    ", OLED_6X8);
    }

    if (!have_frame) {
      OLED_ShowString(0, 24, "waiting node A", OLED_6X8);
    } else {
      const DIY_IMU_Display_t *f = &frame;

      /* ---- 加速度：原始计数 ---- */
      OLED_ShowString(0, 8, "A", OLED_6X8);
      OLED_ShowSignedNum( 12, 8, f->acc_raw[0], 5, OLED_6X8);
      OLED_ShowSignedNum( 48, 8, f->acc_raw[1], 5, OLED_6X8);
      OLED_ShowSignedNum( 84, 8, f->acc_raw[2], 5, OLED_6X8);
      OLED_ShowString(114, 8, "raw", OLED_6X8);

      /* ---- 加速度：mg ---- */
      OLED_ShowString(0, 16, " ", OLED_6X8);
      OLED_ShowSignedNum( 12, 16, f->acc_mg[0], 5, OLED_6X8);
      OLED_ShowSignedNum( 48, 16, f->acc_mg[1], 5, OLED_6X8);
      OLED_ShowSignedNum( 84, 16, f->acc_mg[2], 5, OLED_6X8);
      OLED_ShowString(114, 16, "mg", OLED_6X8);

      /* ---- 角速度 ---- */
      OLED_ShowString(0, 24, "G", OLED_6X8);
      OLED_ShowSignedNum( 12, 24, f->gyro_raw[0], 5, OLED_6X8);
      OLED_ShowSignedNum( 48, 24, f->gyro_raw[1], 5, OLED_6X8);
      OLED_ShowSignedNum( 84, 24, f->gyro_raw[2], 5, OLED_6X8);
      OLED_ShowString(114, 24, "raw", OLED_6X8);

      OLED_ShowString(0, 32, " ", OLED_6X8);
      OLED_ShowSignedNum( 12, 32, f->gyro_01dps[0], 5, OLED_6X8);
      OLED_ShowSignedNum( 48, 32, f->gyro_01dps[1], 5, OLED_6X8);
      OLED_ShowSignedNum( 84, 32, f->gyro_01dps[2], 5, OLED_6X8);
      OLED_ShowString(114, 32, "0.1d", OLED_6X8);

      /* ---- 合加速度 |a|（mg）。
             用整型牛顿迭代开方：F103 无 FPU，且本任务栈只有 192 word，
             浮点 sqrtf 会带来不可控的栈与代码膨胀。 ---- */
      {
        int32_t ax = f->acc_mg[0];
        int32_t ay = f->acc_mg[1];
        int32_t az = f->acc_mg[2];
        uint32_t sq = (uint32_t)(ax * ax) + (uint32_t)(ay * ay) + (uint32_t)(az * az);
        uint32_t r = 0U;
        uint8_t  it;

        if (sq > 0U) {
          uint32_t bit = 1U;
          while ((bit < sq) && (bit < 0x40000000UL)) {
            bit <<= 1;
          }
          r = (bit >> 1);
          if (r == 0U) {
            r = 1U;
          }
          for (it = 0; it < 12U; it++) {
            uint32_t last = r;
            r = (r + sq / r) / 2U;
            if (r == last) {
              break;
            }
          }
        }

        OLED_ShowString(0, 40, "|a|", OLED_6X8);
        OLED_ShowNum(24, 40, r, 5, OLED_6X8);
        OLED_ShowString(54, 40, "mg", OLED_6X8);
      }

      /* ---- 页指示 + 第 6 行内容 ---- */
      if (s_page == 0U) {
        /* 页0：flags 状态位，便于判断节点A的融合状态 */
        OLED_ShowString(78, 40, "F", OLED_6X8);
        OLED_ShowHexNum(90, 40, f->flags, 4, OLED_6X8);

        OLED_ShowString(0, 48, "CRC", OLED_6X8);
        OLED_ShowNum(24, 48, snap.crc_err, 4, OLED_6X8);
        OLED_ShowString(54, 48, "DROP", OLED_6X8);
        OLED_ShowNum(84, 48, snap.seq_drop, 4, OLED_6X8);
      } else if (s_page == 1U) {
        /* 页1：三个姿态角（0.01 度 -> 带两位小数的度）。
             六轴下 yaw 是【相对上电基准】的转角，不是绝对方位；
             roll/pitch 由重力观测，是绝对的。

             布局核算（6x8 字体每字符步进 11 px，屏幕宽 128）：
               "±RRR.RR" = 7 字符，从起点算占 72 px
               行6: R@1  -> 到 72    P@56 -> 到 127   都在屏内
               行5: Y@1  -> 到 72
             原来 P@60 会到 131，末字符被裁掉，这次一并修正。 */
        /* 三行各放一个角，都用 width=6。
             width 决定【整数部分的显示宽度】：
               width=6, frac=2 -> 整数位 3, 共 7 字符 = 72 px, 可表示 ±180.00
             ★ 这里踩过一次坑：误用 width=5 会让整数位只剩 2 位，
               >=100 度的角度被静默截断（179 显示成 79），而 yaw 经常在 100~180 度。
             三行 x=12 起 -> 最右到 83，既互不重叠也不溢出屏幕。
             代价：页1 不再显示 RS/BR/RO 诊断行（页0 与页2 仍显示）。 */
        OLED_ShowString(0, 40, "R", OLED_6X8);
        OLED_ShowScaled(12, 40, f->rpy_cdeg[0], 100, 2, 6);

        OLED_ShowString(0, 48, "P", OLED_6X8);
        OLED_ShowScaled(12, 48, f->rpy_cdeg[1], 100, 2, 6);

        /* ★ Yaw —— 用户发现它此前完全没显示 */
        OLED_ShowString(0, 56, "Y", OLED_6X8);
        OLED_ShowScaled(12, 56, f->rpy_cdeg[2], 100, 2, 6);
      } else {
        /* 页2：把 mg 换成 g（3 位小数），用于验证整数换算是否正确 */
        OLED_ShowString(78, 40, "unit:g", OLED_6X8);

        OLED_ShowString(0, 48, "X", OLED_6X8);
        OLED_ShowScaled(12, 48, f->acc_mg[0], 1000, 3, 6);
        OLED_ShowString(48, 48, "Z", OLED_6X8);
        OLED_ShowScaled(60, 48, f->acc_mg[2], 1000, 3, 6);
      }
    }

    /* ---- 诊断计数。页1 的第 56 行让给 Yaw，故仅在页0/页2 显示。 ---- */
    if (s_page != 1U) {
    OLED_ShowString(0, 56, "RS", OLED_6X8);
    OLED_ShowNum(18, 56, snap.resync, 3, OLED_6X8);
    OLED_ShowString(42, 56, "BR", OLED_6X8);
    OLED_ShowNum(60, 56, snap.byte_ring_drop, 3, OLED_6X8);
    OLED_ShowString(84, 56, "RO", OLED_6X8);
    OLED_ShowNum(102, 56, snap.ring_overrun, 3, OLED_6X8);
    }
#endif /* OLED_SCREEN_SEL == OLED_SCREEN_IMU */

    OLED_Update();
    /* USER CODE END OLED_Task_Entry_Loop */

    vTaskDelay(pdMS_TO_TICKS(OLED_TASK_PERIOD_MS));
  }
}

/* USER CODE END Application */
