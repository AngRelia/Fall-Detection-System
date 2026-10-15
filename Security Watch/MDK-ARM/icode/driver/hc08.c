/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    hc08.c
  * @brief   HC-08（BLE 4.0 串口透传模块）驱动 —— 接收用「空闲中断 + 中断接收」
  ******************************************************************************
  * 数据流：
  *   手机 --BLE--> HC-08 --TXD--> PA3(USART2_RX) --IT--> s_rx_chunk
  *        --(空闲中断回调)--> s_ring（环形缓冲） --HC08_Process()--> 按行切分 --> s_last_line
  *   MCU  --HC08_Send() / HC08_SendLine()--> PA2(USART2_TX) --> HC08 --BLE--> 手机
  *
  * 设计取舍：
  *   1) 用「空闲中断 + 中断接收」而不是 DMA：BLE 速率低（9600 → 最多 960 B/s），
  *      中断方式开销可忽略，还能复用工程里已有的 HAL_UARTEx_RxEventCallback 分发点，
  *      也不必为它占一个 DMA 通道（ws2812b 删除后 CH6 虽空闲，方案保持不变）。
  *   2) 环形缓冲 + 按行切分：手机侧协议是"一行一条命令/一条 JSON"，切行最省事。
  *   3) 发送是阻塞式（HAL_UART_Transmit，超时 300 ms）：BLE 报文短（< 200 B），
  *      9600 下发 200 B 约 210 ms，属低频事件，阻塞可接受且不需要额外缓冲。
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "hc08.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef *s_huart = NULL;

/* 中断接收落地缓冲：ReceiveToIdle_IT 每次都从下标 0 开始填，
   空闲中断回调里把 [0, Size) 搬进环形缓冲后立刻重新武装 */
static uint8_t  s_rx_chunk[HC08_RX_CHUNK_SIZE];

/* 环形缓冲（中断写、任务读；head/tail 用 volatile） */
static uint8_t           s_ring[HC08_RX_RING_SIZE];
static volatile uint16_t s_ring_head = 0U;
static volatile uint16_t s_ring_tail = 0U;

/* 统计量 */
static volatile uint32_t s_rx_bytes = 0U;
static volatile uint32_t s_tx_bytes = 0U;
static volatile uint32_t s_overrun  = 0U;
static volatile uint32_t s_last_rx_tick = 0U;

/* 行装配（只在任务上下文使用） */
static char     s_line[HC08_LINE_MAX];
static uint16_t s_line_len = 0U;
/* 最后一个字节到达的时刻：用于"静默判帧"（见 HC08_Process） */
static uint32_t s_line_last_tick = 0U;

/* 最近一条完整行 + "有新行待消费"标志（消费方是 ble_task） */
static char     s_last_line[HC08_LINE_MAX];
static uint8_t  s_line_ready = 0U;

static uint32_t s_lines   = 0U;
static uint32_t s_at_tx   = 0U;
static uint32_t s_at_ok   = 0U;
static uint32_t s_at_err  = 0U;

/* Private function prototypes -----------------------------------------------*/
static void HC08_FeedByte(uint8_t c);
static void HC08_FinishLine(void);
static uint8_t HC08_LineHas(const char *line, const char *needle);

/* ---------------------------------------------------------------------------
 * 初始化 / 接收武装
 * ------------------------------------------------------------------------- */
void HC08_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return;
    }

    s_huart      = huart;
    s_ring_head  = 0U;
    s_ring_tail  = 0U;
    s_line_len   = 0U;
    s_line_ready = 0U;
    s_last_line[0] = '\0';
    s_last_rx_tick = HAL_GetTick();

    /* 启动「收到数据或空闲即回调」的中断接收；之后由回调自己续接 */
    (void)HAL_UARTEx_ReceiveToIdle_IT(s_huart, s_rx_chunk, (uint16_t)HC08_RX_CHUNK_SIZE);
}

/* 由 HAL 在「空闲事件」或「缓冲收满」时调用（中断上下文） */
void HC08_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint16_t i;

    if ((s_huart == NULL) || (huart != s_huart))
    {
        return;
    }

    for (i = 0U; i < Size; i++)
    {
        uint16_t next = (uint16_t)((s_ring_head + 1U) % HC08_RX_RING_SIZE);

        if (next == s_ring_tail)
        {
            s_overrun++;                 /* 环形缓冲满：丢最新字节，保证不覆盖未读数据 */
            continue;
        }
        s_ring[s_ring_head] = s_rx_chunk[i];
        s_ring_head = next;
    }

    s_rx_bytes    += Size;
    s_last_rx_tick = HAL_GetTick();

    /* 必须重新武装，否则只能收到这一次（HAL 在回调前已把 RxState 复位为 READY） */
    (void)HAL_UARTEx_ReceiveToIdle_IT(s_huart, s_rx_chunk, (uint16_t)HC08_RX_CHUNK_SIZE);
}

/* ---------------------------------------------------------------------------
 * 任务侧：排空环形缓冲 → 组帧
 *
 * 组帧有两条判据（缺一不可）：
 *   1) 显式行尾：收到 '\n' 或 '\r' 就收尾 —— 手机 App 勾了"发送新行"时走这条；
 *   2) 静默判帧：行里已有内容、且距最后一个字节 ≥ HC08_LINE_IDLE_MS（默认 100 ms）
 *      就认为这一条命令发完了。
 *
 * 为什么必须有第 2 条（2026-10-14 实测踩到）：
 *   汇承 App 默认**不**给发送内容加换行。用户点 5 次"发送 PING"，App 统计"发送 20 字节"
 *   = 5×4，也就是每次都只有 `PING` 四个字符、没有 CRLF。若只认 '\n'，
 *   驱动会一直攒着这 4 个字符等行尾 → 命令永远不完整 → 手机收不到任何应答。
 *   100 ms 的静默窗口远大于 BLE 分包间隔（一个连接间隔内可到 50 ms），
 *   所以既兜住了"裸命令"，又不会把一条被拆成几个 BLE 包的长报文截断。
 * ------------------------------------------------------------------------- */
void HC08_Process(void)
{
    while (s_ring_tail != s_ring_head)
    {
        uint8_t c = s_ring[s_ring_tail];

        s_ring_tail = (uint16_t)((s_ring_tail + 1U) % HC08_RX_RING_SIZE);
        HC08_FeedByte(c);
    }

    /* 静默判帧：缓冲区已排空、行里有内容、且超过静默窗口 */
    if ((s_line_len > 0U) &&
        ((uint32_t)(HAL_GetTick() - s_line_last_tick) >= HC08_LINE_IDLE_MS))
    {
        HC08_FinishLine();
    }
}

/* 把当前装配中的行收尾成"一条完整行"（供 '\n' / '\r' 与静默判帧共用） */
static void HC08_FinishLine(void)
{
    if (s_line_len == 0U)
    {
        return;
    }

    s_line[s_line_len] = '\0';

    {
        uint16_t n = s_line_len;

        if (n > (HC08_LINE_MAX - 1U))
        {
            n = (HC08_LINE_MAX - 1U);
        }
        (void)memcpy(s_last_line, s_line, n);
        s_last_line[n] = '\0';
    }

    s_lines++;
    s_line_ready = 1U;

    /* AT 应答分类：模块正常回 "OK" 或 "OK+Set:..."，失败回 "ERROR"/"FAIL" */
    if (HC08_LineHas(s_last_line, "OK") != 0U)
    {
        s_at_ok++;
    }
    else if ((HC08_LineHas(s_last_line, "ERROR") != 0U) ||
             (HC08_LineHas(s_last_line, "FAIL") != 0U))
    {
        s_at_err++;
    }

    s_line_len = 0U;
}

static void HC08_FeedByte(uint8_t c)
{
    s_line_last_tick = HAL_GetTick();

    /* 行尾：'\n' 或 '\r' 都当作一条命令结束（CRLF 时第二次调用是空行，直接返回） */
    if ((c == (uint8_t)'\n') || (c == (uint8_t)'\r'))
    {
        HC08_FinishLine();
        return;
    }

    if (s_line_len < (HC08_LINE_MAX - 1U))
    {
        s_line[s_line_len] = (char)c;
        s_line_len++;
    }
    else
    {
        /* 行太长：丢弃整行，避免半截命令被误解析 */
        s_line_len = 0U;
    }
}

/* 子串查找（不引 strstr：AC5 下也能省一点代码，且行为完全可控） */
static uint8_t HC08_LineHas(const char *line, const char *needle)
{
    uint16_t i;
    uint16_t nl = (uint16_t)strlen(needle);

    if ((line == NULL) || (needle == NULL) || (nl == 0U))
    {
        return 0U;
    }

    for (i = 0U; line[i] != '\0'; i++)
    {
        uint16_t j = 0U;

        while ((j < nl) && (line[i + j] != '\0') && (line[i + j] == needle[j]))
        {
            j++;
        }
        if (j == nl)
        {
            return 1U;
        }
    }
    return 0U;
}

/* ---------------------------------------------------------------------------
 * 发送
 * ------------------------------------------------------------------------- */
uint8_t HC08_Send(const uint8_t *data, uint16_t len)
{
    if ((s_huart == NULL) || (data == NULL) || (len == 0U))
    {
        return 0U;
    }

    if (HAL_UART_Transmit(s_huart, (uint8_t *)data, len, 300U) != HAL_OK)
    {
        return 0U;
    }
    s_tx_bytes += len;
    return 1U;
}

uint8_t HC08_SendLine(const char *s)
{
    static const uint8_t crlf[2] = { (uint8_t)'\r', (uint8_t)'\n' };

    if (s == NULL)
    {
        return 0U;
    }
    if (HC08_Send((const uint8_t *)s, (uint16_t)strlen(s)) == 0U)
    {
        return 0U;
    }
    return HC08_Send(crlf, 2U);
}

uint8_t HC08_SendAT(const char *cmd)
{
    if (cmd == NULL)
    {
        return 0U;
    }
    s_at_tx++;
    return HC08_SendLine(cmd);
}

/* ---------------------------------------------------------------------------
 * 状态与统计
 * ------------------------------------------------------------------------- */
uint8_t HC08_IsConnected(void)
{
#if (HC08_USE_STATE_PIN == 1)
    GPIO_PinState st = HAL_GPIO_ReadPin(HC08_STATE_GPIO_Port, HC08_STATE_Pin);

#if (HC08_STATE_ACTIVE_HIGH == 1)
    return (uint8_t)((st == GPIO_PIN_SET) ? 1U : 0U);
#else
    return (uint8_t)((st == GPIO_PIN_RESET) ? 1U : 0U);
#endif
#else
    return 0U;
#endif
}

uint32_t HC08_GetRxBytes(void)      { return s_rx_bytes; }
uint32_t HC08_GetTxBytes(void)      { return s_tx_bytes; }
uint32_t HC08_GetLineCount(void)    { return s_lines; }
uint32_t HC08_GetAtTxCount(void)    { return s_at_tx; }
uint32_t HC08_GetAtOkCount(void)    { return s_at_ok; }
uint32_t HC08_GetAtErrCount(void)   { return s_at_err; }
uint32_t HC08_GetOverrunCount(void) { return s_overrun; }

uint32_t HC08_GetLastRxAge(uint32_t now_ms)
{
    return (uint32_t)(now_ms - s_last_rx_tick);
}

uint16_t HC08_GetLastLine(char *buf, uint16_t buflen)
{
    uint16_t n = 0U;

    if ((buf == NULL) || (buflen == 0U))
    {
        return 0U;
    }

    while ((s_last_line[n] != '\0') && (n < (uint16_t)(buflen - 1U)))
    {
        buf[n] = s_last_line[n];
        n++;
    }
    buf[n] = '\0';
    return n;
}

uint8_t HC08_TakeLine(char *buf, uint16_t buflen)
{
    if (s_line_ready == 0U)
    {
        return 0U;
    }
    s_line_ready = 0U;
    (void)HC08_GetLastLine(buf, buflen);
    return 1U;
}
