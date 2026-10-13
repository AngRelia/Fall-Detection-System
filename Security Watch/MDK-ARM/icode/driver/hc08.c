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

/* 最近一条完整行 + "有新行待消费"标志（消费方是 ble_task） */
static char     s_last_line[HC08_LINE_MAX];
static uint8_t  s_line_ready = 0U;

static uint32_t s_lines   = 0U;
static uint32_t s_at_tx   = 0U;
static uint32_t s_at_ok   = 0U;
static uint32_t s_at_err  = 0U;

/* Private function prototypes -----------------------------------------------*/
static void HC08_FeedByte(uint8_t c);
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
 * 任务侧：排空环形缓冲 → 按行切分
 * ------------------------------------------------------------------------- */
void HC08_Process(void)
{
    while (s_ring_tail != s_ring_head)
    {
        uint8_t c = s_ring[s_ring_tail];

        s_ring_tail = (uint16_t)((s_ring_tail + 1U) % HC08_RX_RING_SIZE);
        HC08_FeedByte(c);
    }
}

static void HC08_FeedByte(uint8_t c)
{
    if (c == (uint8_t)'\n')
    {
        /* 去掉行尾 '\r'，收尾成一条完整行 */
        if ((s_line_len > 0U) && (s_line[s_line_len - 1U] == '\r'))
        {
            s_line_len--;
        }
        s_line[s_line_len] = '\0';

        if (s_line_len > 0U)
        {
            uint16_t n = s_line_len;

            if (n > (HC08_LINE_MAX - 1U))
            {
                n = (HC08_LINE_MAX - 1U);
            }
            (void)memcpy(s_last_line, s_line, n);
            s_last_line[n] = '\0';

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
        }
        s_line_len = 0U;
        return;
    }

    if (c == (uint8_t)'\r')
    {
        /* 行内容里不保留 '\r'（有些模块用 CR 单独作为行结束） */
        if (s_line_len < (HC08_LINE_MAX - 1U))
        {
            s_line[s_line_len] = '\0';
            if (s_line_len > 0U)
            {
                uint16_t n = s_line_len;

                (void)memcpy(s_last_line, s_line, n);
                s_last_line[n] = '\0';
                s_lines++;
                s_line_ready = 1U;
                if (HC08_LineHas(s_last_line, "OK") != 0U)
                {
                    s_at_ok++;
                }
            }
            s_line_len = 0U;
        }
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
    return (uint8_t)((HAL_GPIO_ReadPin(HC08_STATE_GPIO_Port, HC08_STATE_Pin) == GPIO_PIN_SET) ? 1U : 0U);
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
