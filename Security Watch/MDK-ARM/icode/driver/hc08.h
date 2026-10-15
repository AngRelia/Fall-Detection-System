#ifndef __HC08_H__
#define __HC08_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * HC-08（蓝牙 4.0 BLE 串口透传模块）驱动
 *
 * 硬件/协议要点（详见 docs/HC-08接入方案.md）：
 *   · 供电 **3.3 V**（2.0~3.6 V），**不能接 5 V** —— 与 GPS 底板（要 5 V）相反，务必注意
 *   · 默认串口 9600 8N1，与本工程 USART2（PA2=TX / PA3=RX）一致
 *   · 透传模式：模块把 BLE 收到的字节原样从 TXD 吐出，把 RXD 收到的字节原样发给手机
 *   · AT 指令（`AT` / `AT+NAME...` / `AT+BAUDn` …）在**未连接**状态下从串口直接发，
 *     模块回 `OK` / `OK+Set:xxx`；连接后一般进入透传、不再解析 AT
 *   · STATE 引脚：模块已连接时输出高电平（可选接线，接 PB15）
 *
 * 接收方式：**「空闲中断 + 中断接收」**（HAL_UARTEx_ReceiveToIdle_IT）。
 *   为什么用中断而不是 DMA：BLE 数据率极低（9600 下最多 960 B/s），中断方式完全够用、
 *   实现简单，且不必为它占一个 DMA 通道（当初 DMA1_CH6 被 WS2812 占用，2026-10-14
 *   ws2812b 已删除、CH6 空闲，但中断方案保持不变）。
 * ------------------------------------------------------------------------- */

/* 缓冲尺寸（本板静态 RAM 只剩几百字节，尺寸都是算着给的）
   · RX_CHUNK：中断接收的一次落地缓冲；空闲中断到达时把其中数据搬进环形缓冲
   · RX_RING ：环形缓冲，吸收"任务还没排空时又来一帧"的突发
   · LINE_MAX：单条命令行（手机下发的命令与模块的 AT 应答都很短，64 足够；
               每减 32 字节能省 3×32 = 96 字节静态 RAM —— 驱动里两份拷贝
               以及屏幕侧那份拷贝都跟着变小） */
#define HC08_RX_CHUNK_SIZE      64U
#define HC08_RX_RING_SIZE       128U
#define HC08_LINE_MAX           64U

/* STATE 引脚是否接线（1 = 接了，见 gpio.c / main.h 的 HC08_STATE_Pin）。
   没接也不影响收发，只是 HC08_IsConnected() 恒为 0（屏幕显示 NOT CONN）。 */
#define HC08_USE_STATE_PIN      1

/* STATE 的有效电平：1 = 模块"已连接"时该脚输出高（常见），0 = 低有效（个别批次）。
   若手机已连接、STATE 脚也确实接了，屏幕却仍显示 NOT CONN，就把这里改成 0。 */
#define HC08_STATE_ACTIVE_HIGH  1

/* 行结束的两种判据（见 hc08.c）：
   1) 收到 '\n' / '\r' —— 手机 App 勾了"发送新行"时走这条；
   2) 静默超时 —— 手机 App **默认不加换行**（2026-10-14 实测：发 5 次 PING 共 20 字节，
      即每次 4 字节、无 CRLF），没有这条就永远收不到命令。
      100 ms 远大于 BLE 分包间隔（≤50 ms），既兜住裸命令，又不会把拆包的长报文切断。 */
#define HC08_LINE_IDLE_MS       100U

/* ---- 生命周期 ---- */
/* 绑定串口并启动「空闲中断 + 中断接收」。必须在调度器启动后、任务里调用。 */
void HC08_Init(UART_HandleTypeDef *huart);

/* 由 main.c 的 HAL_UARTEx_RxEventCallback() 转发（USART2 分支） */
void HC08_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

/* 排空环形缓冲、拼行（在任务上下文周期调用） */
void HC08_Process(void);

/* ---- 发送 ---- */
uint8_t HC08_Send(const uint8_t *data, uint16_t len);   /* 1 = 发送成功 */
uint8_t HC08_SendLine(const char *s);                   /* 追加 CRLF 后发送 */
uint8_t HC08_SendAT(const char *cmd);                   /* 发送 "AT..." + CRLF，并计数 */

/* ---- 状态与统计（只读，供屏幕/上报用） ---- */
uint8_t  HC08_IsConnected(void);        /* STATE 引脚：1 = 手机已连接 */
uint32_t HC08_GetRxBytes(void);         /* 累计收到字节 */
uint32_t HC08_GetTxBytes(void);         /* 累计发出字节 */
uint32_t HC08_GetLineCount(void);       /* 累计收到的完整行数 */
uint32_t HC08_GetAtTxCount(void);       /* 累计发出的 AT 指令数 */
uint32_t HC08_GetAtOkCount(void);       /* 模块回 "OK" 的次数（AT 链路畅通的判据） */
uint32_t HC08_GetAtErrCount(void);      /* 模块回 "ERROR"/"FAIL" 的次数 */
uint32_t HC08_GetOverrunCount(void);    /* 环形缓冲溢出丢字节次数 */
uint32_t HC08_GetLastRxAge(uint32_t now_ms);  /* 距最后一次收到字节的毫秒数 */

/* 去看最近一条收到的行（不清标志，屏幕用） */
uint16_t HC08_GetLastLine(char *buf, uint16_t buflen);

/* 取走一条"尚未被消费"的行（消费方=ble_task；屏幕请用 HC08_GetLastLine） */
uint8_t  HC08_TakeLine(char *buf, uint16_t buflen);

#ifdef __cplusplus
}
#endif

#endif /* __HC08_H__ */
