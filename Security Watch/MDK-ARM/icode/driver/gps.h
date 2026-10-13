#ifndef __GPS_H__
#define __GPS_H__

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===========================================================================
 * gps.h —— DX-GP22 / ATGM336H GNSS 模块驱动（NMEA0183，UART 串口）
 *
 * 对应课题要求（4）：跌倒事件的 BLE/4G 上报纸文需要携带设备自身的位置。
 * 模块资料：《DX-GP22-GNSS模块技术手册》《DX-GP22-GNSS模块_串口UART_应用指导》
 *          《DX-GP22-NMEA0183协议规范》
 *
 * ---------------------------------------------------------------------------
 * 模块事实（来自厂家资料，非推测）
 *   · 底板 DX-GP22-A：VDD = 3.6~6 V（典型 5 V，板上有 LDO）
 *     裸模块 DX-GP22：VCC = 2.7~3.6 V（典型 3.3 V），负载能力要 ≥100 mA
 *   · 串口：默认 9600 bps / 8 数据位 / 无校验 / 1 停止位
 *     支持 4800/9600/19200/38400/57600/115200，用 $PCAS01,n 修改
 *   · 默认定位更新率 1 Hz（$PCAS02 可设 2/5/10 Hz）
 *   · 默认同时输出 GGA/GLL/GSA/GSV/RMC/VTG/ZDA 等语句
 *   · 功耗：32 mA @3.3 V（type）；VBAT 备电 10 uA（保 RTC/SRAM，热启动）
 *   · 定位成功后 1PPS 输出 1 Hz 秒脉冲；ON/OFF 引脚高电平=工作、低电平=关断
 *
 * ---------------------------------------------------------------------------
 * 本驱动的设计取舍
 *   · 走「循环 DMA + 空闲中断」接收，中断里**只记录位置**，解析全部放任务里，
 *     避免在 ISR 里做 80 字节的字符串解析（与节点 A 的 Serial.c 同思路，
 *     但节点 A 是在 ISR 里解析，本项目改为任务解析）。
 *   · 全程整数定点，不用 float / 不用 printf：本工程已刻意裁掉 vsprintf 以
 *     省 Flash（见 docs/TinyML 使用说明.md），这里绝不能再把格式化库拉回来。
 *   · 定位结果按「最新快照 + 序号（seqlock）」发布，写方只有 GPS 任务，
 *     读方（ble_task / oled_task）无需加锁即可拿到自洽的一份拷贝。
 *
 * ---------------------------------------------------------------------------
 * 标度约定（与帧协议里 mg / 0.01° / 0.1°/s 的定点风格一致）
 *   lat_e7 / lon_e7 : 度 × 1e7（北纬/东经为正，南纬/西经为负）
 *                     int32 足够放下 ±180×1e7 = ±1.8e9
 *   alt_cm          : 海拔 厘米
 *   speed_cms       : 对地速度 厘米/秒（模块原始单位是节）
 *   course_cdeg     : 对地航向 度 × 100
 *   hdop_x100       : HDOP × 100
 *   utc_ms          : UTC 当日毫秒
 *   date_yyyymmdd   : 日期，如 20240709
 *   stamp_ms        : HAL_GetTick()，写入该次定位的本机时刻（判"定位是否新鲜"）
 * ========================================================================= */

/* NMEA 语句缓冲区：最长语句（含 GSV）约 82 字节，留足余量 */
#define GPS_SENTENCE_MAX   96U
/* 循环 DMA 接收缓冲：中断按位置差把新字节搬进环形缓冲，128 B 足够
   （一次空闲事件最多覆盖 1~2 条语句；默认语句集约 620 B/s，20 ms 轮询一次） */
#define GPS_RX_DMA_SIZE    128U
/* 中断 → 任务的字节环形缓冲。默认语句集约 620 B/s（突发 GSV 约 900 B），
   任务 20 ms 消费一次，256 B 有约 0.25 s 的抗阻塞余量；
   精简输出语句（只留 GGA+RMC）后流量降到约 150 B/s，余量更宽。 */
#define GPS_RING_SIZE      256U
/* 发送 $PCA 命令的缓存长度 */
#define GPS_CMD_MAX        48U

/* 最新定位快照 */
typedef struct
{
    int32_t  lat_e7;         /* 纬度 ×1e7 度，N 为正 */
    int32_t  lon_e7;         /* 经度 ×1e7 度，E 为正 */
    uint8_t  valid;          /* 1 = 定位有效（RMC status=A 且 GGA quality>0） */
    uint8_t  status;         /* RMC 定位有效标志：1=A 0=V */
    uint8_t  quality;        /* GGA 定位质量：0 无效 / 1 单点 / 2 差分 */
    uint8_t  sats;           /* 参与定位的卫星数 */
    uint16_t hdop_x100;      /* HDOP ×100 */
    int32_t  alt_cm;         /* 海拔 cm */
    uint16_t speed_cms;      /* 对地速度 cm/s */
    uint16_t course_cdeg;    /* 对地航向 ×0.01° */
    uint32_t utc_ms;         /* UTC 当日毫秒 */
    uint32_t date_yyyymmdd;  /* 日期 yyyymmdd */
    uint32_t stamp_ms;       /* 本机 tick（写入时刻） */
    uint8_t  has_rmc;        /* 收到过 RMC */
    uint8_t  has_gga;        /* 收到过 GGA */
    /* ---- 以下两项来自 GSV，用于判断"天线/视野"是否正常（不是定位解算量）----
       为什么需要：模块在无源（陶瓷）天线下会一直报 $GPTXT ... ANTENNA OPEN，
       那条语句说明不了天线好坏；真正能说明问题的是"可见卫星数"和"最强信噪比"。
       · sats_view = 0 且 snr_max = 0 → 天线/射频真没收到信号（或完全在室内）
       · sats_view 有值、snr_max 20~45 → 天线在工作，只是还没凑够定位条件
       GSV 默认约 3 s 输出一轮，所以这两项刷新较慢，属正常。 */
    uint8_t  sats_view;      /* 可见卫星总数（GSV 第一句的"卫星数"字段） */
    uint8_t  snr_max;        /* 本轮 GSV 里最强的信噪比（dBHz，0~99） */
} GPS_Fix_t;

/* ---------------------------------------------------------------------------
 * 生命周期
 * ------------------------------------------------------------------------- */

/* 绑定串口并启动「循环 DMA + 空闲中断」接收。
 * huart 由 CubeMX 生成（例如 &huart1 / &huart2 / &huart3）。
 * 调用前请确认 CubeMX 里该串口已配好 RX DMA（Circular）与全局中断。 */
void GPS_Init(UART_HandleTypeDef *huart);

/* 重新武装接收（掉线恢复用）：先 Abort 再 Init */
void GPS_Reinit(void);

/* 在 HAL_UARTEx_RxEventCallback() 里转发进来的接收事件。
 * CubeMX 生成的 stm32f1xx_it.c → USARTx_IRQHandler → HAL_UART_IRQHandler，
 * 最终回到用户回调 HAL_UARTEx_RxEventCallback，在那里调用本函数即可：
 *
 *   void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
 *   {
 *     GPS_RxEventCallback(huart, Size);
 *   }
 *
 * 注意：F1 HAL 的 Size 是「DMA 缓冲里的绝对位置」，不是本次新增字节数，
 * 所以本驱动用位置差来计算新数据，天然支持环形回卷。 */
void GPS_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

/* ---------------------------------------------------------------------------
 * 数据处理（任务上下文调用）
 * ------------------------------------------------------------------------- */

/* 消费 DMA 缓冲中尚未处理的字节 → 装配 NMEA 语句 → 更新快照。
 * 建议在 GPS 任务里 20~50 ms 调一次。 */
void GPS_Process(void);

/* 直接喂字节（自测 / 软串口 / 离线回放用，不经过 DMA） */
void GPS_FeedBytes(const uint8_t *data, uint16_t len);

/* ---------------------------------------------------------------------------
 * 读取与应用
 * ------------------------------------------------------------------------- */

/* 取最新快照。返回 1 = 拷贝自洽；0 = 正在被写入（极小概率），此时 out 不可用 */
uint8_t GPS_GetFix(GPS_Fix_t *out);

/* 定位是否新鲜：valid 且距今不超过 max_age_ms（室内长时间无星时用来降级） */
uint8_t GPS_IsFixFresh(uint32_t max_age_ms, uint32_t now_ms);

/* 拼一段 JSON 片段（不带前后逗号），供 ble_task 直接插进告警报文：
 *   {"v":1,"lat":399876543,"lon":1163123456,"q":1,"sat":9,
 *    "hdop":95,"alt":4300,"spd":634,"cog":0,"fix_age":1200}
 * 返回写入的字符数（不含结尾 '\0'）。无定位时 v=0 且经纬度为 0。 */
uint16_t GPS_BuildJson(char *buf, uint16_t buflen, uint32_t now_ms);

/* 拼人类可读位置串，给 OLED 用："39.9876542N 116.3123456E" / "NO GPS FIX" */
uint16_t GPS_FormatPosition(char *buf, uint16_t buflen, uint32_t now_ms);

/* 最近一条【通过校验】的完整语句原文（含前导 '$'，如 "$GNRMC,073028.600,A,...*00"），
   给"GPS 接收页"直接显示用 —— 联调时看它最快：
     · 空串      → 一个字节都没解析出完整语句（没接线 / 波特率不对 / 模块没供电）
     · 有 NMEA   → 模块在说话，接下来只需等 status=A（定位有效）
     · 全是乱码  → 校验错会很多（CSE 计数上涨），多半是波特率或电平问题
   没收到过语句时写入空串。缓冲建议 >= 96 字节。 */
void GPS_GetLastSentence(char *buf, uint16_t buflen);

/* 最近一条**定位语句**（RMC/GGA）原文（含前导 '$'）。
 * 与 GPS_GetLastSentence() 的区别：后者是"最近一条任意语句"，而 GPTXT/GSV/GSA
 * 会频繁插进来（实测 GPTXT 占了 1478 条里的 632 条），排查字段问题时必须看这一份：
 * 经纬度/UTC 时间/卫星数都只来自 RMC 或 GGA。缓冲建议 >= 96 字节。 */
void GPS_GetLastPosSentence(char *buf, uint16_t buflen);

/* ---------------------------------------------------------------------------
 * 配置指令（$PCA 命令，见《串口UART_应用指导》第 5 章）
 * ------------------------------------------------------------------------- */

/* 发一条 $PCA 命令：body 不含 '$' 与 '*CS'，例如 "PCAS01,1"。
 * 校验和（$ 与 * 之间所有字符的异或）内部自动计算并补上 CRLF。 */
void GPS_SendCmd(const char *body);

/* 精简输出语句：只保留 **GGA + GSV + RMC**（既够定位解算，又保留可见卫星数/信噪比）。
 * ⚠ PCAS03 的参数个数在厂家不同文档版本里有 14/15 两种写法，
 *   上板前先用 PC 端 sscom 实测确认（见 docs/GPS模块接入方案.md 第 8 节）。 */
void GPS_ConfigMinimal(void);

/* 恢复"全部语句打开"（≈ 出厂默认输出集合）。
 * 用途：模块被之前的 PC 端试验用 $PCAS03 关掉了 GGA/RMC/GSV 时，
 *       屏幕上只剩零星 $GPTXT，经纬度永远出不来 —— 发这一条救回。 */
void GPS_ConfigAll(void);

/* ---------------------------------------------------------------------------
 * 电源门控（可选）
 * 底板 ON/OFF 引脚高电平=工作、低电平=关断。若硬件把该脚接到 MCU，
 * 在 gps.c 顶部定义 GPS_PWR_GPIO_Port / GPS_PWR_GPIO_Pin 即可生效。
 * ------------------------------------------------------------------------- */
void GPS_Power(uint8_t on);

/* ---------------------------------------------------------------------------
 * 诊断量（OLED 显示 / 论文"链路质量"实验都直接用得上）
 * ------------------------------------------------------------------------- */
uint32_t GPS_GetRxBytes(void);        /* 累计收到字节数 */
uint32_t GPS_GetSentenceCount(void);  /* 校验通过的语句数 */
uint32_t GPS_GetCsumErrCount(void);   /* 校验和错误语句数 */
uint32_t GPS_GetOverrunCount(void);   /* 解析跟不上的丢字节次数 */

/* ---- 语句类型普查（联调利器）----
   把"校验通过的语句"按类型计数。用途：**一眼看出模块缺哪种语句**。
   最典型的坑：模块被 PC 端工具改过输出语句集（$PCAS03），只吐 GSV/GSA，
   而经纬度在 GGA/RMC 里 —— 此时 GGA/RMC 计数恒为 0，定位永远出不来。 */
#define GPS_TYPE_RMC    0U
#define GPS_TYPE_GGA    1U
#define GPS_TYPE_GSV    2U
#define GPS_TYPE_GSA    3U
#define GPS_TYPE_OTH    4U
uint32_t GPS_GetTypeCount(uint8_t type);

uint32_t GPS_GetLastRxAge(uint32_t now_ms);  /* 距最后一次收到字节的毫秒数 */
uint8_t  GPS_IsAlive(uint32_t now_ms);       /* 5 s 内收到过字节 */

#ifdef __cplusplus
}
#endif

#endif /* __GPS_H__ */
