/**
  ******************************************************************************
  * @file    diy_imu.c
  * @brief   DIY IMU（节点A）串口数据链路驱动实现
  ******************************************************************************
  * 数据流（与 gps.c 同一路数，便于维护）：
  *
  *   ① USART3 DMA(Circular) 持续把字节填进 s_dma_buf
  *   ② 空闲中断触发 DIY_IMU_RxEventCallback(Size)
  *        把 [上次位置, Size) 的增量字节搬进 s_byte_ring
  *   ③ 任务里调 DIY_IMU_Process()
  *        从 s_byte_ring 逐字节取，用状态机找帧头、凑满 32 字节、校验 CRC
  *        通过则换算单位后入队 s_frame_ring
  *   ④ 任务里调 DIY_IMU_TakeFrame() 取帧消费
  *
  * 并发说明：
  *   - s_dma_buf / s_byte_ring 的写侧在中断上下文，读侧在任务上下文。
  *     单生产者单消费者 + 每次只推进一个索引，无需关中断。
  *   - s_frame_ring 同样单生产（DIY_IMU_Process）单消费（TakeFrame）。
  *   - 统计量 s_stats 会被中断与任务同时改，读写用 taskENTER_CRITICAL 保护。
  ******************************************************************************
  */
#include "diy_imu.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* Private define ------------------------------------------------------------*/
/* DMA 接收缓冲。512 B 在 115200 下约 44 ms 的流量，
   空闲中断通常每帧（约 24 ms @42Hz）就会来一次，余量充足。 */
#define DIY_DMA_BUF_SIZE        128u

/* 字节环形缓冲。留 1 字节区分空/满，可用容量 = SIZE-1。
   100 Hz × 32 B = 3.2 KB/s，1024 B 约 320 ms 余量。 */
#define DIY_BYTE_RING_SIZE      256u

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef *s_huart = NULL;

/* DMA 直写缓冲（循环模式，只启动一次） */
static uint8_t  s_dma_buf[DIY_DMA_BUF_SIZE];
static uint16_t s_dma_old_pos = 0;      /* 上次已搬走的位置（绝对下标） */

/* 字节环形缓冲：中断写 head，任务读 tail */
static uint8_t           s_byte_ring[DIY_BYTE_RING_SIZE];
static volatile uint16_t s_byte_head = 0;
static volatile uint16_t s_byte_tail = 0;

/* 帧环形缓冲：任务解析后写，任务消费 */
static DIY_IMU_Frame_t   s_frame_ring[DIY_IMU_FRAME_RING];
static volatile uint16_t s_frame_head = 0;
static volatile uint16_t s_frame_tail = 0;

/* 最新一帧快照，仅供显示读取（不参与队列消费）。
   单生产（DIY_FeedByte，任务上下文）单消费（显示任务），
   结构体整体赋值在 Cortex-M3 上不是原子的，所以用临界区保护。 */
static DIY_IMU_Display_t s_latest;
static volatile uint8_t s_latest_valid = 0;


/* 帧解析状态机 */
static uint8_t  s_frame_buf[DIY_IMU_FRAME_LEN];
static uint16_t s_frame_idx = 0;
static uint8_t  s_hdr_seen = 0;         /* 已收到 hdr0，等 hdr1 */

/* 序号连续性检查 */
static uint8_t  s_last_seq = 0;
static uint8_t  s_last_seq_valid = 0;

/* 统计 */
static DIY_IMU_Stats_t s_stats;

/* CRC-16/MODBUS 查表（512 B，放 flash）。
   与节点A的 proto.c 用同一张表 —— 这是两端能互通的前提。
   已用 4 个权威测试向量 + 2000 组随机数据与逐位实现交叉验证。 */
static const uint16_t s_crc_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

uint16_t DIY_IMU_CRC16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFFu;

    if (buf == NULL) {
        return crc;
    }
    while (len--) {
        crc = (uint16_t)((crc >> 8) ^ s_crc_table[(crc ^ *buf++) & 0xFFu]);
    }
    return crc;
}

/* Private function prototypes -----------------------------------------------*/
static void     DIY_BytePush(uint8_t b);
static uint8_t  DIY_BytePop(uint8_t *b);
static void     DIY_FramePush(const DIY_IMU_Frame_t *f);
static void     DIY_FeedByte(uint8_t b);

/* Private functions ---------------------------------------------------------*/

/* 字节入环（中断上下文调用）。缓冲满则丢弃本字节并计数 —— 绝不阻塞。 */
static void DIY_BytePush(uint8_t b)
{
    uint16_t next = (uint16_t)((s_byte_head + 1u) % DIY_BYTE_RING_SIZE);

    if (next == s_byte_tail) {
        s_stats.byte_ring_drop++;
        return;
    }
    s_byte_ring[s_byte_head] = b;
    s_byte_head = next;
}

/* 字节出环（任务上下文调用）。返回 1 = 取到。 */
static uint8_t DIY_BytePop(uint8_t *b)
{
    if (s_byte_tail == s_byte_head) {
        return 0;
    }
    *b = s_byte_ring[s_byte_tail];
    s_byte_tail = (uint16_t)((s_byte_tail + 1u) % DIY_BYTE_RING_SIZE);
    return 1;
}

/* 帧入环。整车缓冲满时覆盖最旧帧（丢旧留新：新数据对跌倒检测更有价值） */
static void DIY_FramePush(const DIY_IMU_Frame_t *f)
{
    uint16_t next = (uint16_t)((s_frame_head + 1u) % DIY_IMU_FRAME_RING);

    if (next == s_frame_tail) {
        s_frame_tail = (uint16_t)((s_frame_tail + 1u) % DIY_IMU_FRAME_RING);
        s_stats.ring_overrun++;
    }
    s_frame_ring[s_frame_head] = *f;
    s_frame_head = next;
}

/* ---------------------------------------------------------------------------
 * 逐字节状态机：找 0xAA → 确认 0x55 → 收满 32 字节 → 校验 CRC → 换算 → 入队
 * ------------------------------------------------------------------------- */
static void DIY_FeedByte(uint8_t b)
{
    /* ---- 阶段1：找帧头 ---- */
    if (s_hdr_seen == 0u) {
        if (b == DIY_IMU_HDR0) {
            s_frame_buf[0] = b;
            s_frame_idx = 1u;
            s_hdr_seen = 1u;
        }
        return;
    }

    /* ---- 阶段2：第二个字节必须是 0x55 ---- */
    if (s_frame_idx == 1u) {
        if (b == DIY_IMU_HDR1) {
            s_frame_buf[1] = b;
            s_frame_idx = 2u;
        } else if (b == DIY_IMU_HDR0) {
            /* 连续两个 0xAA：把当前这个当作新帧头，继续等 0x55 */
            s_stats.resync++;
            s_frame_idx = 1u;
        } else {
            /* 假帧头，丢弃，重新找 */
            s_stats.resync++;
            s_hdr_seen = 0u;
            s_frame_idx = 0u;
        }
        return;
    }

    /* ---- 阶段3：收满一帧 ---- */
    s_frame_buf[s_frame_idx++] = b;
    if (s_frame_idx < DIY_IMU_FRAME_LEN) {
        return;
    }

    /* 收满，开始校验 */
    s_frame_idx = 0u;
    s_hdr_seen = 0u;

    {
        const uint8_t *f = s_frame_buf;

        /* 类型检查。将来扩展其它 type 时在这里分支。 */
        if (f[2] != DIY_IMU_TYPE_IMU) {
            s_stats.resync++;
            return;
        }

        /* CRC 校验：覆盖字节 2..29 */
        {
            uint16_t crc_want = (uint16_t)(f[DIY_IMU_CRC_OFF]
                                           | ((uint16_t)f[DIY_IMU_CRC_OFF + 1] << 8));
            uint16_t crc_got  = DIY_IMU_CRC16(&f[DIY_IMU_CRC_START], DIY_IMU_CRC_LEN);

            if (crc_want != crc_got) {
                s_stats.crc_err++;
                return;
            }
        }

        /* 校验通过：解析并换算 */
        {
            DIY_IMU_Frame_t fr;
            uint8_t i;

            memset(&fr, 0, sizeof(fr));

            fr.seq = f[3];
            fr.timestamp_ms = (uint32_t)f[4]
                            | ((uint32_t)f[5]  << 8)
                            | ((uint32_t)f[6]  << 16)
                            | ((uint32_t)f[7]  << 24);

            for (i = 0; i < 3u; i++) {
                /* 原始计数：小端 int16 */
                fr.acc_raw[i]  = (int16_t)((uint16_t)f[8u  + i * 2u]
                                           | ((uint16_t)f[9u  + i * 2u] << 8));
                fr.gyro_raw[i] = (int16_t)((uint16_t)f[14u + i * 2u]
                                           | ((uint16_t)f[15u + i * 2u] << 8));
                fr.rpy_cdeg[i] = (int16_t)((uint16_t)f[20u + i * 2u]
                                           | ((uint16_t)f[21u + i * 2u] << 8));

                /* ★ 精确整数换算（无浮点、无舍入误差）
                 *   mg     = raw * 16000 / 32768 = raw * 125 / 256
                 *   0.1dps = raw * 20000 / 32768 = raw * 625 / 1024
                 * 右移对负数是算术右移（ARM 上 int32 右移即向下取整），
                 * 与"除以 2^n 后取整"在负半轴会有 1 LSB 差异，
                 * 但该差异远小于传感器噪声，忽略。 */
                fr.acc_mg[i]     = (int16_t)(((int32_t)fr.acc_raw[i]  * 125) >> 8);
                fr.gyro_01dps[i] = (int16_t)(((int32_t)fr.gyro_raw[i] * 625) >> 10);

                /* 浮点值仅供显示调试 */
                fr.acc_g[i]    = (float)fr.acc_raw[i]  * (16.0f   / 32768.0f);
                fr.gyro_dps[i] = (float)fr.gyro_raw[i] * (2000.0f / 32768.0f);
            }

            fr.flags = (uint32_t)f[26]
                     | ((uint32_t)f[27] << 8)
                     | ((uint32_t)f[28] << 16)
                     | ((uint32_t)f[29] << 24);

            /* 序号连续性：跳变说明中间丢了帧 */
            if (s_last_seq_valid) {
                uint8_t expect = (uint8_t)(s_last_seq + 1u);
                if (fr.seq != expect) {
                    /* 累计丢失量（处理回绕） */
                    s_stats.seq_drop += (uint32_t)(uint8_t)(fr.seq - expect);
                }
            }
            s_last_seq = fr.seq;
            s_last_seq_valid = 1u;

            DIY_FramePush(&fr);
            s_stats.frames_ok++;

            /* 同步更新显示用快照（只拷显示需要的字段） */
            taskENTER_CRITICAL();
            {
                uint8_t k;
                for (k = 0; k < 3u; k++) {
                    s_latest.acc_raw[k]     = fr.acc_raw[k];
                    s_latest.gyro_raw[k]    = fr.gyro_raw[k];
                    s_latest.acc_mg[k]      = fr.acc_mg[k];
                    s_latest.gyro_01dps[k]  = fr.gyro_01dps[k];
                    s_latest.rpy_cdeg[k]    = fr.rpy_cdeg[k];
                }
                s_latest.flags   = fr.flags;
                s_latest.seq     = fr.seq;
                s_latest_valid = 1u;
            }
            taskEXIT_CRITICAL();
        }
    }
}

/* Public functions ----------------------------------------------------------*/

uint8_t DIY_IMU_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return DIY_IMU_ERR_NULL;
    }

    s_huart       = huart;
    s_dma_old_pos = 0u;
    s_byte_head   = 0u;
    s_byte_tail   = 0u;
    s_frame_head  = 0u;
    s_frame_tail  = 0u;
    s_frame_idx   = 0u;
    s_hdr_seen    = 0u;
    s_last_seq    = 0u;
    s_last_seq_valid = 0u;
    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.inited = 1u;

    /* 循环 DMA + 空闲中断：只启动一次，之后 HAL 每次给出绝对位置 */
    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, s_dma_buf, DIY_DMA_BUF_SIZE) != HAL_OK) {
        return DIY_IMU_ERR_NOT_INIT;
    }
    return DIY_IMU_OK;
}

void DIY_IMU_Reinit(void)
{
    if (s_huart == NULL) {
        return;
    }
    (void)HAL_UART_AbortReceive(s_huart);
    s_dma_old_pos = 0u;
    s_byte_head   = 0u;
    s_byte_tail   = 0u;
    s_frame_idx   = 0u;
    s_hdr_seen    = 0u;
    (void)HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_dma_buf, DIY_DMA_BUF_SIZE);
}

void DIY_IMU_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint16_t old = s_dma_old_pos;
    uint16_t n;

    if (s_huart == NULL || huart != s_huart) {
        return;
    }

    /* ★ 边界注意：HAL 在"缓冲区恰好被填满"时给出的 Size 等于缓冲长度，
     *   而不是 0。若直接与 DIY_DMA_BUF_SIZE 取模会变成 0，从而把整块数据
     *   当成"从 0 到 0"（即什么都不搬）而丢掉。所以这里先夹到 0 再比较。 */
    if (Size >= DIY_DMA_BUF_SIZE) {
        Size = 0u;
    }

    if (Size > old) {
        /* 正常推进：搬 [old, Size) */
        uint16_t i;
        for (i = old; i < Size; i++) {
            DIY_BytePush(s_dma_buf[i]);
        }
        n = (uint16_t)(Size - old);
    } else if (Size < old) {
        /* DMA 缓冲回卷：先搬尾部 [old, SIZE)，再搬头部 [0, Size) */
        uint16_t i;
        for (i = old; i < DIY_DMA_BUF_SIZE; i++) {
            DIY_BytePush(s_dma_buf[i]);
        }
        for (i = 0; i < Size; i++) {
            DIY_BytePush(s_dma_buf[i]);
        }
        n = (uint16_t)(DIY_DMA_BUF_SIZE - old + Size);
    } else {
        /* 位置没变：两次事件之间没有新数据 */
        n = 0u;
    }

    s_dma_old_pos = Size;
    s_stats.rx_bytes += n;
    s_stats.last_rx_tick = HAL_GetTick();
}

uint16_t DIY_IMU_Process(void)
{
    uint8_t  b;
    uint32_t before = s_stats.frames_ok;

    while (DIY_BytePop(&b)) {
        DIY_FeedByte(b);
    }
    return (uint16_t)(s_stats.frames_ok - before);
}

uint8_t DIY_IMU_GetLatest(DIY_IMU_Display_t *out)
{
    uint8_t ok;

    taskENTER_CRITICAL();
    ok = s_latest_valid;
    if (ok && out != NULL) {
        *out = s_latest;
    }
    taskEXIT_CRITICAL();

    return ok;
}

uint8_t DIY_IMU_TakeFrame(DIY_IMU_Frame_t *out)
{
    uint8_t got = 0u;

    taskENTER_CRITICAL();
    if (s_frame_tail != s_frame_head) {
        if (out != NULL) {
            *out = s_frame_ring[s_frame_tail];
        }
        s_frame_tail = (uint16_t)((s_frame_tail + 1u) % DIY_IMU_FRAME_RING);
        got = 1u;
    }
    taskEXIT_CRITICAL();

    return got;
}

void DIY_IMU_GetStats(DIY_IMU_Stats_t *st)
{
    if (st == NULL) {
        return;
    }
    taskENTER_CRITICAL();
    *st = s_stats;
    taskEXIT_CRITICAL();
}

void DIY_IMU_ResetStats(void)
{
    taskENTER_CRITICAL();
    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.inited = (s_huart != NULL) ? 1u : 0u;
    taskEXIT_CRITICAL();

    s_last_seq_valid = 0u;
}

uint32_t DIY_IMU_GetLastRxAge(uint32_t now_ms)
{
    uint32_t last = s_stats.last_rx_tick;

    if (last == 0u) {
        return 0xFFFFFFFFu;         /* 从未收到过 */
    }
    return (uint32_t)(now_ms - last);
}
