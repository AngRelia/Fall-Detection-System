/**
  ******************************************************************************
  * @file    diy_imu.h
  * @brief   DIY IMU（节点A）串口数据链路驱动
  ******************************************************************************
  * 硬件与链路：
  *   节点A (自制 ICM-42688 + MMC5603 板)  --USART3-->  本板 (Security Watch)
  *   USART3: PB10/PB11, 115200 8N1, DMA1_Channel3 循环接收 + 空闲中断
  *
  *   接线（交叉）：
  *     节点A PA2 (TX)  -->  本板 PB11 (USART3_RX)
  *     节点A PA3 (RX)  <--  本板 PB10 (USART3_TX)   [可选，命令回传]
  *     GND             ---  GND                     [必须共地]
  *
  * 帧格式（定长 32 字节，小端）——与节点A的 docs/串口帧协议.md 一致：
  *   [ 0] uint8  hdr0      = 0xAA
  *   [ 1] uint8  hdr1      = 0x55
  *   [ 2] uint8  type      = 0x01
  *   [ 3] uint8  seq       序号，回绕
  *   [ 4] uint32 timestamp_ms
  *   [ 8] int16  acc[3]    原始 ADC 计数（±16 g 量程）
  *   [14] int16  gyro[3]   原始 ADC 计数（±2000 °/s 量程）
  *   [20] int16  rpy[3]    0.01 °（六轴下 yaw 为相对上电基准的转角）
  *   [26] uint32 flags     状态位
  *   [30] uint16 crc16     CRC-16/MODBUS，覆盖字节 2..29
  *
  * ★ 单位换算（本驱动的核心职责之一）：
  *   帧里发的是**原始 ADC 计数**（零量化损失，主机可自行换算），
  *   但本工程的 TinyML 与疲劳判定模块约定的是工程量（见 ml_features.h）：
  *       acc  : mg      （±16 g 量程，1 g = 1000 mg）
  *       gyro : 0.1 °/s （±2000 °/s 量程，1 °/s = 10 LSB）
  *   两者可以**精确整数换算**，无浮点、无舍入误差：
  *       mg      = raw × 16000 / 32768 = raw × 125 / 256   → (raw * 125) >> 8
  *       0.1dps  = raw × 20000 / 32768 = raw × 625 / 1024  → (raw * 625) >> 10
  *
  *   推导：满量程 32768 计数对应 16 g = 16000 mg，故 16000/32768 = 125/256；
  *         32768 计数对应 2000 °/s = 20000 × (0.1°/s)，故 20000/32768 = 625/1024。
  *   量程上限检查：16000 mg 与 20000 LSB 都 < int16 上限 32767，不会溢出。
  ******************************************************************************
  */
#ifndef __DIY_IMU_H__
#define __DIY_IMU_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- 协议常量 */
#define DIY_IMU_HDR0            0xAAu
#define DIY_IMU_HDR1            0x55u
#define DIY_IMU_TYPE_IMU        0x01u
#define DIY_IMU_FRAME_LEN       32u     /* 帧总长 */
#define DIY_IMU_CRC_OFF         30u     /* crc16 偏移 */
#define DIY_IMU_CRC_START       2u      /* CRC 覆盖起始 */
#define DIY_IMU_CRC_LEN         28u     /* CRC 覆盖长度（2..29） */

/* flags 状态位（与节点A的 proto.h 对应） */
#define DIY_IMU_FLAG_FUSION_READY   (1uL << 0)
#define DIY_IMU_FLAG_MAG_ENABLED    (1uL << 1)
#define DIY_IMU_FLAG_GYRO_AZ_DONE   (1uL << 2)
#define DIY_IMU_FLAG_HEADING_SETTLE (1uL << 3)
#define DIY_IMU_FLAG_MAG_REF_VALID  (1uL << 4)
#define DIY_IMU_FLAG_CALIB_OK       (1uL << 5)
#define DIY_IMU_FLAG_MAG_INIT_ERR   (1uL << 6)

/* 帧环形缓冲深度。100 Hz 下 16 帧 = 160 ms 余量，
   足以吸收 TinyML 单次推理（约 20 ms）造成的排空停顿。 */
#define DIY_IMU_FRAME_RING      8u

/* 状态码 */
#define DIY_IMU_OK              0u
#define DIY_IMU_ERR_NULL        1u
#define DIY_IMU_ERR_NOT_INIT    2u

/* ------------------------------------------------------------------ 数据结构 */

/** 一帧解析结果与统计快照 */
typedef struct {
    /* ---- 本帧原始计数（无损，直接来自节点A） ---- */
    int16_t  acc_raw[3];        /* ±16 g 量程 */
    int16_t  gyro_raw[3];       /* ±2000 °/s 量程 */

    /* ---- 还原为物理量的浮点值（供显示/调试，不参与 TinyML） ---- */
    float    acc_g[3];          /* g   */
    float    gyro_dps[3];       /* °/s */

    /* ---- 姿态角，单位 0.01 °（六轴下 yaw 为相对上电基准的转角） ---- */
    int16_t  rpy_cdeg[3];       /* 0.01° */

    uint32_t timestamp_ms;      /* 节点A的 HAL_GetTick */
    uint8_t  seq;               /* 帧序号 */
    uint32_t flags;             /* 状态位 */

    /* ---- 换算后的工程量，单位与 ml_features.h 约定一致 ★ ---- */
    int16_t  acc_mg[3];         /* mg      */
    int16_t  gyro_01dps[3];     /* 0.1 °/s */
} DIY_IMU_Frame_t;

/** 链路统计，供 OLED 与论文实测数据表使用 */
typedef struct {
    uint32_t frames_ok;         /* 校验通过的帧数 */
    uint32_t crc_err;           /* CRC 校验失败次数 */
    uint32_t resync;            /* 帧头重同步次数（丢字节/丢帧的间接指标） */
    uint32_t seq_drop;          /* 序号跳变累计（估计丢了多少帧） */
    uint32_t ring_overrun;      /* 帧环形缓冲覆盖（消费不及时） */
    uint32_t rx_bytes;          /* 收到的原始字节数 */
    uint32_t byte_ring_drop;    /* 字节环形缓冲溢出丢弃数 */
    uint8_t  inited;            /* 1 = 已初始化 */
    uint8_t  last_seq_valid;    /* 1 = last_seq 有效 */
    uint8_t  last_seq;
    uint32_t last_rx_tick;      /* 最后一次收到字节的 tick */
} DIY_IMU_Stats_t;

/* ------------------------------------------------------------------ 接口 */

/**
  * @brief  绑定串口并启动「循环 DMA + 空闲中断」接收
  * @param  huart  USART3 句柄（&huart3）
  * @retval DIY_IMU_OK / 错误码
  * @note   调用前 CubeMX 必须已给该串口配好 RX DMA(Circular) 与全局中断。
  */
uint8_t DIY_IMU_Init(UART_HandleTypeDef *huart);

/**
  * @brief  由 HAL_UARTEx_RxEventCallback 调用，把 DMA 收到的字节搬进软件环形缓冲
  * @param  huart 触发事件的串口
  * @param  Size  该串口 DMA 缓冲中的**数据总量**（不是本次新增量）
  * @note   在中断上下文执行，只做搬运，不解析。
  */
void DIY_IMU_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

/**
  * @brief  排空字节缓冲并解析出完整帧（在任务上下文调用）
  * @retval 本次新解析并入队的帧数
  */
uint16_t DIY_IMU_Process(void);

/**
  * @brief  读取【最新一帧】的快照，供 OLED 等显示模块使用（非阻塞）
  * @param  out 输出
  * @retval 1 = 有有效帧；0 = 还没收到过任何帧
  *
  * @note   与 DIY_IMU_TakeFrame() 的区别（很重要）：
  *           TakeFrame —— 从帧队列【取走】一帧。消费者是 imu_recv_task，
  *                        取走即消失，用于喂 TinyML。
  *           GetLatest —— 只是【看一眼】最近一帧，不消费队列。
  *
  *         显示任务绝不能用 TakeFrame：那会和 imu_recv_task 抢同一个队列，
  *         导致 TinyML 随机少喂数据，而且丢帧取决于刷屏节奏，极难排查。
  */
/** 显示用精简快照 —— 只含 OLED 要画的字段。
  * 为什么不直接用 DIY_IMU_Frame_t：那个结构体有 113 字节（含 acc_g/gyro_dps
  * 等浮点量），而显示只需要整数与 flags。本机 RAM 已到极限，每个字节都要算。 */
typedef struct {
    int16_t  acc_raw[3];
    int16_t  gyro_raw[3];
    int16_t  acc_mg[3];
    int16_t  gyro_01dps[3];
    int16_t  rpy_cdeg[3];
    uint32_t flags;
    uint8_t  seq;
} DIY_IMU_Display_t;

uint8_t DIY_IMU_GetLatest(DIY_IMU_Display_t *out);

/**
  * @brief  取出一帧（非阻塞）
  * @param  out 输出，可为 NULL（此时只出队）
  * @retval 1 = 取到；0 = 无可用帧
  */
uint8_t DIY_IMU_TakeFrame(DIY_IMU_Frame_t *out);

/** 读取统计快照（临界区保护） */
void DIY_IMU_GetStats(DIY_IMU_Stats_t *st);

/** 清空统计（保留 inited） */
void DIY_IMU_ResetStats(void);

/**
  * @brief  距最后一次收到字节的毫秒数，用于判断链路是否还活着
  * @param  now_ms 当前 tick
  */
uint32_t DIY_IMU_GetLastRxAge(uint32_t now_ms);

/**
  * @brief  链路异常时重新武装接收（先 Abort 再启动，重复调用安全）
  */
void DIY_IMU_Reinit(void);

/**
  * @brief  CRC-16/MODBUS 计算（多项式 0x8005 反射 0xA001，初值 0xFFFF）
  * @note   与节点A的 Proto_CRC16 必须完全一致，否则所有帧都会被判为坏帧。
  */
uint16_t DIY_IMU_CRC16(const uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __DIY_IMU_H__ */
