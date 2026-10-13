#ifndef __IMU_RECV_TASK_H__
#define __IMU_RECV_TASK_H__

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "queue.h"
#include "diy_imu.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * imu_recv_task —— DIY IMU（节点A）数据接收解析任务
 *
 * 职责：
 *   1) 从节点A接收 32 字节定长 IMU 数据帧（见 diy_imu.h 的帧格式说明）
 *   2) 排空字节缓冲并解析，校验 CRC-16/MODBUS
 *   3) 单位换算成 mg / 0.1°/s 后喂给 tinyml_task（TinyML_PushFrame）
 *   4) 统计帧率、丢帧数、CRC 错误数，供 OLED 与论文实测数据表使用
 *
 * 设计要点：
 *   - 驱动层（diy_imu.c）用「循环 DMA + 空闲中断」收字节，任务只消费已落的字节
 *   - 本任务优先级高于 tinyml_task：数据先落地，避免因推理耗时导致丢帧
 *   - 不直接刷 OLED：软件 I2C 很慢，会和 oled_task 抢总线。统计数据由
 *     oled_task 通过 IMU_Recv_GetSnapshot() 自取。
 *
 * ★ 数据单位契约（与 ml_features.h / tinyml_task.h 一致，改动必须三处同步）：
 *      acc  : mg      （±16 g 量程，1 g = 1000 mg）
 *      gyro : 0.1 °/s （±2000 °/s 量程，1 °/s = 10 LSB）
 *   帧里传的是原始 ADC 计数，换算在 diy_imu.c 里以整数精确完成。
 *
 * ⚠️ TinyML 的滑动窗口按 100 Hz 设计（ML_WIN=200 → 2 s）。
 *    若实测节点A的帧率明显低于 100 Hz，窗口实际跨度会超过 2 s，
 *    29 维特征里的时间相关量（过零率、jerk、方差）都会系统性偏移。
 *    所以本任务上报的 frame_rate 是必须实测确认的指标，不能想当然。
 */

/* 任务句柄（由 IMU_Recv_Task_Init 创建后赋值） */
extern TaskHandle_t IMU_RecvTaskHandle;

/* 创建任务并分配优先级与栈大小，由 freertos.c 的 MX_FREERTOS_Init() 调用。
   prio  : osPriority_t 枚举值
   stack : 栈深度，单位 word（1 word = 4 字节） */
void IMU_Recv_Task_Init(osPriority_t prio, uint32_t stack);

/* 链路运行快照，供 OLED 显示页使用 */
typedef struct {
    uint32_t frames_ok;         /* 累计校验通过帧数 */
    uint32_t crc_err;           /* CRC 错误累计 */
    uint32_t seq_drop;          /* 序号跳变累计（估算丢帧数） */
    uint32_t resync;            /* 帧头重同步次数 */
    uint32_t ring_overrun;      /* 帧环形缓冲覆盖次数 */
    uint32_t byte_ring_drop;    /* 字节缓冲溢出丢弃数 */
    uint16_t frame_rate;        /* 实测帧率（Hz，四舍五入） */
    uint8_t  link_lost;         /* 1 = 超过阈值没收到数据 */
    uint8_t  inited;            /* 1 = 驱动已初始化 */
} IMU_Recv_Snapshot_t;

/* 读取链路运行快照（任务安全） */
void IMU_Recv_GetSnapshot(IMU_Recv_Snapshot_t *snap);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_RECV_TASK_H__ */
