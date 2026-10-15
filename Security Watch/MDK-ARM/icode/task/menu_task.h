#ifndef __MENU_TASK_H__
#define __MENU_TASK_H__

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * menu_task —— 菜单/状态总览任务（对应 docs/菜单与交互设计.md 第 1~2 步）
 *
 * 职责边界（很重要，见设计文档 §1）：
 *   · 本任务只负责"决定显示什么"：维护当前页号，并周期性把各子系统快照汇总成一个
 *     Menu_View_t；**不直接刷屏**（OLED 是软件 I2C，绘制只归 oled_task）。
 *   · 按键归属：上/下用 Key_TakeNav() 的导航锁存（不占用事件队列）；
 *     队列的消费者仍然只有一个（将来是菜单；当前告警任务只 Peek 长按返回）。
 *
 * 双缓冲：写者(本任务)填另一份缓冲再原子翻转索引，读者(oled_task)拷贝当前这份，
 * 因此不会读到"写一半"的数据（与 gps.c 的 seqlock 同一思路，但这里只需双缓冲）。
 * ------------------------------------------------------------------------- */

#define MENU_PAGE_NUM   4       /* 0=总览 1=节点A 2=GPS 3=BLE */

/* 汇总视图：字段刻意保持"整数 + 少量状态位"，便于 OLED 定点绘制 */
typedef struct
{
    uint8_t  page;              /* 当前页号 0..MENU_PAGE_NUM-1 */

    /* ---- 节点 A（IMU 链路，imu_recv_task）---- */
    uint16_t a_rate;            /* 实测帧率 Hz */
    uint8_t  a_lost;            /* 1 = 超时未收到数据 */
    uint32_t a_ok;              /* 校验通过帧数 */
    uint32_t a_crc;             /* CRC 错误数 */
    uint32_t a_drop;            /* 序号跳变（估算丢帧） */
    uint32_t a_rsy;             /* 帧头重同步次数 */
    uint32_t a_rovr;            /* 帧环形缓冲覆盖次数 */

    /* ---- GPS（USART1）---- */
    uint8_t  g_alive;           /* 1 = 5 s 内收到过字节 */
    uint8_t  g_fix;             /* 1 = 定位有效 */
    uint8_t  g_q;              /* GGA quality */
    uint8_t  g_st;              /* 参与定位卫星数 */
    uint8_t  g_vw;              /* 可见卫星数 */
    uint8_t  g_mx;              /* 最强信噪比 dBHz */
    int32_t  g_lat_e7;          /* 纬度 度×1e7 */
    int32_t  g_lon_e7;          /* 经度 度×1e7 */
    uint32_t g_rx;              /* 累计收到字节 */
    uint32_t g_sen;             /* 通过校验语句数 */
    uint32_t g_cse;             /* 校验错语句数 */

    /* ---- BLE（USART2 / HC-08）---- */
    uint8_t  b_conn;            /* 1 = 模块 STATE 脚为已连接 */
    uint32_t b_txb;             /* 累计发送字节 */
    uint32_t b_rxb;             /* 累计接收字节 */
    uint32_t b_atok;            /* 模块回 OK 次数 */
    uint32_t b_aterr;           /* 模块回 ERROR/FAIL 次数 */
    uint32_t b_send;            /* 成功上报帧数 */
    uint32_t b_drop;            /* 上报丢弃帧数 */
    char     b_last[12];        /* 最近收到的一行（截断显示） */

    /* ---- 系统 ---- */
    uint32_t heap;              /* 剩余 FreeRTOS 堆字节 */
    uint32_t uptime_s;          /* 运行秒数 */
} Menu_View_t;

/* 任务句柄（由 _Task_Init 创建后赋值） */
extern TaskHandle_t MenuTaskHandle;

/* 创建任务：优先级与栈大小由 freertos.c 统一传入 */
void Menu_Task_Init(osPriority_t prio, uint32_t stack);

/* 取当前视图（供 oled_task 绘制；无锁，双缓冲保证不会读到写一半的数据） */
void Menu_GetView(Menu_View_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __MENU_TASK_H__ */
