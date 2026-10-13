#ifndef __GPS_TASK_H__
#define __GPS_TASK_H__

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * gps_task —— GNSS 定位任务（对应课题要求 4 的"附带 GPS 位置信息"）
 *
 * 职责：
 *   1) 绑定 GPS 串口并启动「循环 DMA + 空闲中断」接收
 *   2) 周期调用 GPS_Process() 把 NMEA 语句解析成最新定位快照
 *   3) 上电后发送 $PCAS03 精简输出语句（只留 GGA + RMC）
 *   4) 链路健康检查：长时间收不到字节就重新武装接收
 *
 * 与其它模块的关系：
 *   · 本任务**只写**定位快照，不直接发 BLE / 不刷 OLED，避免多任务抢外设
 *   · ble_task 在告警确认时调用 GPS_BuildJson() 取位置片段
 *   · oled_task 可用 GPS_FormatPosition() 显示经纬度
 *
 * ⚠ 接线前提（详见 docs/GPS模块接入方案.md）：
 *   GPS TXD → MCU RX，GPS RXD → MCU TX，双方共地；
 *   GPS 底板 VDD = 3.6~6 V（典型 5 V），不能用 3.3 V 供电；
 *   VBAT 接 3 V 备电可保住热启动，ON/OFF 高电平=工作。
 *
 * ⚠ 需要在 main.c 的 USER CODE BEGIN 4 里把接收事件转发给驱动：
 *   void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
 *   {
 *     GPS_RxEventCallback(huart, Size);
 *   }
 */

/* 任务句柄（由 _Task_Init 创建后赋值） */
extern TaskHandle_t GPSTaskHandle;

/* 创建任务并分配优先级与栈大小，由 freertos.c 的 MX_FREERTOS_Init() 调用。
   prio  : osPriority_t 枚举值
   stack : 栈深度，单位 word（1 word = 4 字节） */
void GPS_Task_Init(osPriority_t prio, uint32_t stack);

#ifdef __cplusplus
}
#endif

#endif /* __GPS_TASK_H__ */
