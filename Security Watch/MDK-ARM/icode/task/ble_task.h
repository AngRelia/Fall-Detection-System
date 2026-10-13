#ifndef __BLE_TASK_H__
#define __BLE_TASK_H__

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ble_task —— BLE 上报任务（HC-08 透传）
 *
 * 职责：
 *   1) 上电驱动 HC-08（USART2 透传），可选发 AT 探测/改名（见 ble_task.c 顶部开关）
 *   2) 把其它模块投递的报文发出去：`BLE_Report(json)`（单槽暂存，非阻塞）
 *   3) 接收手机下发的命令行，解析极简指令集（PING / STAT / AT...），
 *      可选把收到的行回显给手机（联调时验证双向通信用）
 *
 * 报文格式（与 docs/HC-08接入方案.md 一致，一行一条 JSON）：
 *   {"t":1712345678,"dev":"JBX-01","ev":"FALL","conf":93,
 *    "acc":[1200,-2800,3900],"rpy":[821,-153,76],"step":98,"seq":4521,
 *    "gps":{"v":1,"lat":399876543,"lon":1163123456,"q":1,"sat":9,
 *           "hdop":95,"alt":4300,"spd":634,"cog":0,"fix_age":1200}}
 *   gps 片段直接调 GPS_BuildJson(buf, len, HAL_GetTick()) 生成；标度见 gps.h。
 *
 * ⚠ HC-08 是 CC2541 + 私有透传固件，不是标准 BLE 串口 GATT 服务：
 *   官方"汇承"App 或通用 BLE 调试 App（nRF Connect / BLE 串口助手）可以连，
 *   自研 App 需要先用 nRF Connect 查出服务/特征 UUID 再对接。
 *   若后续要做通用自研 App，换 JDY-31 / AT-09 等标准 GATT 模块更省事。
 */

/* 任务句柄（由 _Task_Init 创建后赋值） */
extern TaskHandle_t BLETaskHandle;

/* 创建任务并分配优先级与栈大小，由 freertos.c 的 MX_FREERTOS_Init() 调用。 */
void BLE_Task_Init(osPriority_t prio, uint32_t stack);

/* ---- 供其它模块使用的发送接口 ---- */
/* 投递一条报文（自动补 CRLF）。非阻塞：拷进单槽暂存区，由 BLE 任务发出。
   返回 1 = 已受理；0 = 上一帧还没发完 / 报文过长（会被计入丢弃数，见 BLE_GetStats）。 */
uint8_t BLE_Report(const char *line);

/* 统计：ok = 成功发出的帧数，drop = 因暂存槽忙或过长而被丢弃的帧数 */
void BLE_GetStats(uint32_t *ok, uint32_t *drop);

#ifdef __cplusplus
}
#endif

#endif /* __BLE_TASK_H__ */
