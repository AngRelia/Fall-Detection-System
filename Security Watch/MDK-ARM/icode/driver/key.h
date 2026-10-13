#ifndef __KEY_H
#define __KEY_H

#include "main.h"

/* ===========================================================================
 * key.h —— 4 个菜单导航按键
 *
 * 引脚（详见 docs/按键接线与引脚分配.md）：
 *   KEY_1 = PA0    KEY_2 = PA1    KEY_3 = PA11    KEY_4 = PA12
 *   全部 GPIO 输入 + 内部上拉，按键另一端接 GND，低电平有效，无需外部电阻。
 *
 * 分工（用户定稿：做 OLED 菜单导航）：
 *   KEY_1 = 上      KEY_2 = 下      KEY_3 = 确认      KEY_4 = 返回
 *
 * 用法（三步）：
 *   1) **一个**任务里每 10 ms 调一次 Key_Scan()：消抖、边沿、长按判定与连发都在里面做；
 *   2) 菜单任务用 Key_GetEvent() 取事件来翻页 / 进入 / 返回；
 *   3) 告警取消逻辑先用 Key_PeekEvent() 看一眼队首是不是 KEY_EVENT_BACK_LONG，
 *      确认是给自己的再 Key_GetEvent() 取走，免得把事件从菜单手里抢走。
 *
 * ? Key_Scan() 全工程只能有一个调用者，否则按住时长会被两个任务各算一遍，长按与连发全部失准。
 * ? 事件队列是"单生产者（Key_Scan 调用者）/ 单消费者（取事件的任务）"模型。
 *
 * 自检接口（本文件最后那几个 Key_GetXxx）供"按键检测页"用：**只读、不消费事件**，
 * 因此在任何任务里调都不会影响菜单/告警的事件流。
 * ========================================================================= */

typedef enum
{
    KEY_EVENT_NONE = 0,
    KEY_EVENT_UP,            /* 上：短按 */
    KEY_EVENT_DOWN,          /* 下：短按 */
    KEY_EVENT_OK,            /* 确认：短按 */
    KEY_EVENT_BACK,          /* 返回：短按 */
    KEY_EVENT_UP_REPEAT,     /* 上：长按连发（菜单快速滚动） */
    KEY_EVENT_DOWN_REPEAT,   /* 下：长按连发 */
    KEY_EVENT_BACK_LONG      /* 返回：长按（建议作为"取消告警"，对应题面要求 5） */
} Key_Event_t;

/* 按键编号：与硬件 KEY_n 一一对应，便于日志与数据打标 */
#define KEY_ID_UP        1U
#define KEY_ID_DOWN      2U
#define KEY_ID_OK        3U
#define KEY_ID_BACK      4U

/* 按键个数（KEY_1..KEY_4） */
#define KEY_NUM_MAX      4U

/* 清内部状态。引脚本身已由 MX_GPIO_Init() 配好，这里不重复配 GPIO。 */
void Key_Init(void);

/* 每 10 ms 调一次（必须只有一个调用者） */
void Key_Scan(void);

/* 看队首事件但不取走；无事件返回 KEY_EVENT_NONE */
Key_Event_t Key_PeekEvent(void);

/* 取走队首事件；无事件返回 KEY_EVENT_NONE */
Key_Event_t Key_GetEvent(void);

/* ---- 兼容旧接口 ---- */

/* 0 = 无键按下；1~4 = 当前按住的键号（用的是消抖后的状态，比直读引脚稳） */
uint8_t Key_GetState(void);

/* 取最近一次"按下"的键号并清零（消费型）。旧版这个函数恒返回 0，本次已修好。 */
uint8_t Key_GetNum(void);

/* ---- 自检 / 显示接口（只读，不消费事件）---- */

/* 原始电平位图：bit0=KEY_1 … bit3=KEY_4，**1 = 引脚被拉低 = 按键按下**。
   这是"消抖前"的瞬时值，接线对不对（按下去必须变 1）靠它一眼看出来。 */
uint8_t Key_GetRawMask(void);

/* 第 idx 个键（0~3）当前已按住的毫秒数；未按住返回 0 */
uint16_t Key_GetHoldMs(uint8_t idx);

/* 第 idx 个键（0~3）累计的【按下次数】（短按+长按都算一次按下） */
uint16_t Key_GetPressCount(uint8_t idx);

/* 第 idx 个键（0~3）累计的【长按次数】 */
uint16_t Key_GetLongCount(uint8_t idx);

/* Key_Scan() 累计被调用的次数。正常情况下它应按 100 次/秒 稳定增长，
   屏幕上它不动就说明"扫描调用者"（当前是 alarm_task）没在跑。 */
uint32_t Key_GetScanCount(void);

/* 累计产生的事件个数（含短按/长按/连发），用于判断事件队列是否在被消费 */
uint32_t Key_GetEventCount(void);

/* 最近一次产生的事件（只读，不消费队列），供检测页显示 */
Key_Event_t Key_GetLastEvent(void);

/* 事件短名（固定 5 字符，便于直接覆盖显示，不会残留旧字符）：
     "-----" / "UP   " / "DOWN " / "OK   " / "BACK " / "U-REP" / "D-REP" / "B-LNG" */
char *Key_EventName(Key_Event_t e);

/* ---- 导航事件锁存（供"翻页 / 菜单"用）----
   Key_GetEvent() 只能从队列头部取；若队列里先堆了 OK/BACK，后面的 UP/DOWN 就取不出来。
   所以这里额外锁存"最近一次导航事件"（UP/DOWN 及其连发），由 OLED/菜单任务单独取用：
     · 翻页方：Key_TakeNav()   —— 取走并清零
     · 告警方：Key_PeekEvent() —— 只对 BACK_LONG 做反应，队列消费权仍归它
   两条路径互不干扰（不会出现两个任务抢同一队列），连按上/下也不会被夹在中间的别的键卡住。 */
Key_Event_t Key_TakeNav(void);

#endif
