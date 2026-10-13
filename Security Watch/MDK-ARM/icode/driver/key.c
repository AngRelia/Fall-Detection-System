#include "key.h"

/* ===========================================================================
 * key.c —— 4 键消抖 / 边沿 / 长按连发 → 菜单事件队列
 * 设计说明与用法见 key.h
 *
 * 时序全部以 Key_Scan() 的调用周期为基准（默认 10 ms 一次）：
 *   消抖 30 ms  →  按下沿立即出短按事件  →  按住 500 ms 起连发（每 150 ms）
 *                                        →  按住 1000 ms 出长按事件
 * ========================================================================= */

/* ---- 时间参数（单位 ms） ---- */
#define KEY_SCAN_MS            10U     /* Key_Scan 调用周期 */
#define KEY_DEBOUNCE_MS        30U     /* 消抖：连续 3 次同值才算稳定 */
#define KEY_REPEAT_DELAY_MS    500U    /* 上/下键：按住多久后开始连发 */
#define KEY_REPEAT_MS          150U    /* 连发间隔 */
#define KEY_LONG_MS            1000U   /* 长按判定（返回键用它做"取消告警"） */

#define KEY_DEBOUNCE_CNT       (KEY_DEBOUNCE_MS   / KEY_SCAN_MS)
#define KEY_REPEAT_DELAY_CNT   (KEY_REPEAT_DELAY_MS / KEY_SCAN_MS)
#define KEY_REPEAT_CNT         (KEY_REPEAT_MS     / KEY_SCAN_MS)
#define KEY_LONG_CNT           (KEY_LONG_MS       / KEY_SCAN_MS)

#define KEY_NUM                4U      /* 按键个数 */
#define KEY_EVT_QUEUE_LEN      8U      /* 事件队列深度（菜单一次最多积压 8 个事件） */

/* ---------------------------------------------------------------------------
 * 按键表：改分工只改这张表（端口 / 引脚 / 短按事件 / 连发事件 / 长按事件）
 * ------------------------------------------------------------------------- */
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
    Key_Event_t   ev_short;
    Key_Event_t   ev_repeat;
    Key_Event_t   ev_long;
} Key_Map_t;

static const Key_Map_t s_map[KEY_NUM] =
{
    { KEY_1_GPIO_Port, KEY_1_Pin, KEY_EVENT_UP,   KEY_EVENT_UP_REPEAT,   KEY_EVENT_NONE      },
    { KEY_2_GPIO_Port, KEY_2_Pin, KEY_EVENT_DOWN, KEY_EVENT_DOWN_REPEAT, KEY_EVENT_NONE      },
    { KEY_3_GPIO_Port, KEY_3_Pin, KEY_EVENT_OK,   KEY_EVENT_NONE,        KEY_EVENT_NONE      },
    { KEY_4_GPIO_Port, KEY_4_Pin, KEY_EVENT_BACK, KEY_EVENT_NONE,        KEY_EVENT_BACK_LONG },
};

/* ---- 每个键的状态 ---- */
static uint8_t  s_stable[KEY_NUM];     /* 消抖后的稳定电平：0=松开 1=按住 */
static uint8_t  s_raw_last[KEY_NUM];   /* 上一次原始采样 */
static uint8_t  s_raw_cnt[KEY_NUM];    /* 连续相同原始采样的次数 */
static uint16_t s_hold_ms[KEY_NUM];    /* 已按住时长（ms） */
static uint8_t  s_long_sent[KEY_NUM];  /* 长按事件是否已发（防止重复/松开再发） */

/* ---- 自检统计（供按键检测页显示，全部只增不减） ---- */
static uint8_t  s_raw_mask   = 0U;     /* bit0..bit3：原始电平，1=按下 */
static uint16_t s_press_cnt[KEY_NUM];  /* 按下次数 */
static uint16_t s_long_cnt[KEY_NUM];   /* 长按次数 */
static uint32_t s_scan_cnt   = 0U;     /* Key_Scan 调用次数 */
static uint32_t s_evt_cnt    = 0U;     /* 累计事件数 */
static Key_Event_t s_last_evt = KEY_EVENT_NONE;   /* 最近一次事件 */

/* ---- 事件队列 ---- */
static volatile Key_Event_t s_evt[KEY_EVT_QUEUE_LEN];
static volatile uint8_t     s_evt_head = 0U;
static volatile uint8_t     s_evt_tail = 0U;

/* ---- 兼容旧接口用：最近一次按下的键号 ---- */
static uint8_t s_pressed_num = 0U;

/* ---- 导航事件锁存（UP/DOWN 及连发），供 OLED/菜单翻页用 ---- */
static volatile Key_Event_t s_nav_evt = KEY_EVENT_NONE;

/* ---------------------------------------------------------------------------
 * 内部函数
 * ------------------------------------------------------------------------- */

/* 入队；队列满则丢弃新事件（宁可丢新，也不打乱已有顺序）。
   同时记录"最近一次事件"与累计事件数，供检测页显示（不影响队列）。 */
static void Key_PushEvent(Key_Event_t e)
{
    uint8_t next = (uint8_t)((s_evt_head + 1U) % KEY_EVT_QUEUE_LEN);

    s_last_evt = e;
    s_evt_cnt++;

    /* 顺手锁存导航事件（只影响 Key_TakeNav，不动队列本身） */
    switch (e)
    {
        case KEY_EVENT_UP:
        case KEY_EVENT_DOWN:
        case KEY_EVENT_UP_REPEAT:
        case KEY_EVENT_DOWN_REPEAT:
            s_nav_evt = e;
            break;
        default:
            break;
    }

    if (next == s_evt_tail)
    {
        return;
    }
    s_evt[s_evt_head] = e;
    s_evt_head = next;
}

/* 清空队列与所有按键状态 */
void Key_Init(void)
{
    uint8_t i;

    for (i = 0U; i < KEY_NUM; i++)
    {
        s_stable[i]    = 0U;
        s_raw_last[i]  = 0U;
        s_raw_cnt[i]   = 0U;
        s_hold_ms[i]   = 0U;
        s_long_sent[i] = 0U;
        s_press_cnt[i] = 0U;
        s_long_cnt[i]  = 0U;
    }
    s_evt_head    = 0U;
    s_evt_tail    = 0U;
    s_pressed_num = 0U;
    s_raw_mask    = 0U;
    s_last_evt    = KEY_EVENT_NONE;
    /* 统计量（s_scan_cnt / s_evt_cnt）刻意不清零：它们反映"上电以来"的累计，
       清掉反而看不出"扫描是否一直在跑"。 */
}

/* ---------------------------------------------------------------------------
 * 周期扫描：消抖 → 边沿 → 长按/连发
 * ------------------------------------------------------------------------- */
void Key_Scan(void)
{
    uint8_t i;
    uint8_t mask = 0U;

    s_scan_cnt++;

    for (i = 0U; i < KEY_NUM; i++)
    {
        /* 低电平有效：读到 RESET 表示按住 */
        uint8_t raw = (HAL_GPIO_ReadPin(s_map[i].port, s_map[i].pin) == GPIO_PIN_RESET) ? 1U : 0U;

        if (raw != 0U)
        {
            mask |= (uint8_t)(1U << i);        /* 原始电平位图（消抖前） */
        }

        /* --- 1. 消抖：连续 KEY_DEBOUNCE_CNT 次同值才认 --- */
        if (raw == s_raw_last[i])
        {
            if (s_raw_cnt[i] < KEY_DEBOUNCE_CNT)
            {
                s_raw_cnt[i]++;
            }
        }
        else
        {
            s_raw_last[i] = raw;
            s_raw_cnt[i]  = 1U;
        }

        /* --- 2. 稳定电平发生变化 → 按下沿 / 松开沿 --- */
        if ((s_raw_cnt[i] >= KEY_DEBOUNCE_CNT) && (s_stable[i] != raw))
        {
            s_stable[i] = raw;

            if (raw != 0U)                       /* 按下沿 */
            {
                s_hold_ms[i]   = 0U;
                s_long_sent[i] = 0U;
                s_pressed_num  = (uint8_t)(i + 1U);      /* 兼容 Key_GetNum() */

                if (s_press_cnt[i] < 0xFFFFU)
                {
                    s_press_cnt[i]++;
                }

                if (s_map[i].ev_short != KEY_EVENT_NONE)
                {
                    Key_PushEvent(s_map[i].ev_short);    /* 短按事件在按下沿就出，菜单手感最好 */
                }
            }
            else                                 /* 松开沿 */
            {
                s_hold_ms[i]   = 0U;
                s_long_sent[i] = 0U;
            }
        }

        /* --- 3. 按住期间：长按事件 + 连发 --- */
        if (s_stable[i] != 0U)
        {
            if (s_hold_ms[i] < 0xFFFFU)
            {
                s_hold_ms[i]++;
            }

            /* 长按（只发一次） */
            if ((s_map[i].ev_long != KEY_EVENT_NONE) &&
                (s_long_sent[i] == 0U) &&
                (s_hold_ms[i] >= KEY_LONG_CNT))
            {
                Key_PushEvent(s_map[i].ev_long);
                s_long_sent[i] = 1U;
                if (s_long_cnt[i] < 0xFFFFU)
                {
                    s_long_cnt[i]++;
                }
            }

            /* 连发（上/下键）：按住 KEY_REPEAT_DELAY_MS 后，每 KEY_REPEAT_CNT 拍发一次 */
            if ((s_map[i].ev_repeat != KEY_EVENT_NONE) &&
                (s_hold_ms[i] >= KEY_REPEAT_DELAY_CNT) &&
                (((s_hold_ms[i] - KEY_REPEAT_DELAY_CNT) % KEY_REPEAT_CNT) == 0U))
            {
                Key_PushEvent(s_map[i].ev_repeat);
            }
        }
    }

    s_raw_mask = mask;
}

/* ---------------------------------------------------------------------------
 * 事件读取
 * ------------------------------------------------------------------------- */
Key_Event_t Key_PeekEvent(void)
{
    if (s_evt_tail == s_evt_head)
    {
        return KEY_EVENT_NONE;
    }
    return s_evt[s_evt_tail];
}

Key_Event_t Key_GetEvent(void)
{
    Key_Event_t e;

    if (s_evt_tail == s_evt_head)
    {
        return KEY_EVENT_NONE;
    }
    e = s_evt[s_evt_tail];
    s_evt_tail = (uint8_t)((s_evt_tail + 1U) % KEY_EVT_QUEUE_LEN);
    return e;
}

/* ---------------------------------------------------------------------------
 * 兼容旧接口
 * ------------------------------------------------------------------------- */
uint8_t Key_GetState(void)
{
    uint8_t i;

    for (i = 0U; i < KEY_NUM; i++)
    {
        if (s_stable[i] != 0U)
        {
            return (uint8_t)(i + 1U);
        }
    }
    return 0U;
}

uint8_t Key_GetNum(void)
{
    uint8_t n = s_pressed_num;

    s_pressed_num = 0U;
    return n;
}

/* ---------------------------------------------------------------------------
 * 自检 / 显示接口（只读，不消费事件）
 * ------------------------------------------------------------------------- */
uint8_t Key_GetRawMask(void)
{
    return s_raw_mask;
}

uint16_t Key_GetHoldMs(uint8_t idx)
{
    if (idx >= KEY_NUM)
    {
        return 0U;
    }
    return s_hold_ms[idx];
}

uint16_t Key_GetPressCount(uint8_t idx)
{
    if (idx >= KEY_NUM)
    {
        return 0U;
    }
    return s_press_cnt[idx];
}

uint16_t Key_GetLongCount(uint8_t idx)
{
    if (idx >= KEY_NUM)
    {
        return 0U;
    }
    return s_long_cnt[idx];
}

uint32_t Key_GetScanCount(void)
{
    return s_scan_cnt;
}

uint32_t Key_GetEventCount(void)
{
    return s_evt_cnt;
}

Key_Event_t Key_GetLastEvent(void)
{
    return s_last_evt;
}

/* 取走"最近一次导航事件"（UP/DOWN 及连发）；没有则返回 KEY_EVENT_NONE。
   与 Key_GetEvent() 互不干扰：翻页用这个，告警队列仍归 Key_PeekEvent/Key_GetEvent。 */
Key_Event_t Key_TakeNav(void)
{
    Key_Event_t e = s_nav_evt;

    s_nav_evt = KEY_EVENT_NONE;
    return e;
}

/* 事件短名一律 5 字符：显示时直接覆盖 5 个字符位，不会残留上一次的字 */
char *Key_EventName(Key_Event_t e)
{
    switch (e)
    {
        case KEY_EVENT_UP:            return "UP   ";
        case KEY_EVENT_DOWN:          return "DOWN ";
        case KEY_EVENT_OK:            return "OK   ";
        case KEY_EVENT_BACK:          return "BACK ";
        case KEY_EVENT_UP_REPEAT:     return "U-REP";
        case KEY_EVENT_DOWN_REPEAT:   return "D-REP";
        case KEY_EVENT_BACK_LONG:     return "B-LNG";
        default:                      return "-----";
    }
}
