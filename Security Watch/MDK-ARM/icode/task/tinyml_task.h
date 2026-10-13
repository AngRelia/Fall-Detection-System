#ifndef __TINYML_TASK_H__
#define __TINYML_TASK_H__

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "queue.h"
#include "ml_infer.h"
#include "ml_features.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * tinyml_task —— TinyML 跌倒检测推理任务
 *
 * 数据流：
 *   imu_recv_task 每解析出一帧就调用 TinyML_PushFrame()，帧写入长度为 ML_WIN+1 的
 *   环形缓冲（单帧 memcpy，约 1 µs）。本任务每 TINYML_TASK_PERIOD_MS 检查一次：
 *
 *       够 200 帧了吗 ──否──▶ 继续等
 *            │是
 *            ▼
 *       ① 阈值预筛：窗口内 |a| 峰值 > 2.5 g 或 |ω| 峰值 > 300 °/s ？
 *            │否 → 直接丢弃，**不进网络**（绝大多数日常动作在此被挡掉）
 *            │是
 *            ▼
 *       ② ML_ExtractFeaturesRing → ML_QuantizeFeatures → ML_Run → ML_Softmax
 *            │
 *            ▼
 *       ③ 判定 FALL 且置信度 ≥ 0.8 → 置事件，交 alarm_task 处理
 *
 * 两阶段判定的意义（论文里可单独成节）：
 *   实测（合成数据集，1200 个窗口）：日常动作只有 23.25% 的窗会触发预筛，
 *   也就是说网络只在这 23.25% 的窗上被调用，其余 76.75% 连特征都不提取。
 *   而在 2500 mg 门限下，跌倒的触发率是 100% —— 门限再往上抬到 4000 mg，
 *   跌倒触发率就掉到 97.25%，开始漏检。这就是 2.5 g 这个取值的依据。
 *
 * 资源占用（armcc 实测，Cortex-M3 -O1）：
 *   Flash：ml_infer 588 B + ml_features 1816 B + ml_weights 820 B
 *          + ml_selftest 1208 B（可关） ≈ 4.4 KB
 *          （另有 sqrtf/acosf/expf 数学库首次引入的开销，需看整工程 map）
 *   RAM  ：环形缓冲 (200+1)×6×2 = 2412 B + 本任务栈约 500 B
 *   CPU  ：预筛 0.1 ms / 次；命中后特征 1.2 ms + 推理 0.05 ms
 */

/* ---- 可调参数（全部集中在这里，方便答辩现场调） ---- */

/* 检查周期（ms）。50 ms = 每次推理最多滞后一个窗口的 1/40，响应足够快 */
#define TINYML_TASK_PERIOD_MS        50

/* 预筛门限：合加速度峰值（mg）。依据见文件头注释 */
#define TINYML_PREFILTER_ACC_MG      2500

/* 预筛门限：角速度模值峰值（0.1°/s）。300 °/s = 3000 */
#define TINYML_PREFILTER_GYRO_DPS    3000

/* 告警置信度门限（softmax 概率） */
#define TINYML_CONF_THRESHOLD        0.80f

/* 同一事件重复上报抑制（ms）。防止连续多个窗都判成跌倒而反复报警 */
#define TINYML_COOLDOWN_MS           5000

/* ---- 事件与统计 ---- */

/* 一次跌倒判定结果，供 alarm_task / ble_task 使用 */
typedef struct {
    uint8_t  cls;                        /* ML_CLASS_ADL / NEARFALL / FALL   */
    int8_t   logit[ML_OUT_DIM];          /* int8 logit，ADL/NEAR/FALL        */
    float    conf[ML_OUT_DIM];           /* 反量化 + softmax 后的概率        */
    int      margin;                     /* 最高与次高 logit 之差（int8 单位）*/
    int16_t  acc_mg[3];                  /* 触发窗最后一帧的加速度，供 BLE   */
    float    feat[ML_FEAT_DIM];          /* 29 维特征，存档与离线复算用      */
    uint32_t tick;                       /* 判定发生的系统 tick              */
} TinyML_Event_t;

/* 运行统计，供 OLED 显示与论文的实测数据表使用 */
typedef struct {
    uint32_t frames_pushed;              /* 累计推入的帧数（应 ≈ 100/s）     */
    uint32_t windows_evaluated;          /* 数据够长后评估过的窗口数          */
    uint32_t prefilter_triggers;         /* 预筛命中次数（= 真正跑网络的次数）*/
    uint32_t falls_detected;             /* 判定为跌倒的次数（去重前）        */
    uint8_t  selftest_ok;                /* ML_SelfTest 是否通过             */
    uint16_t selftest_class_ok;          /* 黄金向量类别一致条数              */
    uint16_t selftest_total;             /* 黄金向量总条数                    */
    uint16_t selftest_maxdiff;           /* int8 输出最大偏差（LSB）          */
} TinyML_Stats_t;

/* 任务句柄（由 _Task_Init 创建后赋值） */
extern TaskHandle_t TinyMLTaskHandle;

/* 创建任务并分配优先级与栈大小，由 freertos.c 的 MX_FREERTOS_Init() 调用。
   prio  : osPriority_t 枚举值
   stack : 栈深度，单位 word（1 word = 4 字节）。本任务建议 ≥ 384 */
void TinyML_Task_Init(osPriority_t prio, uint32_t stack);

/*
 * 推入一帧新数据。必须在**任务上下文**调用（不要在中断里直接调）。
 * frame 单位：acc = mg，gyro = 0.1 °/s，顺序 ax, ay, az, gx, gy, gz
 * 内部只做一次 12 字节 memcpy 与一次下标自增，耗时约 1 µs。
 */
void TinyML_PushFrame(const int16_t frame[ML_CH]);

/*
 * 取走一个待处理的跌倒事件（非阻塞）。
 * 返回 1 = 取到了事件并已填入 ev；返回 0 = 当前没有事件。
 * 典型调用者：alarm_task 的循环。
 */
uint8_t TinyML_TakeEvent(TinyML_Event_t *ev);

/* 读取 / 清零运行统计 */
void TinyML_GetStats(TinyML_Stats_t *st);
void TinyML_ResetStats(void);

#ifdef __cplusplus
}
#endif

#endif /* __TINYML_TASK_H__ */
