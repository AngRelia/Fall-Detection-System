/**
  ******************************************************************************
  * @file    ml_infer.h
  * @brief   跌倒检测 int8 定点推理内核（自写，不依赖 TensorFlow Lite Micro）
  *
  * 为什么不用 TFLite Micro：
  *   官方运行时（内核 + 解释器 + 张量管理）在 Cortex-M3 上就要 60~100 KB Flash，
  *   而 STM32F103C8T6 只有 64 KB，现有工程已占 40 KB，只剩约 24 KB。物理上装不下。
  *   本内核 + 权重合计约 2~3 KB，功能等价（对这个小网络而言）。
  *
  * 全部整数运算：int8 权重 × int8 激活 → int32 累加 → 定点乘子重缩放回 int8。
  * F103 没有 FPU，这是它唯一跑得动深度学习的方式。
  ******************************************************************************
  */
#ifndef __ML_INFER_H__
#define __ML_INFER_H__

#include <stdint.h>
#include "ml_weights.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  特征标准化 + 量化为 int8
  * @param  feat  [in]  ML_IN_DIM 个原始特征（ML_ExtractFeatures 的输出）
  * @param  out_q [out] ML_IN_DIM 个 int8 量化值
  * @note   z = (feat - mean) * inv_std ; q = clamp(floor(z/s_x + 0.5), -128, 127)
  */
void ML_QuantizeFeatures(const float feat[ML_IN_DIM], int8_t out_q[ML_IN_DIM]);

/**
  * @brief  跑一次完整推理
  * @param  in_q  [in]  ML_IN_DIM 个 int8 输入（ML_QuantizeFeatures 的输出）
  * @param  out_q [out] ML_OUT_DIM 个 int8 logit；三类共用同一 scale，
  *                     因此 argmax 可直接在 int8 上做，无需反量化
  * @note   纯整数运算，无堆分配，栈占用约 84 字节
  */
void ML_Run(const int8_t in_q[ML_IN_DIM], int8_t out_q[ML_OUT_DIM]);

/**
  * @brief  一步到位：原始特征 → 类别
  * @retval 0=ADL 1=NEAR_FALL 2=FALL
  */
int ML_Predict(const float feat[ML_IN_DIM]);

/**
  * @brief  logit 反量化 + softmax，得到三类的置信度
  * @param  out_q [in]  ML_Run 的输出
  * @param  prob  [out] ML_OUT_DIM 个概率，和为 1
  * @note   只有 3 个输出，直接调 expf 即可；整机最坏情况也就 3 次 expf。
  *         若要求"全定点"，可改用 (top1 - top2) 作为置信度代理，见 ML_Margin。
  */
void ML_Softmax(const int8_t out_q[ML_OUT_DIM], float prob[ML_OUT_DIM]);

/**
  * @brief  置信度代理：最高与次高 logit 的差值（int8 单位）
  * @retval 差值，越大越可信。纯整数，无浮点，适合放在中断或高频路径里。
  */
int ML_Margin(const int8_t out_q[ML_OUT_DIM]);

#ifdef __cplusplus
}
#endif

#endif /* __ML_INFER_H__ */
