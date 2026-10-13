/**
  ******************************************************************************
  * @file    ml_weights.h
  * @brief   跌倒检测 int8 模型权重（由 tools/tinyml/train_quantize.py 自动生成）
  *
  * !!! 请勿手工修改本文件 —— 重新训练请改脚本后重跑 !!!
  *
  * 生成信息
  *   结构      : 29 -> 16(ReLU) -> 3(softmax)
  *   参数量    : 531
  *   权重占用  : 512 字节 int8
  *   float32 测试准确率 : 0.9767
  *   int8    测试准确率 : 0.9758   量化损失 0.0008
  *
  * 量化约定（与 ml_infer.c 严格对应）
  *   输入  z = (feature - mean) / std, 再 q = round(z / s_x) 钳位到 [-128,127]
  *   第1层 acc1 = b1q + W1q·xq
  *         hq   = clamp((acc1*MULT1 + 2^(SHIFT1-1)) >> SHIFT1, 0, 127)
  *   第2层 acc2 = b2q + W2q·hq
  *         outq = clamp((acc2*MULT2 + 2^(SHIFT2-1)) >> SHIFT2, -128, 127)
  *   logit_i = outq_i * S_Y2   （做 softmax 前先反量化）
  ******************************************************************************
  */
#ifndef __ML_WEIGHTS_H__
#define __ML_WEIGHTS_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ML_IN_DIM       29
#define ML_H1_DIM       16
#define ML_OUT_DIM      3

/* 类别编号：必须与 fallsim.py / tinyml_task.c 的三分类保持一致 */
#define ML_CLASS_ADL       0
#define ML_CLASS_NEARFALL  1
#define ML_CLASS_FALL      2

/* 输入反量化：z = (x - ML_FEAT_MEAN[i]) * ML_FEAT_INV_STD[i] */
extern const float ML_FEAT_MEAN[ML_IN_DIM];
extern const float ML_FEAT_INV_STD[ML_IN_DIM];

/* 量化 scale（反量化用） */
#define ML_S_X       4.221761873e-02f
#define ML_S_Y1      7.280051224e-02f
#define ML_S_Y2      1.042751358e-01f

/* 定点乘子 */
#define ML_MULT1     1091023977
#define ML_SHIFT1    38
#define ML_MULT2     1523899694
#define ML_SHIFT2    38

extern const int8_t  ML_W1[ML_H1_DIM * ML_IN_DIM];    /* 行主序，每行一个输出神经元 */
extern const int32_t ML_B1[ML_H1_DIM];
extern const int8_t  ML_W2[ML_OUT_DIM * ML_H1_DIM];
extern const int32_t ML_B2[ML_OUT_DIM];

#ifdef __cplusplus
}
#endif

#endif /* __ML_WEIGHTS_H__ */
