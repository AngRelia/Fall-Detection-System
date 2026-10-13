/**
  ******************************************************************************
  * @file    ml_infer.c
  * @brief   跌倒检测 int8 定点推理内核
  *
  * 数值语义与 tools/tinyml/train_quantize.py 中的 infer_int8() **逐位对应**：
  *
  *     第一层  acc1 = B1[o] + Σ W1[o][i] · xq[i]                  （int32 累加）
  *             hq   = clamp( (acc1·MULT1 + 2^(SHIFT1-1)) >> SHIFT1, 0, 127 )   ← ReLU
  *     第二层  acc2 = B2[o] + Σ W2[o][j] · hq[j]
  *             outq = clamp( (acc2·MULT2 + 2^(SHIFT2-1)) >> SHIFT2, -128, 127 )
  *
  * 两个易错点，改动时务必注意：
  *   1. 乘法必须提升到 64 位。acc 可达 ±4.7e5，MULT 约 1.5e9，32 位必溢出。
  *   2. 移位前先加 2^(shift-1) 做四舍五入。C 的 >> 对负数是算术右移（向下取整），
  *      与 Python 的 >> 语义一致，因此训练端模拟出的结果可以直接比对。
  ******************************************************************************
  */
#include "ml_infer.h"
#include <math.h>

/* ---------------------------------------------------------------------------
 * 特征标准化 + 量化
 * ------------------------------------------------------------------------- */
void ML_QuantizeFeatures(const float feat[ML_IN_DIM], int8_t out_q[ML_IN_DIM])
{
    const float inv_sx = 1.0f / ML_S_X;
    int i;

    for (i = 0; i < ML_IN_DIM; i++) {
        float z = (feat[i] - ML_FEAT_MEAN[i]) * ML_FEAT_INV_STD[i];
        float v = floorf(z * inv_sx + 0.5f);

        if (v > 127.0f)  v = 127.0f;
        if (v < -128.0f) v = -128.0f;
        out_q[i] = (int8_t)v;
    }
}

/* ---------------------------------------------------------------------------
 * 推理
 * ------------------------------------------------------------------------- */
void ML_Run(const int8_t in_q[ML_IN_DIM], int8_t out_q[ML_OUT_DIM])
{
    int8_t  hid[ML_H1_DIM];
    int32_t acc;
    int64_t wide;
    int o, i;

    /* ---- 第 1 层：全连接 + ReLU ---- */
    for (o = 0; o < ML_H1_DIM; o++) {
        const int8_t *row = &ML_W1[o * ML_IN_DIM];

        acc = ML_B1[o];
        for (i = 0; i < ML_IN_DIM; i++) {
            acc += (int32_t)row[i] * (int32_t)in_q[i];
        }

        wide = ((int64_t)acc * (int64_t)ML_MULT1
                + ((int64_t)1 << (ML_SHIFT1 - 1))) >> ML_SHIFT1;
        if (wide < 0)        wide = 0;          /* ReLU：量化域 0 即实数域 0 */
        else if (wide > 127) wide = 127;
        hid[o] = (int8_t)wide;
    }

    /* ---- 第 2 层：全连接（输出 logit，分类层不带激活） ---- */
    for (o = 0; o < ML_OUT_DIM; o++) {
        const int8_t *row = &ML_W2[o * ML_H1_DIM];

        acc = ML_B2[o];
        for (i = 0; i < ML_H1_DIM; i++) {
            acc += (int32_t)row[i] * (int32_t)hid[i];
        }

        wide = ((int64_t)acc * (int64_t)ML_MULT2
                + ((int64_t)1 << (ML_SHIFT2 - 1))) >> ML_SHIFT2;
        if (wide > 127)       wide = 127;
        else if (wide < -128) wide = -128;
        out_q[o] = (int8_t)wide;
    }
}

/* ---------------------------------------------------------------------------
 * 便捷入口
 * ------------------------------------------------------------------------- */
int ML_Predict(const float feat[ML_IN_DIM])
{
    int8_t in_q[ML_IN_DIM];
    int8_t out_q[ML_OUT_DIM];

    ML_QuantizeFeatures(feat, in_q);
    ML_Run(in_q, out_q);

    /* 三个 logit 共用同一 scale，argmax 在 int8 域上等价于在实数域上 */
    if (out_q[ML_CLASS_FALL] > out_q[ML_CLASS_NEARFALL]) {
        return (out_q[ML_CLASS_FALL] > out_q[ML_CLASS_ADL])
               ? ML_CLASS_FALL : ML_CLASS_ADL;
    }
    return (out_q[ML_CLASS_NEARFALL] > out_q[ML_CLASS_ADL])
           ? ML_CLASS_NEARFALL : ML_CLASS_ADL;
}

void ML_Softmax(const int8_t out_q[ML_OUT_DIM], float prob[ML_OUT_DIM])
{
    float l[ML_OUT_DIM];
    float mx, sum;
    int i;

    mx = (float)out_q[0];
    for (i = 1; i < ML_OUT_DIM; i++) {
        if ((float)out_q[i] > mx) mx = (float)out_q[i];
    }
    sum = 0.0f;
    for (i = 0; i < ML_OUT_DIM; i++) {
        /* 先反量化到实数域再 softmax：softmax 对缩放敏感，不能直接用 int8 值 */
        l[i] = expf(((float)out_q[i] - mx) * ML_S_Y2);
        sum += l[i];
    }
    if (sum <= 0.0f) {
        for (i = 0; i < ML_OUT_DIM; i++) prob[i] = 1.0f / (float)ML_OUT_DIM;
        return;
    }
    for (i = 0; i < ML_OUT_DIM; i++) {
        prob[i] = l[i] / sum;
    }
}

int ML_Margin(const int8_t out_q[ML_OUT_DIM])
{
    int top1 = -129, top2 = -129;
    int i;

    for (i = 0; i < ML_OUT_DIM; i++) {
        int v = (int)out_q[i];
        if (v > top1) {
            top2 = top1;
            top1 = v;
        } else if (v > top2) {
            top2 = v;
        }
    }
    return top1 - top2;
}
