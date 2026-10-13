/**
  ******************************************************************************
  * @file    ml_selftest.c
  * @brief   模型上板自测实现
  ******************************************************************************
  */
#include "ml_selftest.h"

#if ML_ENABLE_SELFTEST

#include "ml_weights.h"
#include "ml_golden.h"      /* 由 tools/tinyml/train_quantize.py 生成 */
#include <math.h>

int ML_SelfTest(ML_SelfTestResult_t *res)
{
    uint16_t i;
    uint16_t exact = 0;
    uint16_t cls_ok = 0;
    uint16_t maxdiff = 0;

    if (res != 0) {
        res->total = ML_GOLDEN_N;
        res->exact_out = 0;
        res->class_ok = 0;
        res->max_lsb_diff = 0;
    }

    for (i = 0; i < ML_GOLDEN_N; i++) {
        int8_t in_q[ML_IN_DIM];
        int8_t out_q[ML_OUT_DIM];
        int    diff = 0;
        int    matched = 1;
        int    top = 0;
        int    k;

        /* 用与线上完全相同的入口：原始特征 → 量化 → 推理 */
        ML_QuantizeFeatures(ML_GOLDEN_FEAT[i], in_q);
        ML_Run(in_q, out_q);

        for (k = 0; k < ML_OUT_DIM; k++) {
            int d = (int)out_q[k] - (int)ML_GOLDEN_OUT[i][k];
            if (d != 0) {
                matched = 0;
            }
            if (d < 0) d = -d;
            if (d > diff) diff = d;
        }
        if (matched) {
            exact++;
        }
        if (diff > (int)maxdiff) {
            maxdiff = (uint16_t)diff;
        }

        /* 类别判定必须与 PC 一致 —— 这一条比 int8 逐位一致更重要 */
        if (out_q[1] > out_q[0]) {
            top = (out_q[2] > out_q[1]) ? 2 : 1;
        } else {
            top = (out_q[2] > out_q[0]) ? 2 : 0;
        }
        if (top == (int)ML_GOLDEN_CLS[i]) {
            cls_ok++;
        }
    }

    if (res != 0) {
        res->exact_out = exact;
        res->class_ok = cls_ok;
        res->max_lsb_diff = maxdiff;
    }

    /* 判据：类别必须全对；int8 输出允许极少数 1 LSB 偏差
       （C 用 float、Python 用 double，浮点舍入在量化边界上可能差 1 个 LSB，
         这属于正常现象；若出现 >=2 LSB 或类别不一致，就是真 bug）。 */
    return (cls_ok == ML_GOLDEN_N && maxdiff <= 1) ? 1 : 0;
}

#else  /* ML_ENABLE_SELFTEST */

/* 关闭自测时提供空实现，调用方不必加条件编译（黄金向量与代码都不进 Flash） */
int ML_SelfTest(ML_SelfTestResult_t *res)
{
    if (res != 0) {
        res->total = 0;
        res->exact_out = 0;
        res->class_ok = 0;
        res->max_lsb_diff = 0;
    }
    return 1;
}

#endif /* ML_ENABLE_SELFTEST */
