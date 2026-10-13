/**
  ******************************************************************************
  * @file    ml_selftest.h
  * @brief   模型上板自测：用 PC 端算好的黄金向量验证 MCU 端算得对不对
  *
  * 为什么需要它：
  *   模型"PC 上 98%、板子上乱跳"是本项目最常见的故障，而原因通常只有一个 ——
  *   C 端与 Python 端的量化/归一化语义有细微不一致（比如移位前忘了加四舍五入、
  *   argmax 顺序错了、特征顺序错位）。这类错误不会报编译错误，也不会崩，
  *   只会让结果悄悄变差，极难排查。
  *   本自测把 PC 端算好的 (原始特征 → int8 输出 → 类别) 烧进 Flash，上电跑一遍：
  *     全部一致 → 归一化、量化、推理链路与 PC 完全等价，可以放心用；
  *     有不一致 → 立刻定位到 ml_infer.c，不用去猜特征提取。
  *
  * 关闭方式：不用时把 ML_ENABLE_SELFTEST 改为 0（可省下约 1.1 KB Flash）。
  ******************************************************************************
  */
#ifndef __ML_SELFTEST_H__
#define __ML_SELFTEST_H__

#include <stdint.h>
#include "ml_infer.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ML_ENABLE_SELFTEST   1

typedef struct {
    uint16_t total;        /* 黄金向量总数            */
    uint16_t exact_out;    /* int8 输出逐位一致的数量  */
    uint16_t class_ok;     /* 类别一致的数量          */
    uint16_t max_lsb_diff; /* 单点最大 LSB 偏差        */
} ML_SelfTestResult_t;

/**
  * @brief  跑一遍黄金向量自测
  * @param  res [out] 统计结果，可为 NULL
  * @retval 1 = 全部一致（可放心使用）；0 = 存在不一致
  * @note   约 8 次推理，耗时 < 1 ms。建议在 tinyml_task 初始化时调用一次，
  *         结果打到 OLED 或串口。
  */
int ML_SelfTest(ML_SelfTestResult_t *res);

#ifdef __cplusplus
}
#endif

#endif /* __ML_SELFTEST_H__ */
