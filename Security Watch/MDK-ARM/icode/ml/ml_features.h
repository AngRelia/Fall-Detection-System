/**
  ******************************************************************************
  * @file    ml_features.h
  * @brief   跌倒检测 29 维特征提取（C 端，与 tools/tinyml/features.py 严格一致）
  *
  * ⚠️ 本文件与 features.py 是**同一套算法的两份实现**。任何一边改动都必须同步
  *    另一边，并重跑 tools/tinyml/verify_quant.py。顺序错一位，板子上的结果就会
  *    全错，而且不会报任何错误 —— 这是本项目最容易踩的坑。
  *
  * 输入单位约定：直接吃节点间帧协议的原始整数
  *    acc  : mg      （±16 g 量程，1 g = 1000 mg）
  *    gyro : 0.1 °/s （±2000 °/s 量程，1 °/s = 10 LSB）
  *
  * 内存与算力：全程 O(1) 额外内存（两遍扫描，不缓存整窗），
  *             约 2×200×(6 乘 + 2 平方根) ≈ 1.2 ms @72 MHz（无 FPU，实测为准）。
  ******************************************************************************
  */
#ifndef __ML_FEATURES_H__
#define __ML_FEATURES_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ML_WIN          200     /* 窗口长度 = 2 s @ 100 Hz */
#define ML_CH           6       /* ax, ay, az, gx, gy, gz */
#define ML_FEAT_DIM     29

/* 特征名（顺序即数组下标，调试与文档用，不占 Flash） */
#define ML_FEAT_NAMES                                                       \
    "am_max", "am_mean", "am_std", "am_min", "gm_max", "gm_mean", "gm_std",\
    "sma", "az_std", "ax_mean", "ay_mean", "az_mean", "ax_std", "ay_std",   \
    "ax_absmax", "ay_absmax", "az_absmax", "dam_max", "dam_mean",           \
    "tail_am_std", "tail_az_std", "tilt_deg", "daz_max", "am_range",        \
    "az_zcr", "dmag_mean", "head_am_std", "am_peak_ratio", "tail_gm_mean"

/**
  * @brief  从 2 s 窗口提取 29 维特征
  * @param  win   [in]  ML_WIN × ML_CH 原始整数，单位 mg 与 0.1°/s
  * @param  feat  [out] ML_FEAT_DIM 个物理量（g、°/s、度、无量纲），未标准化
  * @retval 无
  * @note   本入口与 features.py 的 extract_features() 逐行对应，便于交叉验证。
  */
void ML_ExtractFeatures(const int16_t win[ML_WIN][ML_CH], float feat[ML_FEAT_DIM]);

/**
  * @brief  同上，但从**环形缓冲区**直接取窗，不复制、不占额外 RAM
  * @param  ring     [in]  行主序连续存放的环形缓冲
  * @param  ring_len [in]  环的长度（帧数）—— 必须 **大于** ML_WIN，见下方说明
  * @param  start    [in]  最旧一帧的行下标（0 ~ ring_len-1），即"从哪一帧开始算 2 s"
  * @param  feat     [out] 29 维特征
  * @retval 无
  *
  * 为什么需要这个入口（这是个真实的 RAM 权衡）：
  *   200×6 的 int16 窗口 = 2400 字节。
  *   · 放到任务栈上 → 直接爆掉 1 KB 的栈；
  *   · 先快照到临时数组再算 → 再多占 2400 字节，整机 RAM 会很紧；
  *   · 环形缓冲 + 起址索引 → 只占 (ML_WIN+1)×12 字节，且**读取无需临界区**。
  *
  * 环长度为什么要 +1：
  *   生产者下一帧要写的槽位，正好是消费者 start 指向的那一帧。若环长恰为 ML_WIN，
  *   消费者读第一帧时生产者可能正在覆盖它。多留 1 个槽位后，生产者在写完一帧后
  *   至少要再过 (ring_len-1)×10 ms ≈ 2 s 才会碰回消费者正在读的区域，
  *   而一次特征提取只要约 1.2 ms —— 时间上不可能冲突，因此无需关中断。
  */
void ML_ExtractFeaturesRing(const int16_t *ring, uint16_t ring_len,
                            uint16_t start, float feat[ML_FEAT_DIM]);

#ifdef __cplusplus
}
#endif

#endif /* __ML_FEATURES_H__ */
