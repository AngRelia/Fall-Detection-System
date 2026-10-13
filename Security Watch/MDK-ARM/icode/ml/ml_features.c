/**
  ******************************************************************************
  * @file    ml_features.c
  * @brief   跌倒检测 29 维特征提取（C 端实现）
  *
  * 实现要点
  *   · 两遍扫描，O(1) 额外内存 —— 不缓存整窗。F103 只有 20 KB RAM，
  *     若把 200×6 的浮点中间量缓存下来要 4.8 KB，会直接吃垮任务栈。
  *   · 第二遍重算 ax..gz / am / gm（平方根是主要开销，约 1.2 ms @72 MHz）。
  *     这是"省 RAM 换 CPU"，符合本系统推理低频（预筛命中后才跑）的特点。
  *   · 同一份核心代码服务两个入口：线性数组与环形缓冲。区别仅在取帧方式，
  *     见 ML_FrameAt()。这样 features.py 的对应关系只需维护一处。
  *   · 全部使用 float（单精度）以贴近 MCU 实际；与 Python 的 float64 存在
  *     ~1e-6 量级的差异，其影响已由 tools/tinyml/verify_quant.py 量化评估
  *     （结论：1200 个测试窗中类别改变 0 个、量化值差异 0 LSB）。
  ******************************************************************************
  */
#include "ml_features.h"
#include <math.h>

/* 原始整数 → 物理量 */
#define A_LSB     0.001f        /* 1 mg   → g    */
#define G_LSB     0.1f          /* 0.1°/s → °/s  */
#define FS        100.0f        /* 采样率 Hz      */
#define RAD2DEG   57.29577951308232f
#define EPS       1.0e-6f

#define N_HEAD    50            /* 前 0.5 s：冲击前 */
#define N_TAIL    50            /* 后 0.5 s：冲击后 */
#define I_TAIL    150           /* TAIL 起始下标    */

/* 首尾两段平均重力方向的夹角（度），定义见文件末尾 */
static float ML_TiltDeg(float hx, float hy, float hz,
                        float tx, float ty, float tz);

/**
  * @brief 取窗口内第 i 帧（i = 0 表示最旧的一帧）
  * @note  线性数组时 start 恒为 0 且 ring_len = ML_WIN，这里不会回绕。
  */
static const int16_t *ML_FrameAt(const int16_t *base, int ring_len,
                                 int start, int i)
{
    int k = start + i;

    if (k >= ring_len) {
        k -= ring_len;
    }
    return base + (k * ML_CH);
}

/* ---------------------------------------------------------------------------
 * 核心：29 维特征提取
 * ------------------------------------------------------------------------- */
static void ML_ExtractCore(const int16_t *base, int ring_len, int start,
                           float feat[ML_FEAT_DIM])
{
    /* ---- 第一遍：各类均值、极值、头尾姿态 ---- */
    float sum_am = 0.0f, sum_gm = 0.0f;
    float sum_ax = 0.0f, sum_ay = 0.0f, sum_az = 0.0f;
    float sum_sma = 0.0f;
    float am_max = -1.0e30f, am_min = 1.0e30f, gm_max = 0.0f;
    float ax_absmax = 0.0f, ay_absmax = 0.0f, az_absmax = 0.0f;
    float h_ax = 0.0f, h_ay = 0.0f, h_az = 0.0f, sum_head_am = 0.0f;
    float t_ax = 0.0f, t_ay = 0.0f, t_az = 0.0f;
    float sum_tail_am = 0.0f, sum_tail_az = 0.0f, sum_tail_gm = 0.0f;

    /* ---- 第二遍：方差、差分（jerk）、过零率 ---- */
    float sum_am_var = 0.0f, sum_gm_var = 0.0f;
    float sum_ax_var = 0.0f, sum_ay_var = 0.0f, sum_az_var = 0.0f;
    float sum_head_am_var = 0.0f;
    float sum_tail_am_var = 0.0f, sum_tail_az_var = 0.0f;
    float dam_max = 0.0f, sum_dam = 0.0f, daz_max = 0.0f, sum_dmag = 0.0f;
    int   zc = 0;

    float am_mean, gm_mean, ax_mean, ay_mean, az_mean;
    float head_am_mean, tail_am_mean, tail_az_mean;
    float p_ax, p_ay, p_az, p_am;
    int   prev_neg;
    int   i;

    for (i = 0; i < ML_WIN; i++) {
        const int16_t *s = ML_FrameAt(base, ring_len, start, i);
        float ax = (float)s[0] * A_LSB;
        float ay = (float)s[1] * A_LSB;
        float az = (float)s[2] * A_LSB;
        float gx = (float)s[3] * G_LSB;
        float gy = (float)s[4] * G_LSB;
        float gz = (float)s[5] * G_LSB;
        float am = sqrtf(ax * ax + ay * ay + az * az);
        float gm = sqrtf(gx * gx + gy * gy + gz * gz);
        float axa = fabsf(ax), aya = fabsf(ay), aza = fabsf(az);

        sum_am += am;
        sum_gm += gm;
        sum_ax += ax;
        sum_ay += ay;
        sum_az += az;
        sum_sma += axa + aya + aza;

        if (am > am_max) am_max = am;
        if (am < am_min) am_min = am;
        if (gm > gm_max) gm_max = gm;
        if (axa > ax_absmax) ax_absmax = axa;
        if (aya > ay_absmax) ay_absmax = aya;
        if (aza > az_absmax) az_absmax = aza;

        if (i < N_HEAD) {
            h_ax += ax;
            h_ay += ay;
            h_az += az;
            sum_head_am += am;
        }
        if (i >= I_TAIL) {
            t_ax += ax;
            t_ay += ay;
            t_az += az;
            sum_tail_am += am;
            sum_tail_az += az;
            sum_tail_gm += gm;
        }
    }

    am_mean = sum_am / (float)ML_WIN;
    gm_mean = sum_gm / (float)ML_WIN;
    ax_mean = sum_ax / (float)ML_WIN;
    ay_mean = sum_ay / (float)ML_WIN;
    az_mean = sum_az / (float)ML_WIN;
    head_am_mean = sum_head_am / (float)N_HEAD;
    tail_am_mean = sum_tail_am / (float)N_TAIL;
    tail_az_mean = sum_tail_az / (float)N_TAIL;

    p_ax = p_ay = p_az = p_am = 0.0f;
    prev_neg = 0;

    for (i = 0; i < ML_WIN; i++) {
        const int16_t *s = ML_FrameAt(base, ring_len, start, i);
        float ax = (float)s[0] * A_LSB;
        float ay = (float)s[1] * A_LSB;
        float az = (float)s[2] * A_LSB;
        float gx = (float)s[3] * G_LSB;
        float gy = (float)s[4] * G_LSB;
        float gz = (float)s[5] * G_LSB;
        float am = sqrtf(ax * ax + ay * ay + az * az);
        float gm = sqrtf(gx * gx + gy * gy + gz * gz);
        float d;

        d = am - am_mean;      sum_am_var += d * d;
        d = gm - gm_mean;      sum_gm_var += d * d;
        d = ax - ax_mean;      sum_ax_var += d * d;
        d = ay - ay_mean;      sum_ay_var += d * d;
        d = az - az_mean;      sum_az_var += d * d;

        if (i < N_HEAD) {
            d = am - head_am_mean;
            sum_head_am_var += d * d;
        }
        if (i >= I_TAIL) {
            d = am - tail_am_mean;
            sum_tail_am_var += d * d;
            d = az - tail_az_mean;
            sum_tail_az_var += d * d;
        }

        if (i > 0) {
            /* 差分特征：与 numpy 的 np.diff 一致，共 ML_WIN-1 个样本 */
            d = fabsf(am - p_am) * FS;
            if (d > dam_max) dam_max = d;
            sum_dam += d;

            d = fabsf(az - p_az) * FS;
            if (d > daz_max) daz_max = d;

            sum_dmag += (fabsf(ax - p_ax) + fabsf(ay - p_ay)
                         + fabsf(az - p_az)) * FS;

            /* 过零率：用 signbit 而非 (x<0)，以与 numpy.signbit 完全一致
               （两者对 -0.0 的处理不同） */
            {
                int neg = signbit(az - az_mean) ? 1 : 0;
                if (neg != prev_neg) zc++;
                prev_neg = neg;
            }
        } else {
            prev_neg = signbit(az - az_mean) ? 1 : 0;
        }

        p_ax = ax; p_ay = ay; p_az = az; p_am = am;
    }

    /* ---- 组装特征向量：顺序必须与 features.py 的 FEATURE_NAMES 一致 ---- */
    feat[0]  = am_max;
    feat[1]  = am_mean;
    feat[2]  = sqrtf(sum_am_var / (float)ML_WIN);
    feat[3]  = am_min;
    feat[4]  = gm_max;
    feat[5]  = gm_mean;
    feat[6]  = sqrtf(sum_gm_var / (float)ML_WIN);
    feat[7]  = sum_sma / (float)ML_WIN;
    feat[8]  = sqrtf(sum_az_var / (float)ML_WIN);
    feat[9]  = ax_mean;
    feat[10] = ay_mean;
    feat[11] = az_mean;
    feat[12] = sqrtf(sum_ax_var / (float)ML_WIN);
    feat[13] = sqrtf(sum_ay_var / (float)ML_WIN);
    feat[14] = ax_absmax;
    feat[15] = ay_absmax;
    feat[16] = az_absmax;
    feat[17] = dam_max;
    feat[18] = sum_dam / (float)(ML_WIN - 1);
    feat[19] = sqrtf(sum_tail_am_var / (float)N_TAIL);
    feat[20] = sqrtf(sum_tail_az_var / (float)N_TAIL);
    feat[21] = ML_TiltDeg(h_ax, h_ay, h_az, t_ax, t_ay, t_az);
    feat[22] = daz_max;
    feat[23] = am_max - am_min;
    feat[24] = (float)zc / (float)(ML_WIN - 1);
    feat[25] = sum_dmag / (float)(ML_WIN - 1);
    feat[26] = sqrtf(sum_head_am_var / (float)N_HEAD);
    feat[27] = am_max / (am_mean + EPS);
    feat[28] = sum_tail_gm / (float)N_TAIL;
}

/* ---------------------------------------------------------------------------
 * 两个对外入口
 * ------------------------------------------------------------------------- */
void ML_ExtractFeatures(const int16_t win[ML_WIN][ML_CH], float feat[ML_FEAT_DIM])
{
    ML_ExtractCore(&win[0][0], ML_WIN, 0, feat);
}

void ML_ExtractFeaturesRing(const int16_t *ring, uint16_t ring_len,
                            uint16_t start, float feat[ML_FEAT_DIM])
{
    ML_ExtractCore(ring, (int)ring_len, (int)start, feat);
}

/**
  * @brief 首尾两段平均重力方向的夹角（度）
  *        对应 features.py 的 _tilt_deg：acos 参数必须夹到 [-1,1]，否则
  *        浮点误差导致的 1.0000001 会让 acosf 返回 NaN，进而污染整个特征向量。
  */
static float ML_TiltDeg(float hx, float hy, float hz, float tx, float ty, float tz)
{
    float nu = sqrtf(hx * hx + hy * hy + hz * hz);
    float nv = sqrtf(tx * tx + ty * ty + tz * tz);
    float c;

    if (nu < 1.0e-9f || nv < 1.0e-9f) {
        return 0.0f;
    }
    c = (hx * tx + hy * ty + hz * tz) / (nu * nv);
    if (c > 1.0f)  c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    return acosf(c) * RAD2DEG;
}
