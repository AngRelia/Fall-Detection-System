"""
features.py —— 29 维手工特征提取（Python 参考实现）

⚠️ 本文件的特征**顺序、名称、计算公式**必须与 C 端
   `Security Watch/MDK-ARM/icode/ml/ml_features.c` 严格一一对应。
   任何一处不一致都会让"PC 上验证通过、板子上结果不对"，是本项目最容易踩的坑。
   修改时请同时改两边，并重跑 tools/tinyml/verify_quant.py。

输入窗口约定：
    win : (200, 6)  —— 列序 ax, ay, az [g]，gx, gy, gz [°/s]
    C 端从帧协议拿到的是 mg 与 0.1°/s 的 int16，会先转成同样的物理量再调用。

为什么用 29 维特征 + 小 MLP，而不是直接把 200×6 的原窗喂给网络：
    1. 24 KB Flash 预算下，原窗 CNN 的激活值就吃掉几 KB，特征法把输入压到 29 字节；
    2. 手工特征（峰值、失重、冲击后静止、倾角变化）本身就是跌倒检测文献的核心判据，
       模型学到的是物理规律，跨受试者泛化更好；
    3. sklearn 就能训练，不需要 TensorFlow，也不需要自己手写反向传播。
    代价是丢失了波形细节 —— 论文里可作为"特征工程 vs 端到端 CNN"的对比实验。
"""

import numpy as np

FS = 100
WIN = 200
N_FEATURES = 29

# 尾部窗口（"冲击后"阶段）覆盖最后 0.5 s
TAIL0 = 150
# 头部窗口（"冲击前"阶段）覆盖最初 0.5 s
HEAD1 = 50

FEATURE_NAMES = (
    "am_max",        # 0  合加速度峰值 —— 冲击强度最直接的度量
    "am_mean",       # 1  合加速度均值
    "am_std",        # 2  合加速度标准差
    "am_min",        # 3  合加速度最小值 —— 自由落体（失重）判据
    "gm_max",        # 4  角速度模值峰值
    "gm_mean",       # 5  角速度模值均值
    "gm_std",        # 6  角速度模值标准差
    "sma",           # 7  信号幅值面积
    "az_std",        # 8  竖直轴标准差
    "ax_mean",       # 9  重力方向 x 分量
    "ay_mean",       # 10 重力方向 y 分量
    "az_mean",       # 11 重力方向 z 分量 —— 躺倒后显著偏离 1
    "ax_std",        # 12
    "ay_std",        # 13
    "ax_absmax",     # 14
    "ay_absmax",     # 15
    "az_absmax",     # 16
    "dam_max",       # 17 加加速度峰值（jerk）—— 冲击陡峭度
    "dam_mean",      # 18
    "tail_am_std",   # 19 冲击后 0.5 s 的合加速度方差 —— 跌倒后静止判据
    "tail_az_std",   # 20
    "tilt_deg",      # 21 首尾重力方向夹角 —— 姿态改变量
    "daz_max",       # 22
    "am_range",      # 23 合加速度动态范围
    "az_zcr",        # 24 竖直轴过零率 —— 活动强度
    "dmag_mean",     # 25 三轴差分均值 —— 高频能量
    "head_am_std",   # 26 冲击前 0.5 s 的方差 —— 冲击前活动水平
    "am_peak_ratio", # 27 峰值/均值比 —— 冲击相对强度
    "tail_gm_mean",  # 28 冲击后平均角速度 —— 跌倒后静止判据
)


def extract_features(win):
    """(200,6) → (29,) float64"""
    win = np.asarray(win, dtype=np.float64)
    ax, ay, az = win[:, 0], win[:, 1], win[:, 2]
    gx, gy, gz = win[:, 3], win[:, 4], win[:, 5]

    am = np.sqrt(ax * ax + ay * ay + az * az)
    gm = np.sqrt(gx * gx + gy * gy + gz * gz)

    axm, aym, azm = ax.mean(), ay.mean(), az.mean()
    f = np.empty(N_FEATURES, dtype=np.float64)

    f[0] = am.max()
    f[1] = am.mean()
    f[2] = am.std()
    f[3] = am.min()
    f[4] = gm.max()
    f[5] = gm.mean()
    f[6] = gm.std()
    f[7] = (np.abs(ax) + np.abs(ay) + np.abs(az)).mean()
    f[8] = az.std()
    f[9] = axm
    f[10] = aym
    f[11] = azm
    f[12] = ax.std()
    f[13] = ay.std()
    f[14] = np.abs(ax).max()
    f[15] = np.abs(ay).max()
    f[16] = np.abs(az).max()

    # 加加速度：一阶差分 × 采样率。用相邻差分近似，C 端同样处理。
    dam = np.abs(np.diff(am)) * FS
    daz = np.abs(np.diff(az)) * FS
    dmag = (np.abs(np.diff(ax)) + np.abs(np.diff(ay)) + np.abs(np.diff(az))) * FS
    f[17] = dam.max()
    f[18] = dam.mean()
    f[19] = am[TAIL0:].std()
    f[20] = az[TAIL0:].std()
    f[21] = _tilt_deg(win[:HEAD1], win[TAIL0:])
    f[22] = daz.max()
    f[23] = f[0] - f[3]
    f[24] = _zcr(az - azm)
    f[25] = dmag.mean()
    f[26] = am[:HEAD1].std()
    f[27] = f[0] / (f[1] + 1e-6)
    f[28] = gm[TAIL0:].mean()
    return f


def _tilt_deg(head, tail):
    """两段窗口平均重力方向的夹角（度）。acos 参数必须夹到 [-1,1] 防数值越界。"""
    u = head[:, 0:3].mean(axis=0)
    v = tail[:, 0:3].mean(axis=0)
    nu, nv = np.linalg.norm(u), np.linalg.norm(v)
    if nu < 1e-9 or nv < 1e-9:
        return 0.0
    c = float(np.clip(np.dot(u, v) / (nu * nv), -1.0, 1.0))
    return float(np.degrees(np.arccos(c)))


def _zcr(x):
    """过零率：把 x 当作零均值信号，统计符号变化次数占样本间隔的比例。"""
    s = np.signbit(x)
    return float(np.count_nonzero(s[1:] != s[:-1])) / float(len(x) - 1)


def extract_dataset(X):
    """(N,200,6) → (N,29)"""
    return np.stack([extract_features(w) for w in X]).astype(np.float64)


if __name__ == "__main__":
    import time
    from fallsim import make_dataset

    X, y = make_dataset(7, 40)
    t = time.time()
    F = extract_dataset(X)
    print("features", F.shape, "%.1f ms/window" % ((time.time() - t) / len(X) * 1000))
    for name in ("am_max", "am_min", "tail_am_std", "tilt_deg"):
        i = FEATURE_NAMES.index(name)
        print("  %-13s ADL=%8.3f  NEAR=%8.3f  FALL=%8.3f"
              % (name, F[y == 0, i].mean(), F[y == 1, i].mean(), F[y == 2, i].mean()))
