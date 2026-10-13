"""
fallsim.py —— 跌倒 / 近似跌倒 / 日常动作的合成数据生成器

设计动机（写论文时可直接用这段话）：
    真实的跌倒标注数据稀缺且涉及伦理（让老年人真摔不现实）。本模块用**参数化生物
力学模型**生成 6 轴 IMU 波形，把跌倒过程拆成文献中公认的三个阶段：

        自由落体(free-fall)  →  冲击(impact)  →  冲击后静止(post-impact)

    每个阶段都有明确的物理量约束：
      · 自由落体期：合加速度模值下探到 0.2~0.5 g（失重）
      · 冲击期    ：合加速度尖峰 3.5~9 g，角速度尖峰 150~600 °/s
      · 冲击后    ：重力方向相对起始朝向偏转 70~100°（躺倒），且运动幅度骤降

    这三条正是跌倒检测文献里区分"跌倒"与"日常动作"的核心判据，因此用它生成的
    数据训练出来的模型，学到的是**物理规律**而不是噪声拟合。

坐标系约定（与节点间帧协议一致）：
    acc  : 单位 g，三轴 ax/ay/az
    gyro : 单位 °/s，三轴 gx/gy/gz
    起始姿态：设备竖直佩戴，重力方向在机体系为 (0, 0, +1)，即 az = +1 g

类别编号（务必与 C 端 ml_infer.h 的 ML_CLASS_* 保持一致）：
    0 = ADL        日常动作
    1 = NEAR_FALL  近似跌倒
    2 = FALL       跌倒
"""

import numpy as np

FS = 100          # 采样率 Hz
WIN = 200         # 窗口长度 = 2 s @ 100 Hz
N_CH = 6          # ax, ay, az, gx, gy, gz

ADL, NEAR_FALL, FALL = 0, 1, 2
CLASS_NAMES = ("ADL", "NEAR_FALL", "FALL")


# ---------------------------------------------------------------------------
# 工具函数
# ---------------------------------------------------------------------------
def _smooth(x, k):
    """
    长度 k 的滑动平均，用于把折线状的分段函数磨成物理上合理的连续波形。
    支持 1D 与 (n, m) 两种输入（多通道时逐列平滑）。
    """
    if k <= 1:
        return x
    x = np.asarray(x, dtype=np.float64)
    ker = np.ones(k, dtype=np.float64) / float(k)
    if x.ndim == 1:
        return np.convolve(x, ker, mode="same")
    out = np.empty_like(x)
    for c in range(x.shape[1]):
        out[:, c] = np.convolve(x[:, c], ker, mode="same")
    return out


def _ramp(n, a, b):
    """从 a 线性过渡到 b 的 n 点斜坡。"""
    if n <= 1:
        return np.array([b], dtype=np.float64)
    return np.linspace(a, b, n)


def _rot_between(u, v, n):
    """
    把单位向量 u 沿测地线（slerp）平滑旋转到单位向量 v，返回 (n, 3) 轨迹。
    这是"设备姿态怎么翻过去的"最简模型，同时给出角速度的解析来源。

    用 Rodrigues 公式的化简形式：因为旋转轴 k ⊥ u，
        R(θ)·u = u·cosθ + (k×u)·sinθ
    于是整条轨迹可以一次性向量化算出，比逐点旋转快两个数量级。
    """
    u = np.asarray(u, dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)
    u = u / np.linalg.norm(u)
    v = v / np.linalg.norm(v)
    dot = float(np.clip(np.dot(u, v), -1.0, 1.0))
    theta = np.arccos(dot)
    if theta < 1e-6:
        return np.tile(u, (n, 1))
    k = np.cross(u, v)
    nk = np.linalg.norm(k)
    if nk < 1e-9:
        # 反向：任取一条与 u 垂直的轴
        tmp = np.array([1.0, 0.0, 0.0])
        if abs(np.dot(tmp, u)) > 0.9:
            tmp = np.array([0.0, 1.0, 0.0])
        k = np.cross(u, tmp)
        nk = np.linalg.norm(k)
    k = k / nk
    a = np.linspace(0.0, theta, n)
    ku = np.cross(k, u)
    out = np.outer(np.cos(a), u) + np.outer(np.sin(a), ku)
    out /= np.linalg.norm(out, axis=1, keepdims=True)
    return out


def _gyro_from_dirs(dirs, fs=FS):
    """
    由姿态轨迹反解机体系角速度（°/s）。
    用相邻两帧的相对旋转 / 时间步长近似，足够刻画"角速度尖峰"这一特征。
    """
    n = len(dirs)
    g = np.zeros((n, 3), dtype=np.float64)
    if n < 2:
        return g
    u, v = dirs[:-1], dirs[1:]
    c = np.clip(np.einsum("ij,ij->i", u, v), -1.0, 1.0)
    ang = np.arccos(c)                                  # rad
    axis = np.cross(u, v)
    na = np.linalg.norm(axis, axis=1, keepdims=True)
    good = (na > 1e-9).ravel()
    axis = axis / np.where(na > 1e-9, na, 1.0)
    g[1:] = axis * np.degrees(ang)[:, None] * fs
    g[1:][~good] = 0.0
    g[0] = g[1]
    return _smooth(g, 3)


def _add_noise(rng, sig, acc_sigma, gyro_sigma):
    out = sig.copy()
    out[:, 0:3] += rng.normal(0.0, acc_sigma, (len(sig), 3))
    out[:, 3:6] += rng.normal(0.0, gyro_sigma, (len(sig), 3))
    return out


# ---------------------------------------------------------------------------
# 日常动作 ADL
# ---------------------------------------------------------------------------
def _adl_walk(rng, scale):
    """步行：竖直轴 1.6~2.2 Hz 周期振荡，角速度中小幅摆动。"""
    t = np.arange(WIN) / FS
    f = rng.uniform(1.6, 2.2)
    amp = rng.uniform(0.20, 0.45) * scale
    sig = np.zeros((WIN, 6))
    sig[:, 2] = 1.0 + amp * np.sin(2 * np.pi * f * t)
    sig[:, 0] = rng.uniform(0.08, 0.22) * np.sin(2 * np.pi * f * t + rng.uniform(0, 6))
    sig[:, 1] = rng.uniform(0.05, 0.15) * np.sin(2 * np.pi * (f / 2) * t + rng.uniform(0, 6))
    sig[:, 5] = rng.uniform(20, 60) * np.sin(2 * np.pi * f * t)
    sig[:, 3] = rng.uniform(10, 40) * np.sin(2 * np.pi * (f / 2) * t)
    sig[:, 4] = rng.uniform(30, 90) * np.sin(2 * np.pi * f * t + 1.0)
    return sig


def _adl_run(rng, scale):
    """小跑：幅度与角速度都明显大于步行，是最容易被误判成跌倒的一类负样本。"""
    t = np.arange(WIN) / FS
    f = rng.uniform(2.3, 2.9)
    amp = rng.uniform(0.5, 0.9) * scale
    sig = np.zeros((WIN, 6))
    sig[:, 2] = 1.0 + amp * np.sin(2 * np.pi * f * t)
    sig[:, 0] = rng.uniform(0.15, 0.35) * np.sin(2 * np.pi * f * t + rng.uniform(0, 6))
    sig[:, 1] = rng.uniform(0.10, 0.25) * np.sin(2 * np.pi * (f / 2) * t + rng.uniform(0, 6))
    sig[:, 5] = rng.uniform(60, 140) * np.sin(2 * np.pi * f * t)
    sig[:, 3] = rng.uniform(30, 90) * np.sin(2 * np.pi * (f / 2) * t)
    sig[:, 4] = rng.uniform(80, 180) * np.sin(2 * np.pi * f * t + 1.0)
    return sig


def _adl_static(rng, scale):
    """静坐 / 静立：只有呼吸和微小晃动，合加速度恒为 1 g。"""
    sig = np.zeros((WIN, 6))
    t = np.arange(WIN) / FS
    sig[:, 2] = 1.0 + 0.010 * np.sin(2 * np.pi * 0.25 * t)
    sig[:, 0] = 0.008 * np.sin(2 * np.pi * 0.3 * t + rng.uniform(0, 6))
    sig[:, 1] = 0.006 * np.sin(2 * np.pi * 0.2 * t + rng.uniform(0, 6))
    sig[:, 3:6] = rng.normal(0.0, 1.5, (WIN, 3))
    return sig


def _adl_orientation_change(rng, scale):
    """
    缓慢躺下 / 起身：重力方向有 60~90° 的大幅改变，但**没有冲击**。
    这是"仅靠倾角判断跌倒"必然误报的原因，也是本模型必须区分的关键负样本。
    """
    u = np.array([0.0, 0.0, 1.0])
    ang = rng.uniform(np.radians(60), np.radians(95))
    phi = rng.uniform(0, 2 * np.pi)
    v = np.array([np.sin(ang) * np.cos(phi), np.sin(ang) * np.sin(phi), np.cos(ang)])
    dirs = _rot_between(u, v, WIN)
    am = 1.0 + 0.03 * np.sin(2 * np.pi * 0.4 * np.arange(WIN) / FS)
    sig = np.zeros((WIN, 6))
    sig[:, 0:3] = dirs * am[:, None]
    sig[:, 3:6] = _gyro_from_dirs(dirs) * 0.35          # 慢速转动，角速度很小
    return sig


def _adl_sit_stand(rng, scale):
    """快速坐下 / 起立：合加速度会短暂下探再反弹，但峰值有限且姿态回正。"""
    sig = _adl_static(rng, scale)
    t0 = int(rng.uniform(50, 110))
    dur = int(rng.uniform(25, 45))
    dip = rng.uniform(0.25, 0.40)
    peak = rng.uniform(1.25, 1.70)
    n1 = dur // 2
    sig[t0:t0 + n1, 2] = _ramp(n1, 1.0, 1.0 - dip)
    sig[t0 + n1:t0 + dur, 2] = _ramp(dur - n1, 1.0 - dip, peak)
    sig[t0 + dur:, 2] = _ramp(WIN - t0 - dur, peak, 1.0)
    sig[:, 5] = rng.uniform(15, 45) * np.sin(2 * np.pi * 0.8 * np.arange(WIN) / FS) * \
        np.exp(-np.maximum(np.arange(WIN) - t0, 0) / 40.0)
    sig[:, 0] = 0.05 * np.sin(2 * np.pi * 0.5 * np.arange(WIN) / FS)
    return sig


def _adl_stairs(rng, scale):
    """上下楼：竖直轴低频大幅振荡，角速度中等。"""
    t = np.arange(WIN) / FS
    f = rng.uniform(1.0, 1.5)
    sig = np.zeros((WIN, 6))
    sig[:, 2] = 1.0 + rng.uniform(0.25, 0.45) * np.sin(2 * np.pi * f * t)
    sig[:, 0] = rng.uniform(0.10, 0.25) * np.sin(2 * np.pi * f * t + 1.0)
    sig[:, 1] = rng.uniform(0.08, 0.20) * np.sin(2 * np.pi * f * t + 2.0)
    sig[:, 5] = rng.uniform(40, 100) * np.sin(2 * np.pi * f * t)
    sig[:, 3] = rng.uniform(20, 60) * np.sin(2 * np.pi * f * t + 1.0)
    sig[:, 4] = rng.uniform(40, 110) * np.sin(2 * np.pi * f * t)
    return sig


def _adl_jump(rng, scale):
    """
    跳跃 / 跳下台阶：合加速度峰值可达 2.6~4.2 g，**远超 2.5 g 预筛门限**，
    但落地的瞬间姿态仍然竖直（重力方向几乎不变），落地后立刻恢复活动。
    这是纯阈值法必然误报、必须靠网络区分的典型负样本。
    """
    t = np.arange(WIN) / FS
    base = 1.0 + rng.uniform(0.15, 0.30) * np.sin(2 * np.pi * rng.uniform(1.5, 2.4) * t)
    sig = np.zeros((WIN, 6))
    for _ in range(int(rng.integers(2, 4))):
        t0 = int(rng.integers(15, WIN - 30))
        peak = rng.uniform(2.6, 4.2) * scale
        rise = int(rng.integers(4, 8))
        fall_n = int(rng.integers(12, 20))
        base[t0:t0 + rise] = _ramp(rise, 0.65, peak)
        base[t0 + rise:t0 + rise + fall_n] = _ramp(fall_n, peak, 1.0)
        sig[t0:t0 + 10, 5] += rng.uniform(60, 180) * np.linspace(1.0, 0.0, 10)
    sig[:, 2] = _smooth(base, 4)
    sig[:, 0] = rng.uniform(0.05, 0.15) * np.sin(2 * np.pi * 1.8 * t)
    sig[:, 1] = rng.uniform(0.04, 0.12) * np.sin(2 * np.pi * 1.8 * t + 2.0)
    sig[:, 3:6] += rng.normal(0.0, 15.0, (WIN, 3))
    return sig


def _adl_hard_sit(rng, scale):
    """
    重重坐进椅子 / 跌坐进沙发：合加速度冲击峰值 2.4~3.6 g，与近似跌倒区间重叠。
    区分点在于**姿态几乎不变**（正常坐着，重力方向仍接近竖直）。
    """
    sig = _adl_static(rng, scale)
    t0 = int(rng.uniform(40, 100))
    dur = int(rng.uniform(14, 24))
    dip = rng.uniform(0.30, 0.50)
    peak = rng.uniform(2.4, 3.6) * scale
    n1 = max(dur // 2, 2)
    sig[t0:t0 + n1, 2] = _ramp(n1, 1.0, 1.0 - dip)
    sig[t0 + n1:t0 + dur, 2] = _ramp(dur - n1, 1.0 - dip, peak)
    tail = WIN - t0 - dur
    if tail > 0:
        sig[t0 + dur:, 2] = _ramp(tail, peak, 1.0)
    amp = rng.uniform(40, 120)
    sig[:, 5] = amp * np.exp(-np.maximum(np.arange(WIN) - t0, 0) / rng.uniform(12.0, 25.0)) \
        * np.sin(2 * np.pi * 1.0 * np.arange(WIN) / FS)
    sig[:, 0] = 0.06 * np.sin(2 * np.pi * 0.7 * np.arange(WIN) / FS)
    return sig


def _adl_bend(rng, scale):
    """
    快速弯腰捡东西 / 系鞋带：姿态在 0.4 s 内变化 60~85°，倾角特征与跌倒高度相似，
    但全程**没有失重、没有冲击**（合加速度始终在 1 g 附近小幅波动）。
    """
    u = np.array([0.0, 0.0, 1.0])
    ang = rng.uniform(np.radians(60), np.radians(85))
    phi = rng.uniform(0, 2 * np.pi)
    v = np.array([np.sin(ang) * np.cos(phi), np.sin(ang) * np.sin(phi), np.cos(ang)])
    t0 = int(rng.uniform(40, 70))
    dur = int(rng.uniform(35, 50))
    dirs = np.tile(u, (WIN, 1))
    dirs[t0:t0 + dur] = _rot_between(u, v, dur)
    dirs[t0 + dur:] = v
    # 回程：直起来
    back = _rot_between(v, u, max(WIN - t0 - dur, 2))
    if rng.random() < 0.6:
        dirs[t0 + dur:] = back
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    am = 1.0 + 0.05 * np.sin(2 * np.pi * 0.5 * np.arange(WIN) / FS)
    sig = np.zeros((WIN, 6))
    sig[:, 0:3] = dirs * am[:, None]
    sig[:, 3:6] = _gyro_from_dirs(dirs) * rng.uniform(0.5, 0.8)
    return sig


_ADL_MAKERS = (_adl_walk, _adl_run, _adl_static, _adl_orientation_change,
               _adl_sit_stand, _adl_stairs, _adl_jump, _adl_hard_sit, _adl_bend)


# ---------------------------------------------------------------------------
# 跌倒 FALL —— 三阶段模型
# ---------------------------------------------------------------------------
def _fall(rng, scale):
    """
    自由落体 → 冲击 → 冲击后静止。

    时间轴（单位：采样点，100 Hz）：
        0            t0        t1    t2                    WIN
        |--- 站立 ---|-- 失重 --| 冲击 |---- 躺倒静止 ------|
    """
    t0 = int(rng.uniform(35, 60))            # 0.35~0.60 s 开始失重
    tff = int(rng.uniform(18, 30))           # 失重持续 0.18~0.30 s
    t1 = t0 + tff                            # 冲击起始
    timp = int(rng.uniform(8, 16))           # 冲击持续 0.08~0.16 s
    t2 = t1 + timp

    # --- 合加速度模值包络 am(t) ---
    am = np.ones(WIN)
    am[:t0] = 1.0 + 0.012 * rng.normal(0, 1, t0)
    am[t0:t1] = _ramp(t1 - t0, 1.0, rng.uniform(0.20, 0.50))     # 失重下探
    peak = rng.uniform(3.5, 9.0) * scale                          # 冲击尖峰
    am[t1:t2] = _ramp(t2 - t1, rng.uniform(2.0, 3.5), peak)
    n_tail = WIN - t2
    if n_tail > 0:
        # 冲击后回落到 1 g 附近，并叠加衰减振荡
        base = _ramp(n_tail, peak, 1.0)
        damp = np.exp(-np.arange(n_tail) / rng.uniform(8.0, 16.0))
        osc = 0.10 * damp * np.sin(2 * np.pi * 4.0 * np.arange(n_tail) / FS)
        am[t2:] = base + osc
    am = _smooth(am, 4)

    # --- 姿态轨迹：从竖直翻转到躺倒 ---
    u = np.array([0.0, 0.0, 1.0])
    ang = rng.uniform(np.radians(70), np.radians(105))
    phi = rng.uniform(0, 2 * np.pi)
    v = np.array([np.sin(ang) * np.cos(phi), np.sin(ang) * np.sin(phi), np.cos(ang)])
    # 失重期开始翻转，冲击末期基本到位
    dirs = np.tile(u, (WIN, 1))
    roll_n = max(t2 - t0, 2)
    dirs[t0:t2] = _rot_between(u, v, roll_n)
    dirs[t2:] = v
    dirs[:t0] += rng.normal(0.0, 0.004, (t0, 3))
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)

    sig = np.zeros((WIN, 6))
    sig[:, 0:3] = dirs * am[:, None]
    gyro = _gyro_from_dirs(dirs)
    # 冲击瞬间的角速度尖峰（真实跌倒在此处最剧烈）
    spike = np.zeros(WIN)
    spike[t1:t2] = rng.uniform(150, 600)
    gyro[:, 0] += spike
    gyro[:, 1] += spike * rng.uniform(-0.6, 0.6)
    gyro[:, 2] += spike * rng.uniform(-0.4, 0.4)
    sig[:, 3:6] = gyro

    # 躺倒后仅剩极小的残余晃动
    sig[t2:, 0:3] += rng.normal(0.0, 0.02, (WIN - t2, 3))
    sig[t2:, 3:6] *= 0.05
    return sig


# ---------------------------------------------------------------------------
# 近似跌倒 NEAR_FALL
# ---------------------------------------------------------------------------
def _near_fall(rng, scale):
    """
    踉跄 / 险些跌倒：
      · 有失重、有冲击，但**冲击峰值明显低于跌倒**（1.8~2.9 g）
      · 姿态偏转后被**拉回来**，结束朝向接近起始朝向
      · 冲击后仍有明显运动（人在恢复平衡、迈步），不是静止
    这三条正好把 NEAR_FALL 放在 FALL 与 ADL 的中间地带，是最难分的类。
    """
    t0 = int(rng.uniform(40, 70))
    tff = int(rng.uniform(10, 20))
    t1 = t0 + tff
    timp = int(rng.uniform(8, 14))
    t2 = t1 + timp

    am = np.ones(WIN)
    am[:t0] = 1.0 + 0.015 * rng.normal(0, 1, t0)
    am[t0:t1] = _ramp(t1 - t0, 1.0, rng.uniform(0.45, 0.75))     # 轻微失重
    peak = rng.uniform(1.8, 2.9) * scale
    am[t1:t2] = _ramp(t2 - t1, rng.uniform(1.3, 1.9), peak)
    n_tail = WIN - t2
    base = _ramp(n_tail, peak, 1.0)
    t = np.arange(n_tail) / FS
    # 恢复期的迈步动作：持续振荡，这是与"跌倒后静止"的决定性差异
    recover = rng.uniform(0.12, 0.35) * np.sin(2 * np.pi * rng.uniform(1.2, 2.2) * t)
    am[t2:] = base + recover * np.exp(-t / rng.uniform(0.8, 2.0))
    am = _smooth(am, 4)

    u = np.array([0.0, 0.0, 1.0])
    # 偏转出去再拉回来
    ang = rng.uniform(np.radians(25), np.radians(55))
    phi = rng.uniform(0, 2 * np.pi)
    v = np.array([np.sin(ang) * np.cos(phi), np.sin(ang) * np.sin(phi), np.cos(ang)])
    dirs = np.tile(u, (WIN, 1))
    out_n = max(t2 - t0, 2)
    dirs[t0:t2] = _rot_between(u, v, out_n)
    back_n = WIN - t2
    if back_n > 1:
        # 部分恢复：回到与竖直方向夹角 5~20° 的位置
        w = u * rng.uniform(0.97, 0.999) + v * rng.uniform(0.02, 0.2)
        w /= np.linalg.norm(w)
        dirs[t2:] = _rot_between(v, w, back_n)
    dirs[:t0] += rng.normal(0.0, 0.005, (t0, 3))
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)

    sig = np.zeros((WIN, 6))
    sig[:, 0:3] = dirs * am[:, None]
    gyro = _gyro_from_dirs(dirs)
    spike = np.zeros(WIN)
    spike[t1:t2] = rng.uniform(80, 260)
    gyro[:, 0] += spike
    gyro[:, 1] += spike * rng.uniform(-0.5, 0.5)
    sig[:, 3:6] = gyro
    return sig


# ---------------------------------------------------------------------------
# 对外接口
# ---------------------------------------------------------------------------
def make_window(rng, cls, scale=1.0):
    """按类别生成单个 (200, 6) 窗口。"""
    if cls == ADL:
        return rng.choice(_ADL_MAKERS)(rng, scale)
    if cls == NEAR_FALL:
        return _near_fall(rng, scale)
    if cls == FALL:
        return _fall(rng, scale)
    raise ValueError("unknown class %r" % (cls,))


def make_dataset(seed, n_per_class, subject_scale=(1.0, 1.0),
                 acc_sigma=0.02, gyro_sigma=4.0, balance=True):
    """
    生成一个数据集。

    seed          : 随机种子（训练集与测试集必须用不同种子）
    n_per_class   : 每类窗口数
    subject_scale : (lo, hi) 受试者尺度扰动 —— 模拟不同身高体重带来的冲击差异
    acc_sigma     : 加速度计噪声标准差 (g)
    gyro_sigma    : 陀螺仪噪声标准差 (°/s)

    balance       : 各类样本数的重分配。真实场景里跌倒远少于日常，训练时做类别
                    平衡，但**测试集必须用真实先验**才能得到有意义的误报率，
                    所以 verify 阶段会单独按真实先验重采样。

    返回: X (N,200,6) float64, y (N,) int64
    """
    rng = np.random.default_rng(seed)
    X = np.empty((n_per_class * 3, WIN, N_CH), dtype=np.float64)
    y = np.empty(n_per_class * 3, dtype=np.int64)
    k = 0
    for cls in (ADL, NEAR_FALL, FALL):
        for _ in range(n_per_class):
            scale = rng.uniform(*subject_scale)
            sig = make_window(rng, cls, scale)
            sig = _add_noise(rng, sig, acc_sigma, gyro_sigma)
            X[k] = sig
            y[k] = cls
            k += 1
    idx = rng.permutation(len(y))
    return X[idx], y[idx]


def to_frame_units(X):
    """
    把 (N,200,6) 的物理量转成节点间帧协议的整数单位，用于走一遍真实数据通路：
        acc   g      → mg      (int16)
        gyro  °/s    → 0.1°/s  (int16)
    C 端收到的就是这两个整数，特征提取再转回物理量。
    """
    out = np.empty(X.shape, dtype=np.int64)
    out[..., 0:3] = np.round(X[..., 0:3] * 1000.0)
    out[..., 3:6] = np.round(X[..., 3:6] * 10.0)
    return np.clip(out, -32768, 32767).astype(np.int16)


if __name__ == "__main__":
    import time

    t = time.time()
    Xtr, ytr = make_dataset(1, 300)
    print("train", Xtr.shape, np.bincount(ytr), "%.1fs" % (time.time() - t))
    Xte, yte = make_dataset(999, 100, subject_scale=(0.85, 1.18))
    print("test ", Xte.shape, np.bincount(yte))
    print("帧单位范围: acc(mg) ±%d, gyro(0.1°/s) ±%d"
          % (np.abs(to_frame_units(Xtr)[..., 0:3]).max(),
             np.abs(to_frame_units(Xtr)[..., 3:6]).max()))
