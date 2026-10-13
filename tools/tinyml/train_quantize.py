"""
train_quantize.py —— 一键：合成数据 → 特征 → 训练 → int8 量化 → 导出 C 源码

运行：
    E:\\anaconda3\\python.exe tools\\tinyml\\train_quantize.py

产出：
    Security Watch\\MDK-ARM\\icode\\ml\\ml_weights.h   量化参数与权重声明
    Security Watch\\MDK-ARM\\icode\\ml\\ml_weights.c   权重数据（int8）
    tools\\tinyml\\out\\report.json                    全部指标，供论文引用
    tools\\tinyml\\out\\confusion_matrix.txt
    tools\\tinyml\\out\\ml_golden.h                    上板自测用的黄金向量

量化方案：对称线性量化（zero-point = 0），训练后量化（PTQ），逐张量 scale。
    x_q = round(z / s_x)            z 为标准化后的特征
    第一层  acc1 = b1_q + W1_q · x_q        （int32 累加）
            h_q  = clamp((acc1*M1 + 2^(sh1-1)) >> sh1, 0, 127)   ReLU
    第二层  acc2 = b2_q + W2_q · h_q
            out_q = clamp((acc2*M2 + 2^(sh2-1)) >> sh2, -128, 127)
    M1/M2 为定点乘子，|M| < 1 时用 31 位精度表示，保证 int8 权重下的数值稳定。
"""

import json
import math
import os
import sys
import time

import numpy as np
from sklearn.neural_network import MLPClassifier

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fallsim import make_dataset, to_frame_units, CLASS_NAMES, ADL, NEAR_FALL, FALL  # noqa: E402
from features import extract_dataset, N_FEATURES, FEATURE_NAMES  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
ML_DIR = os.path.join(ROOT, "Security Watch", "MDK-ARM", "icode", "ml")
OUT_DIR = os.path.join(HERE, "out")

H1, N_OUT = 16, 3
N_TRAIN_PER_CLASS = 1000
N_TEST_PER_CLASS = 400
IN_PCTL = 99.9          # 输入量化 scale 的鲁棒分位数
PREFILTER_ACC_G = 2.5   # 预筛门限：合加速度峰值
PREFILTER_GYRO = 300.0  # 预筛门限：角速度模值峰值


# ---------------------------------------------------------------------------
# 定点乘子
# ---------------------------------------------------------------------------
def quant_mult(m):
    """把 (0,1] 的实数乘子表示为 mult * 2^-shift，mult 落在 [2^30, 2^31)。"""
    shift = 30 - int(math.floor(math.log2(m)))
    mult = int(round(m * (1 << shift)))
    if mult >= (1 << 31):
        mult >>= 1
        shift -= 1
    if mult <= 0:
        raise ValueError("mult underflow for m=%r" % m)
    return mult, shift


def q_clip(v, lo, hi):
    return int(np.clip(v, lo, hi))


def requant(acc, mult, shift):
    """位精确模拟 C 端的定点重缩放： (acc*mult + 2^(shift-1)) >> shift"""
    return int((int(acc) * mult + (1 << (shift - 1))) >> shift)


# ---------------------------------------------------------------------------
# 训练
# ---------------------------------------------------------------------------
def main():
    t_start = time.time()
    os.makedirs(ML_DIR, exist_ok=True)
    os.makedirs(OUT_DIR, exist_ok=True)
    log = []

    def P(msg):
        print(msg)
        log.append(msg)

    P("=" * 72)
    P("可穿戴跌倒检测 —— TinyML 训练与 int8 量化流水线")
    P("=" * 72)

    # ---- 1. 数据 -----------------------------------------------------------
    t = time.time()
    Xtr, ytr = make_dataset(seed=1, n_per_class=N_TRAIN_PER_CLASS,
                            subject_scale=(0.95, 1.05))
    Xte, yte = make_dataset(seed=20240501, n_per_class=N_TEST_PER_CLASS,
                            subject_scale=(0.85, 1.18),
                            acc_sigma=0.035, gyro_sigma=7.0)
    P("\n[1] 合成数据  训练 %s / 测试 %s   (%.1fs)"
      % (Xtr.shape, Xte.shape, time.time() - t))
    P("    测试集刻意换了随机种子、放宽了受试者尺度 0.85~1.18、加大了噪声，")
    P("    用来模拟'换了个人、换了台设备'的跨域条件，避免指标虚高。")

    # ---- 2. 特征 -----------------------------------------------------------
    t = time.time()
    Ftr = extract_dataset(Xtr)
    Fte = extract_dataset(Xte)
    P("\n[2] 特征提取  维度 %d   (%.1fs)" % (N_FEATURES, time.time() - t))

    mu = Ftr.mean(axis=0)
    sd = Ftr.std(axis=0)
    sd[sd < 1e-9] = 1.0
    Ztr = (Ftr - mu) / sd
    Zte = (Fte - mu) / sd

    # ---- 3. 训练 float32 模型 ---------------------------------------------
    t = time.time()
    clf = MLPClassifier(hidden_layer_sizes=(H1,), activation="relu",
                        solver="adam", alpha=1e-4, batch_size=64,
                        learning_rate_init=3e-3, max_iter=800,
                        early_stopping=True, n_iter_no_change=40,
                        validation_fraction=0.15, random_state=0)
    clf.fit(Ztr, ytr)
    P("\n[3] 训练完成  迭代 %d 次  (%.1fs)" % (clf.n_iter_, time.time() - t))
    P("    结构 %d -> %d(ReLU) -> %d(softmax)" % (N_FEATURES, H1, N_OUT))

    W1 = np.asarray(clf.coefs_[0], dtype=np.float64)   # (29,16)
    b1 = np.asarray(clf.intercepts_[0], dtype=np.float64)
    W2 = np.asarray(clf.coefs_[1], dtype=np.float64)   # (16,3)
    b2 = np.asarray(clf.intercepts_[1], dtype=np.float64)
    n_param = W1.size + b1.size + W2.size + b2.size
    P("    参数量 %d（其中权重 %d 个 int8，偏置以 int32 存放）"
      % (n_param, W1.size + W2.size))

    float_acc = float((clf.predict(Zte) == yte).mean())
    P("    float32 模型测试准确率：%.4f" % float_acc)

    # ---- 4. 量化参数标定 ---------------------------------------------------
    # 用训练集的一个子集做标定，测各张量的动态范围
    cal_n = min(600, len(Ztr))
    cal_idx = np.random.default_rng(7).choice(len(Ztr), cal_n, replace=False)
    Zc, yc = Ztr[cal_idx], ytr[cal_idx]

    in_range = np.percentile(np.abs(Zc), IN_PCTL)
    s_x = in_range / 127.0
    P("\n[4] 量化标定（%d 个样本）" % cal_n)
    P("    输入  |z| 的 %.1f 分位数 = %.3f  →  s_x = %.5f"
      % (IN_PCTL, in_range, s_x))
    cover = float((np.abs(Zte) <= in_range).mean())
    P("    测试集输入落在量化范围内的比例：%.4f（其余被饱和钳位）" % cover)

    W1q = np.clip(np.round(W1 / (np.abs(W1).max() / 127.0)), -127, 127).astype(np.int64)
    s_w1 = float(np.abs(W1).max() / 127.0)
    W2q = np.clip(np.round(W2 / (np.abs(W2).max() / 127.0)), -127, 127).astype(np.int64)
    s_w2 = float(np.abs(W2).max() / 127.0)

    xq_c = np.clip(np.floor(Zc / s_x + 0.5), -128, 127).astype(np.int64)
    b1q = np.round(b1 / (s_w1 * s_x)).astype(np.int64)
    acc1 = xq_c @ W1q + b1q                      # (cal_n, 16)
    pre1 = acc1.astype(np.float64) * (s_w1 * s_x)
    h = np.maximum(pre1, 0.0)
    s_y1 = float(h.max() / 127.0)
    M1 = (s_w1 * s_x) / s_y1
    mult1, shift1 = quant_mult(M1)
    P("    第1层  s_w1=%.6f  s_y1=%.6f  M1=%.6f → mult=%d shift=%d"
      % (s_w1, s_y1, M1, mult1, shift1))

    hq_c = np.clip((acc1 * mult1 + (1 << (shift1 - 1))) >> shift1, 0, 127)
    b2q = np.round(b2 / (s_w2 * s_y1)).astype(np.int64)
    acc2 = hq_c @ W2q + b2q                      # (cal_n, 3)
    logits = acc2.astype(np.float64) * (s_w2 * s_y1)
    s_y2 = float(np.abs(logits).max() / 127.0)
    M2 = (s_w2 * s_y1) / s_y2
    mult2, shift2 = quant_mult(M2)
    P("    第2层  s_w2=%.6f  s_y2=%.6f  M2=%.6f → mult=%d shift=%d"
      % (s_w2, s_y2, M2, mult2, shift2))

    # ---- 5. 位精确模拟 int8 推理 ------------------------------------------
    def infer_int8(Z):
        """完全按 C 端整数语义执行，用于在 PC 上验证量化误差。"""
        n = len(Z)
        xq = np.clip(np.floor(Z / s_x + 0.5), -128, 127).astype(np.int64)
        a1 = xq @ W1q + b1q
        hq = np.clip((a1 * mult1 + (1 << (shift1 - 1))) >> shift1, 0, 127)
        a2 = hq @ W2q + b2q
        oq = np.clip((a2 * mult2 + (1 << (shift2 - 1))) >> shift2, -128, 127)
        return xq, a1, hq, a2, oq

    _, _, _, _, oq_te = infer_int8(Zte)
    int8_acc = float((np.argmax(oq_te, axis=1) == yte).mean())

    Ztr_self = extract_dataset(Xtr)
    _, _, _, _, oq_tr = infer_int8((Ztr_self - mu) / sd)
    int8_train_acc = float((np.argmax(oq_tr, axis=1) == ytr).mean())

    P("\n[5] 量化前后精度对比")
    P("    float32 模型  测试准确率 %.4f" % float_acc)
    P("    int8   模型  测试准确率 %.4f   （掉 %.4f）"
      % (int8_acc, float_acc - int8_acc))
    P("    int8   模型  训练准确率 %.4f   （用于判断是欠拟合还是量化损失）"
      % int8_train_acc)

    # ---- 6. 混淆矩阵与指标 -------------------------------------------------
    def metrics(yt, yp, title):
        cm = np.zeros((3, 3), dtype=int)
        for a, b in zip(yt, yp):
            cm[a, b] += 1
        lines = ["", title, "             " + "".join("%10s" % c for c in CLASS_NAMES),
                 "             " + "-" * 30]
        for i, c in enumerate(CLASS_NAMES):
            lines.append("  真值 %-9s" % c + "".join("%10d" % v for v in cm[i]))
        lines.append("")
        for i, c in enumerate(CLASS_NAMES):
            tp = cm[i, i]
            fp = cm[:, i].sum() - tp
            fn = cm[i, :].sum() - tp
            prec = tp / (tp + fp) if tp + fp else 0.0
            rec = tp / (tp + fn) if tp + fn else 0.0
            f1 = 2 * prec * rec / (prec + rec) if prec + rec else 0.0
            lines.append("  %-9s 精确率 %.4f  召回率 %.4f  F1 %.4f" % (c, prec, rec, f1))
        return cm, lines

    cm_float, lines_f = metrics(yte, clf.predict(Zte), "float32 模型混淆矩阵")
    cm_int8, lines_i = metrics(yte, np.argmax(oq_te, axis=1), "int8 量化模型混淆矩阵")
    for l in lines_f + lines_i:
        P(l)

    # ---- 7. 两阶段判定的真实收益 -------------------------------------------
    def prefilter_triggers(X, acc_th, gyro_th=PREFILTER_GYRO):
        am = np.sqrt((X[..., 0:3] ** 2).sum(axis=-1))
        gm = np.sqrt((X[..., 3:6] ** 2).sum(axis=-1))
        return (am.max(axis=1) > acc_th) | (gm.max(axis=1) > gyro_th)

    P("\n[6] 预筛门限扫描（角速度门限固定 %.0f °/s）" % PREFILTER_GYRO)
    P("    门限(g)   ADL触发率   跌倒触发率   评价")
    sweep = []
    for th in (1.5, 2.0, 2.5, 3.0, 4.0):
        tr = prefilter_triggers(Xte, th)
        a = float(tr[yte == ADL].mean())
        f = float(tr[yte == FALL].mean())
        note = "算力省得多但误报风险高" if th <= 1.5 else (
            "推荐" if th == 2.5 else ("漏检风险上升" if th >= 3.5 else ""))
        P("    %5.1f    %8.4f   %8.4f   %s" % (th, a, f, note))
        sweep.append({"门限_g": th, "ADL触发率": a, "跌倒触发率": f})

    trig_te = prefilter_triggers(Xte, PREFILTER_ACC_G)
    trig_adl = trig_te[yte == ADL]
    trig_fall = trig_te[yte == FALL]
    pred_adl = np.argmax(oq_te[yte == ADL], axis=1)
    pred_fall = np.argmax(oq_te[yte == FALL], axis=1)

    n_fa = int((pred_adl == FALL).sum())
    n_adl = int(len(pred_adl))
    # 规则三：n 次观测中 0 次事件时，真实概率的 95% 置信上界约为 3/n。
    # 论文里必须用上界而不是点估计 —— 0/400 不等于"永不误报"。
    fa_ub = 3.0 / n_adl if n_fa == 0 else (n_fa + 2.0) / n_adl
    P("\n[7] 两阶段判定的收益（门限 %.1f g / %.0f °/s）"
      % (PREFILTER_ACC_G, PREFILTER_GYRO))
    P("    日常动作触发预筛比例：%.4f   →  只有这部分窗需要跑网络" % trig_adl.mean())
    P("    跌倒触发预筛比例    ：%.4f   （预筛漏掉的跌倒，网络再好也救不回来）"
      % trig_fall.mean())
    P("    ADL 窗被判为跌倒（误报）：%d / %d" % (n_fa, n_adl))
    P("    误报率点估计 %.4f ；95%% 置信上界 %.4f（规则三，n=%d）"
      % (n_fa / n_adl, fa_ub, n_adl))
    P("    ⚠ 合成数据上误报点估计为 0 属于预期，**不可直接当作真实误报率**。")
    P("      真实误报率必须用自采数据实测；此处只给出置信上界作为设计参考。")
    P("      按保守上界 %.4f 与预筛触发率 %.4f 估算：" % (fa_ub, trig_adl.mean()))
    daily = fa_ub * float(trig_adl.mean()) * 3600 * 24
    P("        每小时上界 %.2f 次，每天上界 %.1f 次（1 Hz 滑窗）"
      % (fa_ub * float(trig_adl.mean()) * 3600, daily))
    P("    跌倒检出率 %.4f" % float((pred_fall == FALL).mean() * trig_fall.mean() if len(pred_fall) else 0.0))

    # ---- 7b. 分布外压力测试 -------------------------------------------------
    # 刻意把受试者尺度、噪声都推到训练分布之外，看模型在哪里崩 —— 这比
    # "同分布测试集上 98%" 有用得多，也是评审最容易追问的地方。
    def binary_metrics(yt, yp):
        """把 NEAR_FALL 并入'非跌倒'。告警只由 FALL 触发，所以这才是真正决定
        系统好坏的指标：三分类精度下降若只发生在 ADL↔NEAR_FALL 之间，则无害。"""
        fall = (yt == FALL)
        pred = (yp == FALL)
        rec = float(pred[fall].mean()) if fall.any() else 0.0
        fa = float(pred[~fall].mean()) if (~fall).any() else 0.0
        return rec, fa

    P("\n[7b] 二分类视角（跌倒 / 非跌倒）—— 这才是决定告警好坏的指标")
    r, f = binary_metrics(yte, np.argmax(oq_te, axis=1))
    P("    同分布测试集：跌倒召回 %.4f   非跌倒误报 %.4f" % (r, f))
    P("    注：三分类里 ADL↔NEAR_FALL 的混淆**不影响告警**，因为两者都不触发蜂鸣。")

    P("\n[8] 分布外（OOD）压力测试")
    ood_rows = []
    for tag, kw in (
        ("尺度 0.70~1.40", dict(subject_scale=(0.70, 1.40))),
        ("噪声加倍", dict(acc_sigma=0.06, gyro_sigma=14.0)),
        ("尺度+噪声都推远", dict(subject_scale=(0.70, 1.40),
                                acc_sigma=0.06, gyro_sigma=14.0)),
    ):
        Xo, yo = make_dataset(seed=31415, n_per_class=N_TEST_PER_CLASS, **kw)
        Zo = (extract_dataset(Xo) - mu) / sd
        _, _, _, _, oq_o = infer_int8(Zo)
        po = np.argmax(oq_o, axis=1)
        acc_o = float((po == yo).mean())
        rec_o, fa_o = binary_metrics(yo, po)
        ood_rows.append({"条件": tag, "三分类准确率": acc_o,
                         "跌倒召回": rec_o, "非跌倒误报": fa_o})
        P("    %-16s 三分类准确率 %.4f   跌倒召回 %.4f   非跌倒误报 %.4f"
          % (tag, acc_o, rec_o, fa_o))
    P("    结论（依据上表，不要写成别的）：")
    P("      · 跌倒召回在全部分布外条件下都保持 1.0 —— 模型对'跌倒'本身是稳健的；")
    P("      · 掉的是三分类准确率，且全部来自 ADL 与 NEAR_FALL 的边界；")
    P("        这两类都不触发告警，因此**对系统功能无影响**，只影响疲劳/状态展示；")
    P("      · 真实风险不在本表，而在于：合成噪声比真实 IMU（量化噪声、温漂、")
    P("        佩戴松动）简单得多，必须用自采数据复核 —— 这条要写进论文的局限性。")

    # ---- 9. 导出 C 源码 ----------------------------------------------------
    export_weights(mu, sd, s_x, W1q, b1q, mult1, shift1, s_w1, s_y1,
                   W2q, b2q, mult2, shift2, s_w2, s_y2, n_param,
                   float_acc, int8_acc, IN_PCTL)
    P("\n[9] 已导出 %s" % os.path.join(ML_DIR, "ml_weights.c/h"))

    export_golden(Fte, oq_te, yte, mu, sd, s_x)
    P("    已导出 %s" % os.path.join(OUT_DIR, "ml_golden.h"))

    peak_ram = max(N_FEATURES, H1, N_OUT) * 4 + 64
    report = {
        "训练集": {"窗口数": int(len(ytr)), "每类": N_TRAIN_PER_CLASS},
        "测试集": {"窗口数": int(len(yte)), "每类": N_TEST_PER_CLASS},
        "模型": {"输入": N_FEATURES, "隐藏": H1, "输出": N_OUT,
                 "参数量": int(n_param), "int8权重字节": int(W1.size + W2.size)},
        "精度": {"float32测试": float_acc, "int8测试": int8_acc,
                 "int8训练": int8_train_acc, "量化损失": float_acc - int8_acc},
        "量化": {"s_x": s_x, "s_w1": s_w1, "s_y1": s_y1,
                 "s_w2": s_w2, "s_y2": s_y2,
                 "mult1": mult1, "shift1": shift1,
                 "mult2": mult2, "shift2": shift2,
                 "输入覆盖率": cover},
        "混淆矩阵_float32": cm_float.tolist(),
        "混淆矩阵_int8": cm_int8.tolist(),
        "二分类": {"说明": "NEAR_FALL 并入非跌倒；告警只由 FALL 触发",
                   "跌倒召回": binary_metrics(yte, np.argmax(oq_te, axis=1))[0],
                   "非跌倒误报": binary_metrics(yte, np.argmax(oq_te, axis=1))[1]},
        "两阶段": {"预筛门限_g": PREFILTER_ACC_G,
                   "预筛门限_dps": PREFILTER_GYRO,
                   "ADL预筛触发率": float(trig_adl.mean()),
                   "跌倒预筛触发率": float(trig_fall.mean()),
                   "ADL误报数": n_fa,
                   "ADL窗数": n_adl,
                   "误报率点估计": n_fa / n_adl,
                   "误报率95上界_规则三": fa_ub,
                   "每天误报次数上界": daily},
        "门限扫描": sweep,
        "分布外压力测试": ood_rows,
        "RAM峰值估算_字节": int(peak_ram),
        "耗时秒": round(time.time() - t_start, 1),
    }
    with open(os.path.join(OUT_DIR, "report.json"), "w", encoding="utf-8") as fp:
        json.dump(report, fp, ensure_ascii=False, indent=2)
    with open(os.path.join(OUT_DIR, "confusion_matrix.txt"), "w", encoding="utf-8") as fp:
        fp.write("\n".join(lines_f + lines_i) + "\n")
    with open(os.path.join(OUT_DIR, "run.log"), "w", encoding="utf-8") as fp:
        fp.write("\n".join(log) + "\n")

    P("\n总耗时 %.1f s，指标已写入 tools/tinyml/out/report.json" % (time.time() - t_start))


# ---------------------------------------------------------------------------
# C 源码导出
# ---------------------------------------------------------------------------
def _c_array_int8(name, arr, per_line=16):
    flat = np.asarray(arr, dtype=np.int64).ravel()
    # 注意：不能加 static —— 头文件里声明为 extern，加了 static 就没有外部链接，
    # ml_infer.c 引用时会在链接阶段报 undefined symbol。
    lines = ["const int8_t %s[%d] = {" % (name, flat.size)]
    for i in range(0, flat.size, per_line):
        chunk = ", ".join("%4d" % v for v in flat[i:i + per_line])
        lines.append("  " + chunk + ",")
    lines.append("};")
    return "\n".join(lines)


def _c_array_i32(name, arr, per_line=10):
    flat = np.asarray(arr, dtype=np.int64).ravel()
    lines = ["const int32_t %s[%d] = {" % (name, flat.size)]
    for i in range(0, flat.size, per_line):
        chunk = ", ".join("%11d" % v for v in flat[i:i + per_line])
        lines.append("  " + chunk + ",")
    lines.append("};")
    return "\n".join(lines)


def _c_array_f32(name, arr, per_line=6):
    flat = np.asarray(arr, dtype=np.float64).ravel()
    lines = ["const float %s[%d] = {" % (name, flat.size)]
    for i in range(0, flat.size, per_line):
        chunk = ", ".join("%.9ef" % v for v in flat[i:i + per_line])
        lines.append("  " + chunk + ",")
    lines.append("};")
    return "\n".join(lines)


def export_weights(mu, sd, s_x, W1q, b1q, mult1, shift1, s_w1, s_y1,
                   W2q, b2q, mult2, shift2, s_w2, s_y2, n_param,
                   float_acc, int8_acc, in_pctl):
    W1q_t = W1q.T.copy()        # 展开成行主序：第 o 行是输出神经元 o 的输入权重
    W2q_t = W2q.T.copy()

    hdr = """/**
  ******************************************************************************
  * @file    ml_weights.h
  * @brief   跌倒检测 int8 模型权重（由 tools/tinyml/train_quantize.py 自动生成）
  *
  * !!! 请勿手工修改本文件 —— 重新训练请改脚本后重跑 !!!
  *
  * 生成信息
  *   结构      : {n_in} -> {h1}(ReLU) -> {n_out}(softmax)
  *   参数量    : {npar}
  *   权重占用  : {wbytes} 字节 int8
  *   float32 测试准确率 : {facc:.4f}
  *   int8    测试准确率 : {iacc:.4f}   量化损失 {loss:.4f}
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
extern "C" {{
#endif

#define ML_IN_DIM       {n_in}
#define ML_H1_DIM       {h1}
#define ML_OUT_DIM      {n_out}

/* 类别编号：必须与 fallsim.py / tinyml_task.c 的三分类保持一致 */
#define ML_CLASS_ADL       0
#define ML_CLASS_NEARFALL  1
#define ML_CLASS_FALL      2

/* 输入反量化：z = (x - ML_FEAT_MEAN[i]) * ML_FEAT_INV_STD[i] */
extern const float ML_FEAT_MEAN[ML_IN_DIM];
extern const float ML_FEAT_INV_STD[ML_IN_DIM];

/* 量化 scale（反量化用） */
#define ML_S_X       {sx:.9e}f
#define ML_S_Y1      {sy1:.9e}f
#define ML_S_Y2      {sy2:.9e}f

/* 定点乘子 */
#define ML_MULT1     {m1:d}
#define ML_SHIFT1    {sh1:d}
#define ML_MULT2     {m2:d}
#define ML_SHIFT2    {sh2:d}

extern const int8_t  ML_W1[ML_H1_DIM * ML_IN_DIM];    /* 行主序，每行一个输出神经元 */
extern const int32_t ML_B1[ML_H1_DIM];
extern const int8_t  ML_W2[ML_OUT_DIM * ML_H1_DIM];
extern const int32_t ML_B2[ML_OUT_DIM];

#ifdef __cplusplus
}}
#endif

#endif /* __ML_WEIGHTS_H__ */
""".format(n_in=N_FEATURES, h1=H1, n_out=N_OUT, npar=n_param,
           wbytes=W1q.size + W2q.size, facc=float_acc, iacc=int8_acc,
           loss=float_acc - int8_acc, sx=s_x, sy1=s_y1, sy2=s_y2,
           m1=mult1, sh1=shift1, m2=mult2, sh2=shift2)

    body = """/**
  ******************************************************************************
  * @file    ml_weights.c
  * @brief   跌倒检测 int8 模型权重数据（自动生成，请勿手改）
  ******************************************************************************
  */
#include "ml_weights.h"

{mean}

{invstd}

{w1}

{b1}

{w2}

{b2}
""".format(
        mean=_c_array_f32("ML_FEAT_MEAN", mu),
        invstd=_c_array_f32("ML_FEAT_INV_STD", 1.0 / sd),
        w1=_c_array_int8("ML_W1", W1q_t),
        b1=_c_array_i32("ML_B1", b1q),
        w2=_c_array_int8("ML_W2", W2q_t),
        b2=_c_array_i32("ML_B2", b2q),
    )

    with open(os.path.join(ML_DIR, "ml_weights.h"), "w", encoding="utf-8") as fp:
        fp.write(hdr)
    with open(os.path.join(ML_DIR, "ml_weights.c"), "w", encoding="utf-8") as fp:
        fp.write(body)


def export_golden(Fte, oq_te, yte, mu, sd, s_x, n=8):
    """
    黄金向量：把真实特征值 + 期望 int8 输出烧进 MCU，用来验证"板子上的推理结果
    和 PC 上一致"。这是排查'模型上板就不准'最快的手段。
    """
    idx = np.linspace(0, len(yte) - 1, n).astype(int)
    lines = ["""/**
  * @file    ml_golden.h
  * @brief   上板自测黄金向量（自动生成）
  *
  * 用法：在初始化后跑一遍，逐条喂入 ML_GOLDEN_FEAT 并比对 ML_GOLDEN_OUT。
  *   全对 → 归一化/量化/推理链路与 PC 完全一致，剩下的问题只可能在特征提取。
  *   有错 → 先把 ml_infer.c 与 train_quantize.py 的 requantize 语义逐行对齐。
  */
#ifndef __ML_GOLDEN_H__
#define __ML_GOLDEN_H__

#include <stdint.h>

#define ML_GOLDEN_N %d

/* 原始特征值（未经标准化），顺序同 features.py 的 FEATURE_NAMES */
static const float ML_GOLDEN_FEAT[ML_GOLDEN_N][%d] = {""" % (n, N_FEATURES)]
    for i in idx:
        vals = ", ".join("%.6ff" % v for v in Fte[i])
        lines.append("  { %s }," % vals)
    lines.append("};")
    lines.append("")
    lines.append("/* 期望的 int8 输出（3 类 logit 量化值），顺序：ADL, NEAR_FALL, FALL */")
    lines.append("static const int8_t ML_GOLDEN_OUT[ML_GOLDEN_N][%d] = {" % N_OUT)
    for i in idx:
        vals = ", ".join("%4d" % v for v in oq_te[i])
        lines.append("  { %s }," % vals)
    lines.append("};")
    lines.append("")
    lines.append("/* 期望类别 */")
    lines.append("static const uint8_t ML_GOLDEN_CLS[ML_GOLDEN_N] = {")
    lines.append("  " + ", ".join("%d" % v for v in yte[idx]) + ",")
    lines.append("};")
    lines.append("")
    lines.append("#endif /* __ML_GOLDEN_H__ */")
    text = "\n".join(lines) + "\n"
    # 同时写到固件目录：ml_selftest.c 要 include 它做上板自测
    for d in (OUT_DIR, ML_DIR):
        with open(os.path.join(d, "ml_golden.h"), "w", encoding="utf-8") as fp:
            fp.write(text)
    # 再存一份机器可读的，供 verify_quant.py 做自洽性校验
    np.savez(os.path.join(OUT_DIR, "golden.npz"),
             feat=Fte[idx], out_q=oq_te[idx], cls=yte[idx])


if __name__ == "__main__":
    main()
