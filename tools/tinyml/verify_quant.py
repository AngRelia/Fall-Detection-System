"""
verify_quant.py —— 从**导出的 C 代码**反向验证整条链路

为什么要有这个脚本：
    train_quantize.py 是在 Python 内存里跑的，它验证不了"导出到 C 的那一步有没有错"。
    实际项目里最常见、最难查的故障恰恰在这里：
        · 权重按行主序导出，但 C 端按下标跨行读  → 结果全错，却不报任何错
        · 特征顺序在 Python 与 C 里错位一位
        · 移位前忘了加四舍五入，量化误差被放大

    本脚本把 ml_weights.h / ml_weights.c **当成唯一的真值来源**（正则解析出来，
    一个字都不信 Python 侧的记忆），用它重跑一遍测试集，然后：

      1. 复现 report.json 里的指标  → 证明导出无损
      2. 故意把权重矩阵转置        → 证明本验证对这类错误是敏感的（不是走过场）
      3. 把特征按 float32 舍入     → 评估 C 用 float、Python 用 double 带来的风险
      4. 模拟 C 端 ml_selftest      → 确认黄金向量在上板前就是自洽的

运行：
    E:\\anaconda3\\python.exe tools\\tinyml\\verify_quant.py
退出码 0 = 全部通过。
"""

import json
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fallsim import make_dataset, FALL, ADL, NEAR_FALL, CLASS_NAMES  # noqa: E402
from features import extract_dataset, N_FEATURES  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
ML_DIR = os.path.join(ROOT, "Security Watch", "MDK-ARM", "icode", "ml")
OUT_DIR = os.path.join(HERE, "out")

# 必须与 train_quantize.py 完全一致，否则拿不到同一个测试集
N_TEST_PER_CLASS = 400
TEST_SEED = 20240501


# ---------------------------------------------------------------------------
# 解析 C 源码
# ---------------------------------------------------------------------------
def parse_defines(path):
    txt = open(path, encoding="utf-8").read()
    out = {}
    for name, val in re.findall(r"#define\s+(ML_\w+)\s+([0-9eE+\-.]+)f?\s*$", txt, re.M):
        out[name] = float(val) if ("." in val or "e" in val.lower()) else int(val)
    return out


def parse_array(path, name):
    txt = open(path, encoding="utf-8").read()
    m = re.search(r"const\s+\w+\s+%s\s*\[[^\]]*\]\s*=\s*\{(.*?)\};" % name, txt, re.S)
    if not m:
        raise SystemExit("在 %s 里找不到数组 %s" % (path, name))
    body = m.group(1)
    toks = [t.strip() for t in body.replace("\n", " ").split(",") if t.strip()]
    if re.search(r"float\s+%s" % name, txt):
        return np.array([float(t.rstrip("f")) for t in toks], dtype=np.float64)
    return np.array([int(t) for t in toks], dtype=np.int64)


# ---------------------------------------------------------------------------
# 纯整数推理（只用从 C 代码解析出来的常数）
# ---------------------------------------------------------------------------
class Int8Model(object):
    def __init__(self, ml_dir, transpose_w=False):
        h = os.path.join(ml_dir, "ml_weights.h")
        c = os.path.join(ml_dir, "ml_weights.c")
        d = parse_defines(h)
        self.in_dim = d["ML_IN_DIM"]
        self.h1 = d["ML_H1_DIM"]
        self.out_dim = d["ML_OUT_DIM"]
        self.s_x = d["ML_S_X"]
        self.s_y2 = d["ML_S_Y2"]
        self.mult1, self.shift1 = d["ML_MULT1"], d["ML_SHIFT1"]
        self.mult2, self.shift2 = d["ML_MULT2"], d["ML_SHIFT2"]

        self.mean = parse_array(c, "ML_FEAT_MEAN")
        self.inv_std = parse_array(c, "ML_FEAT_INV_STD")
        w1 = parse_array(c, "ML_W1")
        w2 = parse_array(c, "ML_W2")
        self.b1 = parse_array(c, "ML_B1")
        self.b2 = parse_array(c, "ML_B2")

        if transpose_w:
            # 故意读错：模拟"导出/读取时行列颠倒"
            w1 = w1.reshape(self.h1, self.in_dim).T.ravel()
            w2 = w2.reshape(self.out_dim, self.h1).T.ravel()
        self.w1 = w1.reshape(self.h1, self.in_dim)
        self.w2 = w2.reshape(self.out_dim, self.h1)

    def quantize(self, feat):
        z = (feat - self.mean) * self.inv_std
        q = np.floor(z / self.s_x + 0.5)
        return np.clip(q, -128, 127).astype(np.int64)

    def run_q(self, xq):
        """输入已是 int8 量化值，返回 int8 logit（列序 ADL, NEAR_FALL, FALL）"""
        a1 = xq @ self.w1.T + self.b1
        hq = np.clip((a1 * self.mult1 + (1 << (self.shift1 - 1))) >> self.shift1, 0, 127)
        a2 = hq @ self.w2.T + self.b2
        oq = np.clip((a2 * self.mult2 + (1 << (self.shift2 - 1))) >> self.shift2, -128, 127)
        return oq

    def predict(self, feat):
        return np.argmax(self.run_q(self.quantize(feat)), axis=1)


# ---------------------------------------------------------------------------
def main():
    ok = True
    print("=" * 72)
    print("从导出的 C 代码反向验证 TinyML 链路")
    print("=" * 72)

    report = json.load(open(os.path.join(OUT_DIR, "report.json"), encoding="utf-8"))
    model = Int8Model(ML_DIR)

    print("\n[1] 从 C 源码解析出的量化参数")
    print("    维度 %d -> %d -> %d" % (model.in_dim, model.h1, model.out_dim))
    print("    s_x=%.9g  mult1=%d shift1=%d" % (model.s_x, model.mult1, model.shift1))
    print("    s_y2=%.9g  mult2=%d shift2=%d" % (model.s_y2, model.mult2, model.shift2))
    for nm, arr, want in (("ML_W1", model.w1, model.h1 * model.in_dim),
                          ("ML_W2", model.w2, model.out_dim * model.h1),
                          ("ML_B1", model.b1, model.h1),
                          ("ML_B2", model.b2, model.out_dim)):
        if arr.size != want:
            print("    ✗ %s 元素个数 %d，期望 %d" % (nm, arr.size, want))
            ok = False
    if ok:
        print("    ✓ 各数组长度与声明的维度一致")

    # ---- 复现测试集 -------------------------------------------------------
    print("\n[2] 重建测试集并复现指标")
    Xte, yte = make_dataset(seed=TEST_SEED, n_per_class=N_TEST_PER_CLASS,
                            subject_scale=(0.85, 1.18),
                            acc_sigma=0.035, gyro_sigma=7.0)
    F = extract_dataset(Xte)
    pred = model.predict(F)
    acc = float((pred == yte).mean())
    want_acc = report["精度"]["int8测试"]
    print("    从 C 数组重算准确率 : %.4f" % acc)
    print("    report.json 记录值  : %.4f" % want_acc)
    if abs(acc - want_acc) > 1e-9:
        print("    ✗ 不一致 —— 导出环节有问题（最常见：权重行列转置、特征顺序错位）")
        ok = False
    else:
        print("    ✓ 完全一致，导出无损")

    fall_rec = float((pred[yte == FALL] == FALL).mean())
    fa = float((pred[yte != FALL] == FALL).mean())
    print("    跌倒召回 %.4f   非跌倒误报 %.4f" % (fall_rec, fa))

    # ---- 转置鲁棒性：证明本验证不是走过场 ---------------------------------
    print("\n[3] 反例检验：故意把权重转置，看指标是否崩")
    bad = Int8Model(ML_DIR, transpose_w=True)
    bad_acc = float((bad.predict(F) == yte).mean())
    print("    转置后准确率 : %.4f  （原始 %.4f）" % (bad_acc, acc))
    if bad_acc >= acc - 0.02:
        print("    ✗ 转置后指标几乎没变 —— 说明本验证对行列错误不敏感，等于没验")
        ok = False
    else:
        print("    ✓ 指标显著下降，证明本验证确实能抓住行列转置这类错误")

    # ---- float32 / float64 差异风险 ---------------------------------------
    print("\n[4] C(float) 与 Python(double) 的舍入风险")
    F32 = F.astype(np.float32).astype(np.float64)
    pred32 = model.predict(F32)
    changed = int((pred32 != pred).sum())
    print("    仅把特征舍入到 float32 后，类别改变 %d / %d 个窗口" % (changed, len(pred)))
    q_a = model.quantize(F)
    q_b = model.quantize(F32)
    lsb = np.abs(q_a - q_b)
    print("    量化值差异：最大 %d LSB，受影响元素 %.4f%%"
          % (lsb.max(), 100.0 * (lsb > 0).mean()))
    if changed > len(pred) * 0.01:
        print("    ✗ 类别翻转比例 >1%%，板子上结果可能与 PC 明显不同，需要改用定点特征")
        ok = False
    else:
        print("    ✓ 风险可接受：C 端 float 与 Python 端 double 的差异不会实质改变结论")

    # ---- 黄金向量自洽性（等价于在 PC 上先跑一遍 ml_selftest） --------------
    print("\n[5] 黄金向量自洽性（等价于提前跑一遍板上的 ML_SelfTest）")
    gz = np.load(os.path.join(OUT_DIR, "golden.npz"))
    gf, go, gcls = gz["feat"], gz["out_q"], gz["cls"]
    # 固件目录里那份必须与出参完全同步，否则板上自测会误报
    hdr_fw = open(os.path.join(ML_DIR, "ml_golden.h"), encoding="utf-8").read()
    hdr_out = open(os.path.join(OUT_DIR, "ml_golden.h"), encoding="utf-8").read()
    if hdr_fw != hdr_out:
        print("    ✗ icode/ml/ml_golden.h 与 out/ml_golden.h 不一致（固件里是旧版本）")
        ok = False
    else:
        print("    ✓ 固件目录与 out 目录的 ml_golden.h 同步")

    print("    黄金向量 %d 条，维度 %d" % (len(gf), gf.shape[1]))
    gq = model.run_q(model.quantize(gf))
    exact = int((gq == go).all(axis=1).sum())
    gp_pred = model.predict(gf)
    cls_ok = int((gp_pred == gcls).sum())
    maxdiff = int(np.abs(gq - go).max())
    print("    int8 输出逐位一致 %d/%d 条，类别一致 %d/%d 条，最大偏差 %d LSB"
          % (exact, len(gf), cls_ok, len(gf), maxdiff))
    if exact != len(gf) or cls_ok != len(gf):
        print("    ✗ 黄金向量不自洽 —— ml_golden.h 不是最新一次训练的产物。")
        print("      请重跑 tools/tinyml/train_quantize.py。")
        ok = False
    else:
        print("    ✓ 自洽。板上 ML_SelfTest 应当全绿；若板上有偏差，问题必在 ml_features.c")

    print("\n" + "=" * 72)
    print("结论：" + ("全部通过 ✓" if ok else "存在失败项 ✗"))
    print("=" * 72)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
