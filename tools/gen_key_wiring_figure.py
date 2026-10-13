# -*- coding: utf-8 -*-
"""
gen_key_wiring_figure.py —— 生成"四脚轻触开关接线图"（docs/按键四脚接线图.png）

为什么需要它：四脚轻触开关最常踩的坑就是把"同侧两脚"当成两个独立端子，
一焊就变成常闭（永远显示按下）。这张图把内部等效、两种接法与验证方法画清楚。
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import FancyBboxPatch, Circle
from matplotlib.lines import Line2D

# ---- 中文字体（找不到就用英文标签兜底）----
CJK = None
for cand in (r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\msyhbd.ttc",
             r"C:\Windows\Fonts\simhei.ttf", r"C:\Windows\Fonts\simsun.ttc"):
    if os.path.exists(cand):
        font_manager.fontManager.addfont(cand)
        CJK = font_manager.FontProperties(fname=cand).get_name()
        break
if CJK:
    plt.rcParams["font.family"] = CJK
plt.rcParams["axes.unicode_minus"] = False


def label(zh, en):
    return zh if CJK else en


fig, axes = plt.subplots(1, 3, figsize=(15.5, 5.6))
for ax in axes:
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.set_aspect("equal")
    ax.axis("off")

# ===========================================================================
# 面板 A：内部等效
# ===========================================================================
ax = axes[0]
ax.set_title(label("① 四脚开关内部等效（俯视）", "1) Internal equivalent (top view)"),
             fontsize=13, pad=12)

# 外壳
ax.add_patch(FancyBboxPatch((2.6, 3.4), 4.8, 4.8, boxstyle="round,pad=0.15",
                            linewidth=1.6, edgecolor="#333", facecolor="#f5f5f5"))
# 四个脚（左两只、右两只）
for y, ty in ((4.2, "3.6"), (7.4, "7.4")):
    ax.add_patch(FancyBboxPatch((1.5, y - 0.22), 1.1, 0.44, linewidth=1.2,
                                edgecolor="#444", facecolor="#cfd8dc"))
    ax.add_patch(FancyBboxPatch((7.4, y - 0.22), 1.1, 0.44, linewidth=1.2,
                                edgecolor="#444", facecolor="#cfd8dc"))

# 内部：同侧连通
ax.add_line(Line2D([2.0, 4.0], [4.2, 4.2], color="#d32f2f", lw=2.4))
ax.add_line(Line2D([2.0, 4.0], [7.4, 7.4], color="#d32f2f", lw=2.4))
ax.add_line(Line2D([4.0, 4.0], [4.2, 7.4], color="#d32f2f", lw=2.4))
ax.add_line(Line2D([6.0, 8.0], [4.2, 4.2], color="#2e7d32", lw=2.4))
ax.add_line(Line2D([6.0, 8.0], [7.4, 7.4], color="#2e7d32", lw=2.4))
ax.add_line(Line2D([6.0, 6.0], [4.2, 7.4], color="#2e7d32", lw=2.4))

# 开关符号（按下才连通）
ax.plot([4.0, 5.0], [5.8, 5.8], color="#333", lw=2.6)
ax.plot([5.0, 6.0], [5.8, 6.9], color="#333", lw=2.6)   # 断开的触点
ax.plot([5.4, 5.4], [5.2, 6.6], color="#333", lw=0)
ax.add_patch(Circle((6.0, 5.8), 0.16, color="#333"))
ax.text(5.0, 4.9, label("按下才导通", "closed when pressed"), ha="center",
        fontsize=10.5, color="#333")

ax.text(2.4, 2.6, label("同侧两脚是同一个点", "same-side legs = one node"),
        ha="center", fontsize=11, color="#d32f2f")
ax.text(7.8, 2.6, label("另一侧 = 另一端", "other side = other node"),
        ha="center", fontsize=11, color="#2e7d32")
ax.text(5.0, 9.3, label("4 只脚 ≠ 4 个独立端子", "4 legs are NOT 4 terminals"),
        ha="center", fontsize=12, fontweight="bold", color="#b71c1c")
ax.text(5.0, 0.7, label("不按时：同侧两脚通（响），两侧之间不通", "not pressed: same-side shorted, across open"),
        ha="center", fontsize=10, color="#555")

# ===========================================================================
# 面板 B：接法一（只用对角两脚）
# ===========================================================================
ax = axes[1]
ax.set_title(label("② 接法一：只用对角两脚（最省事）", "2) Option A: two diagonal legs"),
             fontsize=13, pad=12)
ax.add_patch(FancyBboxPatch((3.0, 3.6), 4.0, 4.0, boxstyle="round,pad=0.15",
                            linewidth=1.6, edgecolor="#333", facecolor="#f5f5f5"))
# 左上脚 -> MCU   右下脚 -> GND
ax.add_patch(FancyBboxPatch((1.9, 6.8), 1.1, 0.44, linewidth=1.2,
                            edgecolor="#444", facecolor="#ffcdd2"))
ax.add_patch(FancyBboxPatch((7.0, 4.0), 1.1, 0.44, linewidth=1.2,
                            edgecolor="#444", facecolor="#c8e6c9"))
ax.add_patch(FancyBboxPatch((1.9, 4.0), 1.1, 0.44, linewidth=1.2,
                            edgecolor="#bbb", facecolor="#eeeeee"))
ax.add_patch(FancyBboxPatch((7.0, 6.8), 1.1, 0.44, linewidth=1.2,
                            edgecolor="#bbb", facecolor="#eeeeee"))

ax.add_line(Line2D([1.9, 0.4], [7.0, 7.0], color="#d32f2f", lw=2.4))
ax.add_line(Line2D([0.4, 0.4], [7.0, 8.6], color="#d32f2f", lw=2.4))
ax.text(0.35, 8.85, label("MCU 引脚\nPA0 / PA1 / PA11 / PA12", "MCU pin"),
        ha="left", fontsize=10.5, color="#b71c1c")

ax.add_line(Line2D([8.1, 9.6], [4.2, 4.2], color="#2e7d32", lw=2.4))
ax.add_line(Line2D([9.6, 9.6], [4.2, 2.6], color="#2e7d32", lw=2.4))
ax.text(9.55, 2.1, label("GND", "GND"), ha="right", fontsize=11.5, color="#1b5e20")

ax.text(5.0, 9.2, label("对角两脚一进一出", "diagonal legs: in and out"),
        ha="center", fontsize=12, fontweight="bold", color="#0d47a1")
ax.text(5.0, 1.4, label("灰色两脚可不接（只做机械固定）", "grey legs: optional (mechanical only)"),
        ha="center", fontsize=10, color="#555")

# ===========================================================================
# 面板 C：接法二（四脚都用，同侧并接）
# ===========================================================================
ax = axes[2]
ax.set_title(label("③ 接法二：四脚都用（焊得牢，推荐）", "3) Option B: all 4 legs"),
             fontsize=13, pad=12)
ax.add_patch(FancyBboxPatch((3.0, 3.6), 4.0, 4.0, boxstyle="round,pad=0.15",
                            linewidth=1.6, edgecolor="#333", facecolor="#f5f5f5"))
ax.add_patch(FancyBboxPatch((1.9, 6.8), 1.1, 0.44, linewidth=1.2,
                            edgecolor="#444", facecolor="#ffcdd2"))
ax.add_patch(FancyBboxPatch((1.9, 4.0), 1.1, 0.44, linewidth=1.2,
                            edgecolor="#444", facecolor="#ffcdd2"))
ax.add_patch(FancyBboxPatch((7.0, 6.8), 1.1, 0.44, linewidth=1.2,
                            edgecolor="#444", facecolor="#c8e6c9"))
ax.add_patch(FancyBboxPatch((7.0, 4.0), 1.1, 0.44, linewidth=1.2,
                            edgecolor="#444", facecolor="#c8e6c9"))
# 左侧两脚并接
ax.add_line(Line2D([1.9, 1.3], [7.0, 7.0], color="#d32f2f", lw=2.4))
ax.add_line(Line2D([1.9, 1.3], [4.2, 4.2], color="#d32f2f", lw=2.4))
ax.add_line(Line2D([1.3, 1.3], [4.2, 7.0], color="#d32f2f", lw=2.4))
ax.add_line(Line2D([1.3, 0.4], [7.0, 8.6], color="#d32f2f", lw=2.4))
ax.text(0.35, 8.85, label("MCU 引脚", "MCU pin"), ha="left", fontsize=10.5, color="#b71c1c")
# 右侧两脚并接
ax.add_line(Line2D([8.1, 8.7], [7.0, 7.0], color="#2e7d32", lw=2.4))
ax.add_line(Line2D([8.1, 8.7], [4.2, 4.2], color="#2e7d32", lw=2.4))
ax.add_line(Line2D([8.7, 8.7], [4.2, 7.0], color="#2e7d32", lw=2.4))
ax.add_line(Line2D([8.7, 9.6], [4.2, 2.6], color="#2e7d32", lw=2.4))
ax.text(9.55, 2.1, label("GND", "GND"), ha="right", fontsize=11.5, color="#1b5e20")

ax.text(5.0, 9.2, label("同侧两脚先并起来", "bridge same-side legs first"),
        ha="center", fontsize=12, fontweight="bold", color="#0d47a1")
ax.text(5.0, 1.9, label("可选：并一只 100 nF 到 GND（硬件消抖）",
                        "optional: 100 nF across the two nodes"),
        ha="center", fontsize=10, color="#555")
ax.text(5.0, 0.9, label("★ 千万别把左右两侧焊桥连起来 → 会变成常闭（永远显示按下）",
                        "never bridge left to right -> permanently 'pressed'"),
        ha="center", fontsize=10.5, color="#b71c1c", fontweight="bold")

fig.suptitle(label("四脚轻触开关（6×6）接线：同侧两脚同点，对角两脚才是开关两端",
                   "4-leg tactile switch wiring"),
             fontsize=15, fontweight="bold", y=0.99)
fig.tight_layout(rect=[0, 0, 1, 0.95])

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                   "docs", "按键四脚接线图.png")
out = os.path.normpath(out)
os.makedirs(os.path.dirname(out), exist_ok=True)
fig.savefig(out, dpi=150, facecolor="white")
print("字体:", CJK if CJK else "英文兜底")
print("已生成:", out, os.path.getsize(out), "B")
