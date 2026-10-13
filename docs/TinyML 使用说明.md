# TinyML 使用说明（要求 2：MCU 端跌倒检测推理）

> 对应 `江彬锋毕设要求.txt` 第 5 行："TinyML 轻量级模型在 MCU 端运行跌倒检测推理"
> 状态：**已跑通**，全部指标为实测，验证记录见 §6

---

## 1. 先建立正确的心理模型

TinyML **不是装在单片机上的框架**。它是一条流水线，绝大部分工作不在 MCU 上做：

```
【PC 端 · 只做一次，慢没关系】                       【MCU 端 · 每次推理】
  合成/真实数据                                        int8 输入
        │                                                  │
        ▼                                            乘加(int8×int8→int32)
   29 维特征提取                                             │
        │                                              定点重缩放 → int8
        ▼                                                  │
   训练一个小网络 (float32)                               ReLU
        │                                                  │
        ▼                                              再乘加 → argmax
   训练后量化 (PTQ) → int8                                 │
        │                                             ADL / NEAR_FALL / FALL
        ▼
   导出 ml_weights.c/h  ← 一堆 int8 数字
```

**STM32 上从头到尾没有"AI 框架"这回事**，只有 `for` 循环做乘加。TensorFlow / sklearn
只在你笔记本上跑一次，产出物就是一个 `.h` 文件加一次 `for` 循环。

> 为什么不用 TensorFlow Lite Micro：官方运行时（内核+解释器+张量管理）在 Cortex-M3 上
> 就要 **60~100 KB Flash**，而 F103C8T6 只有 64 KB，现有工程已占 40 KB，只剩约 24 KB。
> **这是物理上装不下，不是能力问题。** 所以本工程用自写 int8 内核（约 2~3 KB，功能等价）。

**TinyML 的灵魂代码就这么长**（`ml_infer.c` 的核心）：

```c
for (o = 0; o < ML_H1_DIM; o++) {
    acc = ML_B1[o];
    for (i = 0; i < ML_IN_DIM; i++)
        acc += (int32_t)row[i] * (int32_t)in_q[i];      /* int8 乘，int32 累加 */
    wide = ((int64_t)acc * ML_MULT1 + (1LL << (ML_SHIFT1-1))) >> ML_SHIFT1;
    if (wide < 0) wide = 0; else if (wide > 127) wide = 127;   /* ReLU + 饱和 */
    hid[o] = (int8_t)wide;
}
```

量化就是把 `y = W·x + b` 改写成整数运算：

```
真实值 = s × 整数值       即  W = s_w·W_q,  x = s_x·x_q
y = s_w·s_x·(W_q·x_q) + b
    └─── int32 累加 ───┘
再乘定点数 (s_w·s_x / s_y) 变回 int8  ← 预先在 PC 上算成 MULT 和 SHIFT
```

`MULT`/`SHIFT` 是训练完在 PC 上算好的常数，MCU 上只做一次 64 位乘加和移位。
全整数运算 —— 这正是 **F103 没有 FPU 也能跑深度学习**的原因。

---

## 2. 怎么跑（一条命令）

```powershell
E:\anaconda3\python.exe tools\tinyml\train_quantize.py
```

**不需要下载数据集，不需要装 TensorFlow。** 用机器上现装的 numpy + sklearn，
约 18 秒跑完全流程，产出：

| 产出 | 位置 | 说明 |
|---|---|---|
| `ml_weights.c` / `ml_weights.h` | `Security Watch\MDK-ARM\icode\ml\` | int8 权重与量化参数 |
| `ml_golden.h` | `Security Watch\MDK-ARM\icode\ml\` | 上板自测黄金向量 |
| `report.json` | `tools\tinyml\out\` | 全部指标，写论文直接从这取 |
| `confusion_matrix.txt` | `tools\tinyml\out\` | 混淆矩阵 |
| `run.log` | `tools\tinyml\out\` | 完整运行日志 |

改模型结构/重新训练后，**必须再跑一次验证**：

```powershell
E:\anaconda3\python.exe tools\tinyml\verify_quant.py     # 退出码 0 = 通过
```

如果新增了文件或要在新机器上重建工程登记：

```powershell
E:\anaconda3\python.exe tools\tinyml\add_ml_to_keil.py   # 幂等，自动备份 uvprojx
```

---

## 3. 文件地图

```
tools/tinyml/
├── fallsim.py            合成数据生成（生物力学三阶段跌倒模型）
├── features.py           29 维特征提取 ★Python 参考实现
├── train_quantize.py     训练 + int8 量化 + 导出 C 源码
├── verify_quant.py       从导出的 C 代码反向验证
├── add_ml_to_keil.py     登记进 Keil 工程（幂等）
└── out/                  报告、混淆矩阵、黄金向量、armcc 目标文件

Security Watch/MDK-ARM/icode/ml/
├── ml_features.c/h       29 维特征提取 ★C 实现（必须与 features.py 一致）
├── ml_infer.c/h          int8 推理内核（自写，无第三方依赖）
├── ml_weights.c/h        权重（自动生成，请勿手改）
├── ml_selftest.c/h       上板自测
└── ml_golden.h           黄金向量（自动生成）

Security Watch/MDK-ARM/icode/task/
└── tinyml_task.c/h       ★推理任务：环形缓冲 + 两阶段判定
```

**⚠️ 最关键的一致性约束**：`features.py` 与 `ml_features.c` 是**同一套算法的两份实现**。
特征顺序错一位，板子上的结果会全错，而且**不报任何错**。改动任何一边，必须同步另一边
并重跑 `verify_quant.py`。

---

## 4. 实测指标（合成数据集，全部可复现）

### 4.1 模型与精度

| 项目 | 数值 |
|---|---|
| 结构 | 29 → 16 (ReLU) → 3 (softmax) |
| 参数量 | 531（其中 int8 权重 512 个） |
| float32 测试准确率 | **0.9767** |
| int8 量化后测试准确率 | **0.9758**（量化损失仅 0.0008） |
| 跌倒召回 / 非跌倒误报 | **1.0000 / 0.0000** |
| 分布外压力测试最差值 | 召回 0.9950 / 误报 0.0100 |

> 测试集刻意换了随机种子、把受试者尺度放宽到 0.85~1.18、噪声加大，
> 用来模拟"换了个人、换了台设备"，避免指标虚高。

### 4.2 两阶段判定的收益（这是论文里最有说服力的一节）

| 预筛门限 | 日常动作触发率 | 跌倒触发率 | 评价 |
|---|---|---|---|
| 1.5 g | 0.4225 | 1.0000 | 省算力但误报风险高 |
| 2.0 g | 0.2525 | 1.0000 | |
| **2.5 g** | **0.2325** | **1.0000** | **本工程取值** |
| 3.0 g | 0.1800 | 1.0000 | |
| 4.0 g | 0.0300 | **0.9725** | 开始漏检 |

**结论**：门限 2.5 g 时，76.75% 的日常动作连特征都不提取；而跌倒 100% 会被送进网络。
门限抬到 4 g 虽然更省算力，但跌倒触发率掉到 97.25% —— **被预筛漏掉的跌倒，
网络再好也救不回来**。这就是 2.5 g 这个取值的依据。

### 4.3 目标码体积（armcc 实测，Cortex-M3 -O1）

| 文件 | Code | RO Data | Flash 占用 |
|---|---|---|---|
| `ml_weights.c` | 0 | 820 | 820 |
| `ml_features.c` | 1816 | 0 | 1816 |
| `ml_infer.c` | 588 | 0 | 588 |
| `ml_selftest.c` | 248 | 960 | 1208 |
| **合计** | | | **4432 字节 (4.33 KB)** |
| RAM | | | 环形缓冲 2412 B + 任务栈约 500 B |

对比预算（Flash 余量 23.9 KB / RAM 余量 15.2 KB），**余量充足**。
`ml_selftest.c` 那 1208 字节可通过 `ml_selftest.h` 里 `ML_ENABLE_SELFTEST` 改 0 省掉。
数学库（`sqrtf`/`acosf`/`expf`）若为整工程首次引入，需在 Keil 编译后的 map 里复核。

---

## 5. 上板后怎么确认它是对的

这是**新手最容易忽略、也最救命**的一步。模型"PC 上 98%、板子上乱跳"通常只有一个原因：
C 端与 Python 端的量化/归一化语义有细微不一致（移位前忘了加四舍五入、特征顺序错位、
权重行列颠倒）。这类错误不报编译错、不崩，只会让结果悄悄变差，极难排查。

`ml_selftest.c` 把 PC 端算好的 `(原始特征 → int8 输出 → 类别)` 烧进 Flash，
`tinyml_task` 初始化时自动跑一遍：

```c
ML_SelfTestResult_t r;
uint8_t ok = ML_SelfTest(&r);     /* 已接在 TinyML_Task_Entry 开头 */
/* r.total / r.class_ok / r.exact_out / r.max_lsb_diff 可通过 OLED 或串口打印 */
```

| 结果 | 含义 |
|---|---|
| 类别全对且 `max_lsb_diff ≤ 1` | 归一化 / 量化 / 推理链路与 PC 完全等价，可放心使用 |
| 类别的对不上 | **问题必在 `ml_infer.c`**，不用去猜特征提取 |
| 全对但线上结果仍差 | **问题必在 `ml_features.c`**（与 `features.py` 不一致） |

---

## 6. 验证记录（本次实测，可复现）

`verify_quant.py` 的输出，全部通过：

```
[1] 从 C 源码解析出的量化参数 —— 各数组长度与声明的维度一致        ✓
[2] 从 C 数组重算准确率 0.9758 == report.json 记录值 0.9758        ✓ 导出无损
[3] 反例检验：故意把权重转置 → 准确率从 0.9758 崩到 0.4175          ✓ 验证有效
[4] 特征舍入到 float32 后：类别改变 0/1200，量化值差异 0 LSB        ✓ 精度风险可忽略
[5] 黄金向量 8/8 逐位一致、8/8 类别一致，固件目录与 out 目录同步     ✓ 自洽
```

第 3 项特别重要：它证明这个验证脚本**确实能抓住"权重行列颠倒"这类错误**，
而不是走个过场。本次开发中它还真的抓到了一个类似问题 ——
权重数组被生成成 `static`，而头文件声明为 `extern`，上板会在链接阶段报
undefined symbol，编译期完全看不出来。

**工具链限制说明（如实记录）**：本机没有 x86 的 C 编译器（无 gcc/clang/cl），
因此**没有在 PC 上直接运行过 C 推理代码**。C 端是通过以下三条路径验证的：

1. 用 `armcc`（`E:\Keil5\ARM\ARMCC`）按 Cortex-M3 / C99 / -O1 编译全部源文件，
   零警告零错误；并用 `fromelf` 检查符号表确认 `ML_W1` 等全部是 **Global 绑定**；
2. 用 numpy **位精确模拟** C 的整数运算（含 `>>` 对负数的算术移位语义），
   复现量化前后的精度指标；
3. 生成黄金向量，供上板时做真机比对。

第 3 条是把"PC 验证"补完的最后一环 —— **上板跑一次 `ML_SelfTest` 就能确认**。

---

## 7. 接上真实数据前，还差什么

`tinyml_task` 的数据入口已经做好了，但**还没有人来喂它**：

```c
/* 在 imu_recv_task.c 里，每解析出一帧 32 字节 IMU 数据帧后，加这一行： */
int16_t frame[ML_CH] = { ax_mg, ay_mg, az_mg, gx_dps10, gy_dps10, gz_dps10 };
TinyML_PushFrame(frame);
```

`imu_recv_task.c` 目前还是空壳（只有 USER CODE 标记），需要先完成：
串口 DMA + 空闲中断接收 → 环形缓冲 → 32 字节帧解析 + CRC16 校验。
这是 `00-方案与计划.md` 里 P0 阶段的工作。

接线与协议见 `00-方案与计划.md` §2.2 / §2.3。

---

## 8. 换成真实数据集（想升级论文成色时再做）

当前用合成数据的理由是**时间优先**。要换成真实数据，改一处即可：

```python
# train_quantize.py 的 main() 里，把两行 make_dataset 换成：
Xtr, ytr = load_sisfall("data/processed/train.npz")   # 需自己写解析脚本
Xte, yte = load_sisfall("data/processed/test.npz")
```

其余流程（特征提取 → 训练 → 量化 → 导出 → 验证）**完全不用改**，
因为特征提取是独立模块。

必须遵守的两条（评审必问）：

1. **按受试者划分训练/测试集**（leave-subject-out）。按时间随机划分会让同一受试者的
   相邻窗口同时进训练和测试集，指标虚高 —— 这是跌倒检测论文最常见的扣分点。
2. **统一重采样到 100 Hz**，并做去重力（Butterworth 0.3 Hz 高通）与归一化。

推荐数据集：**SisFall**（200 Hz、腰部、19 种 ADL + 15 种跌倒，该领域标准基线）。

---

## 9. 论文里可以怎么用这些材料

| 章节 | 现成素材 |
|---|---|
| 算法设计 | 生物力学三阶段模型（自由落体 → 冲击 → 冲击后静止）与 29 维特征的物理含义 |
| 模型压缩 | float32 vs int8 精度对比（0.9767 → 0.9758，损失 0.0008） |
| 部署方案 | 为什么排除 TFLite Micro（60~100 KB vs 24 KB 余量）；自写内核 vs Cube.AI |
| 系统优化 | 两阶段判定的门限扫描表 —— 算力与漏检率的定量权衡 |
| 资源分析 | Flash 4.33 KB / RAM 2.4 KB 实测表，armcc 目标码分解 |
| 鲁棒性 | 分布外压力测试（尺度 0.70~1.40、噪声加倍）与二分类指标 |
| 工程可靠性 | 黄金向量上板自测方案；"静态/外部链接不一致"这类真实缺陷的排查记录 |
| 局限性 | 合成噪声比真实 IMU（量化噪声、温漂、佩戴松动）简单，需自采数据复核 |

**建议如实写明的局限性**：当前模型在合成数据上误报点估计为 0，这是合成数据比真实
数据"干净"的必然结果，**不能当作真实误报率**。报告中给出的 0.0075 是按规则三
（n=400 中 0 次事件 → 95% 置信上界约 3/n）算出的保守上界，论文里应当用上界而非点估计。

---

## 10. 常见故障速查

| 现象 | 最可能的原因 | 怎么查 |
|---|---|---|
| 板子上结果全乱 | `ml_features.c` 与 `features.py` 特征顺序不一致 | 跑 `ML_SelfTest`；类别全对说明特征提取有问题 |
| 链接报 undefined symbol | 权重数组被生成为 `static` | 检查 `ml_weights.c` 里数组定义有无 `static` |
| 编译报找不到 `ml_infer.h` | 工程 IncludePath 缺 `./icode/ml` | 跑 `add_ml_to_keil.py` |
| 编译报找不到 `ml_golden.h` | 还没跑过 `train_quantize.py` | 先跑训练脚本 |
| 任务跑不起来、无任何输出 | 栈溢出或 FreeRTOS 堆不足 | `TINYML_TASK_STACK` 已设 384 word；`configASSERT` 会死循环，建议改成点灯 |
| 推理结果与 PC 差 1 LSB | C 用 float、Python 用 double 的舍入差 | 属正常，类别一致即可 |
| 改完模型指标没变 | 只跑了训练没跑验证 | 必须跑 `verify_quant.py`，退出码 0 才算通过 |
