# DIY IMU（节点A）接入说明

> 适用工程：`D:\同学们的毕设\江彬锋的毕设\Security Watch`
> 对应固件：节点A = `D:\STM32Project\ICM42688+MMC5603 9 axis gyroscope`
> 帧协议权威定义：节点A工程的 `docs/串口帧协议.md`

---

## 1. 为什么用 USART3 而不是 USART2

任务最初要求"用 uart2 接收 DIY IMU 数据"，但工程现状不允许：

| 串口 | 引脚 | 波特率 | 用途 | 本工程能否用于 IMU |
|---|---|---|---|---|
| USART1 | PB6/PB7（重映射） | 9600 | GPS（DMA1_CH5 循环） | ❌ 已占用 |
| **USART2** | **PA2/PA3** | **9600** | **HC-08 BLE（中断接收）** | ❌ 已占用，**且 DMA1_CH6 给了 WS2812**，拿不到 DMA |
| **USART3** | **PB10/PB11** | **115200** | **"Node A IMU frame link"（DMA1_CH3 循环）** | ✅ **已配好，正合用** |

`Core/Src/usart.c` 里 USART3 的注释本来就写着
`Node A (sensor board) IMU frame link: PB10/PB11, 115200 8N1, DMA circular Rx`
—— 说明这个链路早就为节点A预留并配置好了。

**故采用 USART3。**

### 接线

```
节点A (ICM42688板)          Security Watch 板
   PA2 (USART2_TX)  ──────▶  PB11 (USART3_RX)
   PA3 (USART2_RX)  ◀──────  PB10 (USART3_TX)    可选，命令回传
   GND              ───────  GND                 ★ 必须共地
```

⚠️ 节点A的 USART2 已改为 **115200**，两边波特率必须一致。

---

## 2. 数据流与分层

```
节点A 100Hz 发帧
      │  115200 8N1
      ▼
USART3 ──DMA(Circular)──▶ s_dma_buf[128]
      │  空闲中断
      ▼
HAL_UARTEx_RxEventCallback (main.c, USER CODE BEGIN 4)
      │  按句柄分发
      ▼
DIY_IMU_RxEventCallback(huart, Size)     ← 中断上下文，只搬字节
      │  按绝对位置算增量
      ▼
s_byte_ring[256]                          ← 中断→任务解耦
      │
      ▼
DIY_IMU_Process()                         ← 任务上下文
      │  找 0xAA55 → 收满 32B → 校验 CRC16 → 整数换算
      ▼
s_frame_ring[8]  (DIY_IMU_Frame_t)
      │
      ▼
IMU_Recv_Task_Entry() 每 5ms
      │  取帧 → 组 6 通道 int16
      ▼
TinyML_PushFrame(frame)  → tinyml_task 的滑动窗口
```

**分层理由**：中断里绝不做解析（CRC + 换算在任务里做），避免长时间占用中断。

---

## 3. 单位换算是本驱动的核心职责

节点A的帧里发的是**原始 ADC 计数**（零量化损失），但本工程 TinyML 与疲劳判定
约定的是工程量（见 `ml_features.h` 的输入约定）：

```
acc  : mg      （±16 g 量程，1 g = 1000 mg）
gyro : 0.1 °/s （±2000 °/s 量程，1 °/s = 10 LSB）
```

两者可以**精确整数换算，无浮点、无舍入误差**：

| 目标 | 推导 | 实现 |
|---|---|---|
| mg | `raw × 16000/32768 = raw × 125/256` | `(raw * 125) >> 8` |
| 0.1 °/s | `raw × 20000/32768 = raw × 625/1024` | `(raw * 625) >> 10` |

推导依据（与节点A驱动一致）：
- `ACCEL_CONFIG0 = 0x07` → ±16 g，32768 计数对应 16000 mg
- `GYRO_CONFIG0 = 0x07` → ±2000 °/s，32768 计数对应 20000 个 0.1°/s

量程上限检查：16000 mg 与 20000 LSB 都 < int16 上限 32767，**不会溢出**。

> 用整数移位而不是浮点乘，是因为 F103 无 FPU，且整数运算是精确的 ——
> 浮点乘 `2000.0f/32768.0f` 再取整会引入 1 LSB 级舍入，而这里不需要付这个代价。

---

## 4. RAM 约束 —— 这次接入最棘手的地方

STM32F103C8 只有 **20 KB RAM**，而这个工程本来就贴着上限。

### 实测账本（来自 armlink map）

| 项 | 字节 |
|---|---|
| RAM 总量 | 20480 |
| 启动栈预留（`Stack_Size = 0x400`） | −1024 |
| **可用** | **19456** |
| FreeRTOS 堆（`configTOTAL_HEAP_SIZE`） | 10240 |
| `tinyml_task.o`（ML 环形缓冲 2412） | 2584 |
| `tasks.o` + `cmsis_os2.o` | 2428 |
| `oled.o`（显存） | 1024 |
| `gps.o` | 584 |
| 其余（usart/tim/adc/rtc…） | ~2600 |

**未加 DIY IMU 前已用约 19400 B，几乎顶满。**

### 首次接入直接链接失败

`diy_imu.o` 最初是 2692 B（DMA 512 + 字节环 1024 + 帧环 16×36 = 2608，
加统计量等），总量冲到 **22007 B，超出 2551 B**，报 28 个 `L6406E: No space`。

### 两步解决

**① 压缩自己的缓冲：2692 → 678 B（省 2014 B）**

| 缓冲 | 原 | 现 | 依据 |
|---|---|---|---|
| `DIY_DMA_BUF_SIZE` | 512 | **128** | 对齐 GPS 的做法；128 B 装得下 4 帧，空闲中断足够频繁 |
| `DIY_BYTE_RING_SIZE` | 1024 | **256** | 任务 5 ms 排空一次，115200 下约 58 B/5ms，256 B 有 4 倍余量 |
| `DIY_IMU_FRAME_RING` | 16 | **8** | 8 帧 = 80 ms @100Hz，足够吸收 TinyML 单次推理停顿 |

**② 发现关键性质：`configTOTAL_HEAP_SIZE` 是固定数组，少建任务不省 RAM**

最初我以为"暂缓一个任务就能腾出 RAM"，**这是错的**。
`configTOTAL_HEAP_SIZE` 是 `heap_4.c` 里一个固定的 `.bss` 数组：

```c
static uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];   /* 不管装几个任务都占这么多 */
```

所以**少建任务只是让数组内部空着，对外占用的 RAM 一点没变**。
要真正省 RAM，必须**直接缩小这个数组本身**。

实测需求（heap_4：每次分配 = 8 B 块头 + 大小按 8 B 对齐）：

| 项 | 字节 |
|---|---|
| Tmr Svc 栈（64 word） | 256 |
| Tmr Svc TCB | 96 |
| IDLE 栈（128 word） | 512 |
| IDLE TCB | 96 |
| 7 个应用任务栈合计（IMU128/TinyML384/Fatigue256/Alarm256/BLE256/GPS128/OLED128） | 6144 |
| 7 个应用任务 TCB 合计 | 672 |
| 定时器队列（10 项） | 80 |
| **合计** | **7856** |

**含 GPS 任务时需要 9080 B。** 故 `configTOTAL_HEAP_SIZE` 由 10240 降到 **9280**（省 960 B），
> 后续更新（2026-10-14）：接入 HC-08 后静态 RAM 吃紧，本值又降到 **8192**（并删除了无调用者的
> `ws2812b` 驱动）；当前有效值以 `Core/Inc/FreeRTOSConfig.h` 为准。
对"含 GPS"与"暂缓 GPS"两种配置都够用。

**③ 暂缓 GPS 任务**

即使堆降到 9280，仍需再省 648 B。既然 GPS 尚未接线、且与跌倒检测无强关联，
**暂缓创建 GPS 任务** —— 但**保留 `gps.c` 全部代码**，只把一行 `xTaskCreate` 注释掉：

```c
/* GPS_Task_Init(GPS_TASK_PRIO, GPS_TASK_STACK); */
```

- **恢复方法**：删掉注释即可，没有任何代码被删除
- **恢复前必须先腾出约 640 B RAM**，可选方式见本节末尾的"若要恢复 GPS"

### 最终结果

```
RW-data   264
ZI-data  18880
合计     19144 / 20480   → 余量 312 B  ✔
```

⚠️ **以后再加任务或缓冲，必须先重算这份账。** 相关说明也写进了
`Core/Src/freertos.c` 与 `FreeRTOSConfig.h` 的注释里，防止有人"顺手改回去"又爆 RAM。

### 若要恢复 GPS（约需再腾 640 B）

按风险从低到高：

| 方式 | 省 | 代价 |
|---|---|---|
| `IMU_RECV_TASK_STACK` 128 → 96 word | 128 B | 该任务调用链已很浅，但余量变小 |
| `FATIGUE` / `ALARM` / `BLE` 各 256 → 192 word | 768 B | 需实测栈高水位（`uxTaskGetStackHighWaterMark`）确认 |
| `configMINIMAL_STACK_SIZE` 128 → 96（影响 IDLE） | 128 B | IDLE 里若加 hook 要重新评估 |
| 缩 TinyML 窗口 `ML_WIN` 200 → 128 | 864 B | **必须重跑 Python 特征对齐并重训模型** |

**推荐组合**：前两项（896 B）即可恢复 GPS，且不必动模型。

## 5. 节点A的实际帧率（✅ 已于 2026-10-14 修复为严格 100 Hz）

`ml_features.h` 的窗口是 **`ML_WIN = 200` 且按 100 Hz 设计（= 2 s）**。
**这个 100 Hz 是硬约束**：29 维特征里有三类直接依赖采样间隔——

| 特征 | 为什么受影响 |
|---|---|
| `daz_max`（差分加速度 / jerk 量级） | 除以采样间隔再乘 `FS`，FS 错了量级就错 |
| `az_zcr`（过零率） | 定义是"每单位时间的过零次数"，时间基准变了就全错 |
| `tail_am_std` / `head_am_std` | 头尾各 0.5 s 的切片位置会错位 |

**曾经的实测**：节点A帧率只有约 **42 Hz**（发送节拍挂在主循环上，每 2 圈发一帧；
主循环受 `MMC5603_ReadData_Single()` 的 10 ms 阻塞与 OLED 软 I2C 刷屏拖慢，一轮约 12 ms）。
那时 200 帧窗口实际跨度约 **4.76 s**，上述特征全部错位。

**修复（节点A 侧，commit `ba23cad`）**：把发帧从主循环搬到 **TIM2 中断**
（200 Hz 解算二分频 → 严格 10 ms 一帧），`yaw_display` 归零计算一并搬到打包前，
主循环不再参与发帧 —— 于是磁力计阻塞与刷屏都不再影响帧间隔。
`Proto_SendImu()` 本身是非阻塞设计（`HAL_UART_Transmit_DMA`，DMA 忙则丢帧并计入
`stat_dropped`），可安全用于中断上下文。**节点B 的 ML 侧零改动**，模型与数据集继续有效。

**为什么选择改节点A而不是改 ML**：
1. 改 ML（把 `FS`/`ML_WIN` 改成 42 Hz 一套）**只治标**：42 Hz 下 24 ms 的采样间隔容易
   "削掉"20~50 ms 的触地冲击尖峰（甚至漏过 2.5 g 预筛），而且主循环发送的帧间隔**不匀**
   （Jitter），差分特征照样被污染；
2. 改 ML 还要同步改 Python 3 个脚本 + C 端 2 个文件 + **重新训练导出权重**，
   而合成数据按 42 Hz 重采样与"真实 42 Hz 间隔不匀采样"并不等价，等于把误差写进模型；
3. 改节点A只需一处（约 10 行），采样严格等间隔，物理上更接近训练数据。

**验证方法（上板必做）**：烧录节点A新固件后，在节点B 把 `oled_task.c` 的
`OLED_SCREEN_SEL` 设为 `OLED_SCREEN_IMU`，看第一行 **`IMU xxxHz`**：
应显示 **100**（±1 以内）。若仍是 4x Hz，说明节点A 没烧成新固件或 `fusion_ready` 未置位。
该数值来自 `IMU_Recv_Snapshot_t.frame_rate`（节点B 实测统计，非节点A自报）。

---

## 6. 如何在 OLED 上显示链路状态

`imu_recv_task` 故意**不直接刷屏**（软件 I2C 很慢，会和 `oled_task` 抢总线）。
统计数据由 `oled_task` 自取：

```c
#include "imu_recv_task.h"

IMU_Recv_Snapshot_t snap;
IMU_Recv_GetSnapshot(&snap);

OLED_ShowString(0,  0, "IMU:", OLED_6X8);
OLED_ShowNum   (24, 0, snap.frame_rate, 3, OLED_6X8);   /* Hz */
OLED_ShowString(48, 0, "CRC:", OLED_6X8);
OLED_ShowNum   (72, 0, snap.crc_err, 3, OLED_6X8);
OLED_ShowString(0,  8, "Drop:", OLED_6X8);
OLED_ShowNum   (32, 8, snap.seq_drop, 4, OLED_6X8);
OLED_ShowString(64, 8, snap.link_lost ? "LOST" : "OK ", OLED_6X8);
```

**验收标准**：

| 指标 | 目标 |
|---|---|
| `frame_rate` | 稳定（具体值取决于节点A，需实测） |
| `crc_err` | **0** |
| `seq_drop` | **0** |
| `resync` | 0 或极小 |
| `link_lost` | 0 |
| `byte_ring_drop` / `ring_overrun` | 0 |

---

## 7. 调试排查顺序

接上线却收不到数据时，按这个顺序查（每步都能独立确认）。
**第 0 步先做，能省掉一大半排查时间**：

0. **★ 烧写完有没有真的跑起来？**
   2026-10-16 实测：主机收不到数据，**手动复位后立刻正常** —— 原因是 Keil 下载后
   没有自动复位运行（停在旧状态/未启动），并不是链路或代码问题。
   - **根治**：Keil → `Options for Target` → `Utilities` → `Settings` → `Download Function`
     勾上 **Reset and Run**，以后下载完自动跑；
   - **不改设置的话**：每次下载后按一下节点A 的复位键，并**看一眼节点A 自己的 OLED
     有没有在动**（数值在跳 = 真的在跑，这是最省事的"活着"判据）。

1. **先用主机端的数字做分层判断（菜单 2/4 页）**
   | `RXB`（原始字节数） | `OK` / `CRC` | 结论 |
   |---|---|---|
   | 不动 | 全 0 | 物理层没数据 → 走第 3~5 步（共地/交叉/波特率） |
   | 在涨 | `OK` 0、`CRC` 涨 | 字节到了但帧不合法 → 第 2、7 步 |
   | 在涨 | `OK` 在涨 | 链路正常，看 `RATE` 是否 100 |
   （`RXB` 是 2026-10-16 加到菜单页的，直接读 `DIY_IMU_GetStats().rx_bytes`，不改本任务）

2. **节点A在发吗** —— 把节点A的 PA2 接 USB-TTL，用 `tools/serial_logger.py` 确认能解析出帧
3. **波特率** —— 两边都是 115200（节点A的 `usart.c` 与它自己的 `.ioc` 都改过）
4. **共地了吗** —— 这是最常见的疏漏
5. **TX/RX 交叉了吗** —— 节点A PA2 → 本板 PB11
6. **`DIY_IMU_Init` 是否成功** —— 失败说明 CubeMX 侧 DMA/中断没配好；
   看 `Core/Src/usart.c` 的 `hdma_usart3_rx` 是否为 `DMA_CIRCULAR`、`USART3_IRQn` 是否使能
7. **回调有没有被调用** —— `main.c` 的 `HAL_UARTEx_RxEventCallback` 里在 USART3 分支打个断点
8. **CRC 是否全错** —— 若 `crc_err` 涨而 `frames_ok` 不动，说明帧格式/CRC 对不上，
   对照节点A的 `docs/串口帧协议.md` 逐字段核

> **主机端有自愈**：`imu_recv_task.c` 里超过 `IMU_LINK_TIMEOUT_MS` 没收到字节会自动
> `DIY_IMU_Reinit()` 重装接收，所以"主机自己卡死"的情况会自己恢复 —— 一旦需要**手动复位**
> 才通，优先怀疑**对端（节点A）没在跑**，而不是主机侧。

---

## 8. 改动清单

| 文件 | 改动 |
|---|---|
| `MDK-ARM/icode/driver/diy_imu.c/.h` | **新增**：链路层驱动（DMA + 字节环 + 帧解析 + CRC16 + 整数换算） |
| `MDK-ARM/icode/task/imu_recv_task.c/.h` | **改写**：原为骨架（只有 TODO），现为完整任务实现 |
| `Core/Src/main.c` | `HAL_UARTEx_RxEventCallback` 增加 USART3 → `DIY_IMU_RxEventCallback` 分支；补 `#include "diy_imu.h"` |
| `Core/Src/stm32f1xx_it.c` | 补 `#include "diy_imu.h"` / `"gps.h"`（回调本体仍在 `main.c`，避免重复定义） |
| `Core/Src/freertos.c` | `IMU_RECV_TASK_STACK` 256→128、`OLED_TASK_STACK` 256→192；追加 RAM 账本说明 |
| `Core/Inc/FreeRTOSConfig.h` | `configTIMER_TASK_STACK_DEPTH` 128→64 |
| `MDK-ARM/Security Watch.uvprojx` | 登记 `diy_imu.c/.h` |
| `Security Watch.ioc` | 删除未使用的 `SWITCH_1`(PB4) / `SWITCH_2`(PB5) 定义 |
| `Core/Inc/main.h`、`Core/Src/gpio.c` | 同步删除 `SWITCH_1/2` 宏与 GPIO 初始化 |

### 本工程的编码约定（这次踩了好几次，务必记住）

| 对象 | 编码 | 说明 |
|---|---|---|
| `Security Watch` 源文件 | **UTF-8** | 本工程 |
| `ICM42688+MMC5603` 源文件 | **GBK** | 节点A工程 |
| `.ioc` 文件 | ASCII，**LF** 行尾 | 而 `Core/Src/*.c` 是 **CRLF** |

**踩过的坑（都真实发生过）：**

1. **两个工程编码不同，不能混用。** 往哪个工程加文件，就按那个工程的编码写。
2. **`main.c` 原本是 GBK**，而工程其余文件是 UTF-8 —— 这个文件本身就是混的。
   我用 PowerShell 的 `Encoding(936)` 改它是对的，但我往里加的注释也是 GBK 字节，
   于是整文件成了"UTF-8 文件里混 GBK 字节"。已统一转为 **UTF-8**（用 GBK 解码后
   重新以 UTF-8 写出，`U+FFFD` 数为 0，无信息丢失）。
3. **脚本里写中文注释不要用裸双引号**，改用【】。裸引号会让 PowerShell
   解析失败，或让"数括号是否平衡"的检查误报。
4. **行尾要自适应**：`sub_once()` 那种按子串替换的工具函数，必须先探测文件是
   `\n` 还是 `\r\n` 再构造模式，否则会静默匹配 0 次。

**验证命令**（确认某文件编码是否合规）：

```python
b = open(path, "rb").read()
b.decode("utf-8")      # 不抛异常 = 合法 UTF-8
```
