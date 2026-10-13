# GPS 模块接入方案（DX-GP22-A / ATGM336H → Security Watch 主控节点）

> 对象：`Security Watch`（可穿戴跌倒与疲劳检测系统的**主控节点 B**，STM32F103C8T6）
> 对应题面：要求（4）"跌倒事件通过 BLE/4G 上报至云端，**附带 GPS 位置信息**"
> 资料位置：`GPS资料\`
> 状态：**驱动与任务代码已落地**（`Security Watch\MDK-ARM\icode\`），CubeMX 侧的串口引脚还需你确认后开一次
> 关联文档：`00-方案与计划.md` §1.1 / §3(4)、`docs\Security Watch 工程骨架说明.md`、`docs\TinyML 使用说明.md`

---

## 1. 一句话结论

**这套资料够用，而且把原方案里最"虚"的一环补实了。**

原方案（`00-方案与计划.md` v1.0）写的是"MCU 不带 GPS，定位由手机提供"。现在有了 DX-GP22-A（ATGM336H 方案，UART 输出 NMEA0183），正确做法是：

> **定位改由设备端自己采**，主控节点 B 用一路 UART 收 NMEA，解析出经纬度后直接拼进跌倒告警报文；
> **手机中继保留，但只负责 4G/WiFi 上云**。这样"跌倒 → 位置 → 上云"整条链路都在你手里，答辩时可现场演示，论文里"移动端"章节也不再被手机定位牵着走。

代价只有一个：**节点 B 需要 3 路 UART（节点 A 数据链路 + GPS + HC-08 BLE），而当前引脚规划只空出 2 路完整 UART**。解决办法是把两个用不到的按键（KEY_3/KEY_4）从 PA2/PA3 挪到空引脚，让出 USART2。详见 §4。

---

## 2. 资料盘点：哪些有用、怎么用

| 资料 | 有用程度 | 用法 |
|---|---|---|
| `DX-GP22-A（有定位工具）资料包\01 模块技术手册\DX-GP22-GNSS模块技术手册.pdf` | ★★★ | 硬参数唯一权威来源：引脚定义、供电范围、功耗、波特率、1PPS、备电、天线设计。**接线前必读 §2.2/§2.6/§3** |
| `…\02 模块功能应用指导\DX-GP22-GNSS模块_串口UART_应用指导.pdf` | ★★★ | `$PCA` 命令集：改波特率（PCAS01）、改更新率（PCAS02）、**精简输出语句（PCAS03）**、选星座（PCAS04）、热/冷启动（PCAS10）。本方案的 `GPS_ConfigMinimal()` 就按它实现 |
| `…\02 模块功能应用指导\DX-GP22-NMEA0183协议规范.pdf` | ★★★ | RMC/GGA 字段定义（本方案解析器逐字段对照实现），含定位质量标志、海拔、HDOP 含义 |
| `…\03 模块测试工具\PC串口助手\sscom5.13.1.exe` + `CH340-USB转串口驱动.rar` | ★★★ | **上板前的地面验证工具**：USB-TTL 直连模块，9600 8N1 看 NMEA 原始语句，并实测 `$PCAS03` 是否被接受 |
| `…\03 模块测试工具\定位测试工具\MobiletekGNSSTool.rar` | ★★ | 厂家上位机（13 MB），可看星空图/信噪比/定位状态，写论文"定位性能"章节时截图很有用 |
| `…\04 模块硬件资料\DX-ATGM336H_PCB_Footprint&SCH_Part.zip` | ★ | ATGM336H 的 PCB 封装与原理图符号；**只有你要自己做底板/毕设 PCB 时才需要** |
| `大夏龙雀-GPS模块例程\…\GPS\`（标准库 SPL 工程） | ★★ | 只能**参考思路**，不能直接并入本工程（本工程是 HAL + FreeRTOS）。它演示了两件对的事：① USART1 用"空闲中断"判断一帧 NMEA 结束；② 解析 RMC 的字段顺序 |
| `大夏龙雀-GPS模块例程\…\GPS-4G\`（C511/WF24 4G 例程） | ★ | 里面 `WIFI.c` 是 4G/WiFi 模块的 AT+MQTT 流程（`AT+MCONFIG / AT+MIPSTART / AT+MPUB`），**当前没有 4G 硬件，先不用**；若日后补 4G 模块，这是现成的 AT 指令参考 |
| `…\05 单片机例程\单片机例程下载地址.txt` | ★ | 厂家网盘（`pan.baidu.com/s/1TM_bhhZxs1XYpwCInLnWmQ?pwd=DXLQ`）与公众号下载入口，需要更多例程时再取 |

### 2.1 ⚠ 对厂家例程代码的审查结论（不要照抄）

`大夏龙雀-GPS模块例程` 的 `main.c` / `USART.C` / `WIFI.c` 是 GBK 编码的 **STM32 标准库**工程，且有明确的工程缺陷。本方案没有照抄，逐条列出以便你在论文/答辩中说明"为什么自己重写"：

| 例程问题 | 后果 | 本方案的处理 |
|---|---|---|
| 全程不做 NMEA 校验和验证 | 半帧/噪声会被当成有效定位 | `GPS_CheckSumOk()` 逐句异或校验，错误计入 `GPS_GetCsumErrCount()` |
| `gps.buff_length` 无上限，中断里直接 `Frame_Buffer[buff_length++]` | 缓冲区溢出，最多写到 526 字节 | 中断只搬字节进环形缓冲并处理回卷；语句装配有 96 字节上限与溢出丢弃 |
| `memset(gps.Frame_Buffer, 0, sizeof(gps)/sizeof(gps.buff_flag))` | 长度算错（`sizeof(struct)` 除以 1，等于整个结构体大小） | 不依赖这种写法，装配缓冲显式清长度 |
| `memcpy(gps.UTCTime, point, nextpoint-point)` 不补 `'\0'` | 字段串没有终止符，`strtod/printf` 越界读 | 全部字段就地 `'\0'` 切分，指针+长度解析，无字符串复制 |
| 用 `strtod()`（浮点）与 `printf("%.3f")` | 拉进浮点格式化库，F103 无 FPU，Flash 也吃不消 | **全整数定点**解析：`e7 = deg*1e7 + (min_e5*5+1)/3`，误差 < 4 mm；格式化自己手写，不引入 `printf` |
| 只匹配 `strstr(Frame_Buffer,"GNRMC")` | 模块切到纯 GPS 模式会输出 `GPRMC`，直接匹配不到 | 只看语句头**后 3 个字符**，`GNRMC/GPRMC/GLRMC/BDRMC` 通吃 |
| 解析放在 `while(1)` 主循环里轮询 `buff_flag` | 丢帧不可控 | 解析放在 GPS 任务（20 ms 周期），中断只做搬运 |
| USART1 中断里清 IDLE 的方式不覆盖溢出/错误标志 | 偶发死锁/丢数据 | 交给 HAL 的 `HAL_UART_IRQHandler` + `HAL_UARTEx_RxEventCallback` |

---

## 3. 模块硬参数（照抄自技术手册，接线前请核对丝印）

| 项目 | 数值 / 说明 |
|---|---|
| 方案 | ATGM336H（AT6558 平台），GPS + BDS + GLONASS 多模单频 |
| **底板 VDD** | **3.6 ~ 6 V，典型 5 V** ← ⚠ **不能用 3.3 V 直供**（低于下限，可能起不来或反复复位） |
| 裸模块 VCC | 2.7 ~ 3.6 V（3.3 V typ）；电源要能提供 **≥100 mA**，VCC 脚旁 10 µF + 100 nF，建议加 TVS |
| VBAT 备电 | 1.5 ~ 3.6 V（3.0 V typ），电流约 10 µA；**接了才能热启动**（TTFF 从 30 s+ 降到数秒），悬空则每次上电都是冷启动 |
| 功耗 | 32 mA typ @3.3 V（全工作模式） |
| 串口 | 默认 **9600 bps / 8 数据位 / 无校验 / 1 停止位**；可设 4800~115200（`$PCAS01,n`） |
| I/O 电平 | 模块侧 I/O ≤ 3.6 V，即 3.3 V TTL，可直接接 STM32（5 V 供电是给底板 LDO 的，不是给 I/O 的）。**首次上板前建议万用表量一下 TXD 空闲电平 ≈3.3 V** |
| 定位更新率 | 默认 1 Hz；可 2/5/10 Hz（`$PCAS02`） |
| 默认输出语句 | GGA / GLL / GSA / GSV / RMC / VTG / ZDA …（约 620 B/s，GSV 突发时约 900 B） |
| 1PPS | 定位成功后 1 Hz 秒脉冲（可接 LED 或 MCU EXTI，本课题不用） |
| ON/OFF | 关断控制，**高电平 = 工作 / 低电平 = 关断**；不用则悬空或接高 |
| 天线 | 底板带陶瓷天线；有源天线由 VCC_RF 供电（≤50 mA），RF 走线 50 Ω |
| 底板接口 | 1.25-6P 贴片端子：VDD / GND / TXD / RXD / 1PPS / ON-OFF（以丝印为准） |

---

## 4. 接到哪里：三路 UART 的账要算清

### 4.1 节点 B 现有引脚占用（来自 `Security Watch.ioc`，实测）

| 已占用 | 用途 |
|---|---|
| PA0~PA3 | KEY_1~KEY_4（按键） |
| PA4 / PA5 | ADC（光敏/电池检测，原智能家居残留） |
| PA6 / PA7 / PB0 / PB1 | TIM3 CH1~CH4（三路 PWM 调光） |
| PB3 / PA15 | TIM2 编码器接口（驱动已删，引脚宏还在） |
| PA10 | TIM1_CH3（PWM）——**ws2812b 驱动已删除，此脚现已空闲** |
| PA13 / PA14 | SWD 调试 |
| PB4 / PB5 / PB12 / PB13 / PB14 | SWITCH_1/2、Encoder_button、DHT11、**BEEP** |
| PB8 / PB9 | OLED（软件 I2C） |
| PC14 / PC15 / PD0 / PD1 | LSE（RTC）与 HSE（晶振） |

**空着的引脚**：PA8、PA9、PA11、PA12、PB2、PB6、PB7、PB10、PB11、PB15、PC13。

### 4.2 可用的 UART（F103C8 LQFP48）

| 串口 | 默认引脚 | 本项目可用性 |
|---|---|---|
| USART1 | PA9(TX) / PA10(RX) | 当时 PA10 被 WS2812 占用；**现 ws2812b 已删、PA9/PA10 也空了**，但仍沿用已接线的重映射 PB6/PB7 |
| USART2 | PA2(TX) / PA3(RX) | ⚠ 被 KEY_3/KEY_4 占用；48 脚封装**没有重映射**，必须让出这两个脚才能用 |
| USART3 | PB10(TX) / PB11(RX) | ✅ 两个脚都空，直接可用 |

**结论：完整可用的只有 2 路（USART1 重映射 + USART3），而本方案需要 3 路。**

### 4.3 三路链路的资源分配（推荐方案）

| 链路 | 串口 / 引脚 | 速率 | 接收方式 | DMA |
|---|---|---|---|---|
| 节点 A → 节点 B（IMU 帧） | **USART3：PB10(TX) / PB11(RX)** | 115200 | 循环 DMA + 空闲中断 | **DMA1_CH3** |
| **GPS NMEA** | **USART1 重映射：PB6(TX) / PB7(RX)** | 9600 | 循环 DMA + 空闲中断 | **DMA1_CH5** |
| HC-08 BLE 透传 | **USART2：PA2(TX) / PA3(RX)** | 9600 或 115200 | 收：字节中断（9600 时完全够）；发：可选 DMA1_CH7 | — |

> **为什么 GPS 放 USART1、把 USART3 留给节点 A 链路**：F103 的 DMA 请求是固定映射
> （USART1_RX=CH5、USART2_RX=CH6、USART3_RX=CH3、USART2_TX=CH7、USART3_TX=CH2）。
> 而当时 **DMA1_CH6 被认为被 WS2812（TIM1_CH3）占用**，所以 USART2 的 **接收**走中断。
> （2026-10-14 删除 ws2812b 后 CH6 已空闲，但 BLE 速率低，中断方式保持不变。）
> 让高速链路（节点 A，3.2 KB/s）走 USART3+DMA1_CH3，GPS（约 620 B/s）走 USART1+DMA1_CH5，
> BLE（低频、短报文）走 USART2 用中断收，DMA 冲突自然消失，不必删 WS2812。

### 4.4 需要动的引脚：KEY_3 / KEY_4 换位

题面只要求"取消按钮"（要求 5），实际功能只需要 1~2 个按键，所以：

| 原 | 新（建议） | 说明 |
|---|---|---|
| KEY_3 = PA2 | **PA11** | 空引脚；CubeMX 里改 User Label 与引脚即可 |
| KEY_4 = PA3 | **PA12** | 同上（PA11/PA12 即 USB D-/D+，本项目不用 USB） |

改完后 PA2/PA3 让给 USART2 → 接 HC-08。若你更希望**完全不动按键接线**，见 §4.5 的方案乙。

### 4.5 备选方案对比

| 方案 | 做法 | 优点 | 缺点 |
|---|---|---|---|
| **甲（推荐）** GPS 挂主控节点 B | 按 §4.3/§4.4 接 | 改动只在 `Security Watch` 一个工程（本工作区可直接编译验证）；定位数据直通告警报文，链路最短；节点 A 固件完全不动 | 要让出 PA2/PA3（改 CubeMX + 挪两根杜邦线） |
| 乙 GPS 挂传感器节点 A，帧转发 | 节点 A 的 USART1（PA9/PA10，两个脚都空）接 GPS，解析后在 32 字节帧里加一个 `type=0x02` 的 GPS 帧（1 Hz）发给节点 B | 节点 B 引脚完全不动（只需 USART3 + USART1 重映射两路） | 要改**两个**工程：`D:\STM32Project\ICM42688+MMC5603 9 axis gyroscope` 不在本工作区；协议要加帧类型；定位多一跳（延迟 <1 s） |
| 丙 软串口 | 用一个空 GPIO + EXTI/定时器位敲 9600 收 NMEA | 不占硬件串口 | 增加 CPU 负载与不确定性，可维护性差；除非真的一个 UART 都挤不出来，否则不用 |

**建议：按方案甲实施**；如果已经按"节点 A 带 GPS"焊好了，再切方案乙。

---

## 5. 硬件接线与供电

```
        ┌────────────────────┐                    ┌──────────────────────────┐
        │  DX-GP22-A 底板     │                    │  Security Watch (B)       │
        │                    │                    │  STM32F103C8T6            │
        │  VDD ──────────────┼──── 5V ────────────┤ 5V / VBAT 升压输出        │
        │  GND ──────────────┼──── GND ───────────┤ GND                       │
        │  TXD ──────────────┼───────────────────▶│ PB7  = USART1_RX          │
        │  RXD ◀─────────────┼────────────────────┤ PB6  = USART1_TX          │
        │  ON/OFF ───────────┼──(可选)───────────▶│ 空 GPIO（省电门控，高=工作）│
        │  1PPS ─────────────┼──(可选)────────────▶│ 空 GPIO / LED             │
        │  VBAT ◀────────────┼── 3V 备电（CR2032）│ —                         │
        └────────────────────┘                    └──────────────────────────┘
```

要点：

1. **共地**必需；TX/RX 必须交叉（GPS TXD → MCU RX）。
2. **VDD 走 5 V**（底板有 LDO，3.6~6 V）：从锂电池升压 5 V 或 USB 5 V 取电，注意能提供 ≥100 mA。
3. **VBAT 接 3 V 备电**（CR2032 或 3 V LDO）：保住 RTC/SRAM → 热启动，演示时"开机 5 秒就能定位"和"每次冷启动等 40 秒"是完全不同的观感。
4. **天线朝上、尽量无金属遮挡**；可穿戴形态下人体和衣服对 GNSS 遮挡严重，**实操建议**：演示前在室外/靠窗处先完成首次定位，进屋后再演示跌倒告警（报文带 `fix_age`，语义仍然正确）。
5. ON/OFF 若接 MCU，注意它是"高=工作"；驱动器里用 `GPS_Power(1/0)`，默认编译为空操作（`gps.c` 顶部打开 `GPS_PWR_GPIO_Port/Pin` 宏即生效）。

---

## 6. 固件设计（已落地代码）

### 6.1 分层与数据流

```
USART RX ──循环 DMA──▶ s_dma_buf(128B) ──RxEvent(ISR, 按位置差搬运)──▶ s_ring(256B)
                                                    │
                                     gps_task / GPS_Process()  ← 20 ms 周期
                                                    ▼
                                        NMEA 语句装配 → 校验和 → 字段切分
                                                    ▼
                                     RMC/GGA 解析（整数定点）→ s_fix 快照(seqlock)
                                                    ▼
                        ble_task: GPS_BuildJson()      oled_task: GPS_FormatPosition()
```

| 文件 | 作用 |
|---|---|
| `Security Watch\MDK-ARM\icode\driver\gps.c / gps.h` | NMEA0183 驱动：接收搬运、语句装配、校验、RMC/GGA 解析、`$PCA` 配置、JSON/显示格式化、诊断计数、可选电源门控 |
| `Security Watch\MDK-ARM\icode\task\gps_task.c / gps_task.h` | GPS 任务：绑定串口、发配置命令、周期解析、链路健康检查（5 s 无字节自动重启接收） |
| `Security Watch\Core\Src\freertos.c` | 已加入 `GPS_TASK_PRIO = osPriorityBelowNormal`、`GPS_TASK_STACK = 256` 与 `GPS_Task_Init()` |

### 6.2 关键接口（`icode/driver/gps.h`）

```c
void     GPS_Init(UART_HandleTypeDef *huart);          /* 绑定串口 + 启动接收 */
void     GPS_RxEventCallback(UART_HandleTypeDef*, uint16_t Size);  /* 在 HAL 回调里转发 */
void     GPS_Process(void);                            /* 任务里周期调用，解析 */
uint8_t  GPS_GetFix(GPS_Fix_t *out);                   /* 取最新定位快照（无锁） */
uint8_t  GPS_IsFixFresh(uint32_t max_age_ms, uint32_t now_ms);
uint16_t GPS_BuildJson(char *buf, uint16_t buflen, uint32_t now_ms);   /* 告警报文片段 */
uint16_t GPS_FormatPosition(char *buf, uint16_t buflen, uint32_t now_ms); /* OLED 用 */
void     GPS_ConfigMinimal(void);                      /* $PCAS03 只留 GGA+RMC */
void     GPS_Power(uint8_t on);                        /* ON/OFF 门控（可选） */
```

**必须在 `main.c` 的 `/* USER CODE BEGIN 4 */` 里补上转发**（否则收不到数据）：

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  GPS_RxEventCallback(huart, Size);
}
```

**任务里的串口句柄只在一处指定**（`gps_task.c` 顶部，默认已按 §4.3 写成 USART1）：

```c
extern UART_HandleTypeDef huart1;
#define GPS_UART_HANDLE     (&huart1)     /* 换串口只改这两行 */
```

### 6.3 报文格式与标度（定点整数，避免浮点格式化）

```json
{"t":1712345678,"dev":"JBX-01","ev":"FALL","conf":0.93,
 "acc":[1.2,-2.8,3.9],"rpy":[82.1,-15.3,7.6],"step":98,"seq":4521,
 "gps":{"v":1,"lat":399876543,"lon":1163123456,"q":1,"sat":9,
        "hdop":95,"alt":4300,"spd":634,"cog":0,"fix_age":1200}}
```

| 字段 | 含义 | 标度 |
|---|---|---|
| `v` | 定位是否有效 | 1 = RMC status=A **且** GGA quality>0 |
| `lat` / `lon` | 纬度 / 经度 | 度 × 1e7（北纬/东经为正）；`399876543` → 39.9876543° |
| `q` | 定位质量 | 0 无效 / 1 单点 / 2 差分（GGA 第 6 字段） |
| `sat` | 参与定位卫星数 | 颗 |
| `hdop` | 水平精度因子 | ×100 |
| `alt` | 海拔 | cm |
| `spd` | 对地速度 | cm/s（模块原始单位是节，驱动内换算） |
| `cog` | 对地航向 | 0.01° |
| `fix_age` | 该定位距今 | ms（**室内无星时靠它表达"位置有多旧"**） |

设计要点：
- **无定位不阻塞告警**：`v=0` 时经纬度为 0，但 `fix_age` 仍给出最后已知位置的新鲜度，云端可据此提示"位置可能已过时"。
- **无锁发布**：GPS 任务是唯一写者，用"序号奇偶"（seqlock）发布快照，`ble_task`/`oled_task` 读时不需要互斥量，也不会读到半新半旧的数据。
- **绝不引入 `printf`**：本工程为省 Flash 已裁掉 `vsprintf`（见 `docs\TinyML 使用说明.md`），所以 JSON 与经纬度字符串都是手写整数转字符串。

### 6.4 资源预算（F103C8：64 KB Flash / 20 KB RAM）—— **实测版**

用 `tools\armcc_build_check.py`（见 §6.5）直接调 armcc/armasm/armlink 完整编译 + 真机 scatter 链接，
得到下面这组**实测数字**（优化 -O3、`--split_sections`、与 Keil 工程选项一致）：

| 构建 | Flash (RO) | RAM (RW+ZI) | 说明 |
|---|---|---|---|
| 上次成功构建（2026-10-01，`Security Watch.map`） | 19,548 B (19.09 KB) | 18,760 B (18.32 KB) | 当时还没把 `icode/ml` 加进工程，也没有 GPS |
| 加 TinyML 窗口缓冲 + GPS + UART 后（未裁剪） | 32,972 B (32.20 KB) | **22,720 B (22.19 KB) ✗** | **RAM 超 20 KB 共 2,240 B，链接直接失败（L6406E/L6407E）** |
| **本次裁剪后（GPS+UART 落地时）** | 32,964 B (32.19 KB) | 19,264 B (18.81 KB) ✓ | 余量：Flash 32,572 B / RAM 1,216 B |
| **最新实测（含 10-12 新增的 `diy_imu.c` 与升级后的按键驱动）** | **34,912 B (34.09 KB)** | **19,176 B (18.73 KB) ✓** | 余量：Flash 30,624 B / **RAM 1,304 B** |

**⚠ 这是本次最重的一个发现，且不是 GPS 引起的全部：**

```
RAM 增量来源（对比上次成功构建）：
  tinyml_task.o  +2,596 B   ← ML 滑窗环形缓冲 s_ring[201][6] int16 = 2,412 B
                               （上一轮做 TinyML 时加的，但那份工程从未真正链接过，
                                 所以没人发现它把 RAM 顶穿了）
  gps.o          +1,008 B   ← 本次 GPS 驱动
  usart.o          +352 B   ← 本次 3 路 UART + 2 路 DMA handle
  ─────────────────────────
  合计            +3,960 B   18.76 KB → 22.72 KB（20 KB 上限）
```

**裁剪措施（4 项，合计 -3,456 B，全部已落地）：**

| # | 改动 | 省 RAM | 理由 / 风险 |
|---|---|---|---|
| 1 | `configTOTAL_HEAP_SIZE` 12,288 → **10,240** | -2,048 B | 实测堆需求 ≈ 8.4 KB（7 个应用任务栈 7,168 B + 9 个 TCB ≈ 940 B + 定时器队列 ≈ 320 B）。空闲/定时器任务的栈是 `cmsis_os2.c` 里的**静态数组**，不占这块堆，所以 10,240 还能剩约 1.8 KB |
| 2 | `configTIMER_TASK_STACK_DEPTH` 256 → **128** | -512 B | 本工程**没有创建任何软件定时器**，512 B 足够；该栈同样是 `cmsis_os2.c` 的静态数组 |
| 3 | startup `Heap_Size` 0x200 → **0x000** | -512 B | 全工程没有 `malloc/calloc/free`（已用 grep 逐个确认），C 库堆纯属白占 |
| 4 | `gps.h`：DMA 256→**128**、环形 512→**256** | -384 B | 默认语句集约 620 B/s、任务 20 ms 轮询一次，256 B 仍有 ~0.25 s 抗阻塞余量；`$PCAS03` 精简后流量降到约 150 B/s |

配套（不省静态 RAM，但增加堆余量）：删掉 CubeMX 生成的空转 `defaultTask`（-512 B 栈 -约 90 B TCB）、
`OLED_TASK_STACK` 512→**256** word、`GPS_TASK_STACK` 192→**128** word。

**本次新增模块的实测占用（论文"资源占用表"可直接用）：**

| 模块 | Flash | RAM（静态） | 说明 |
|---|---|---|---|
| `gps.o`（NMEA 驱动） | **1,986 B** | **624 B** | DMA 128 + 环形 256 + 语句 96 + 快照/统计 |
| `gps_task.o`（任务壳） | **120 B** | 4 B | 栈走 FreeRTOS 堆（128 word = 512 B） |
| `usart.o`（3 路串口 + 2 路 DMA handle） | **624 B** | **352 B** | CubeMX 风格生成码 |
| `stm32f1xx_hal_uart.o`（HAL 串口驱动） | **2,106 B** | 0 | 原工程未启用 UART 模块 |
| **合计** | **4,836 B (4.72 KB)** | **980 B** | 占 64 KB Flash 的 7.4%、20 KB RAM 的 4.8% |

**RAM 结构（裁剪后，共 19,264 B）：** FreeRTOS 堆 10,272 / TinyML 滑窗 2,600 / 内核静态 1,280 /
CMSIS-RTOS2（空闲+定时器栈）1,212 / OLED 显存 1,024 / 主栈 1,024 / **GPS 624** / **UART 352** / 其余约 900。

> 论文里建议这样写：Flash 32.19 KB（50.3%）、RAM 18.81 KB（94.1%），
> 其中定位与串口子系统的增量是 **Flash +4.72 KB / RAM +980 B**；
> 并说明"为容纳 TinyML 滑窗缓冲曾出现 2.2 KB RAM 超限，通过堆裁剪/静态栈精简/缓冲区重算解决"——
> 这段是很好的工程论证素材。

### 6.4.2 2026-10-13 更新：GPS 任务已恢复创建（三路串口分工落定）

**背景**：DIY IMU 链路（`diy_imu.c` + `imu_recv_task.c`，占 **USART3**）接进来后 RAM 更紧，
上一轮为了腾 RAM 把 `GPS_Task_Init()` **注释掉了** —— 于是 `gps.c` 全套代码都在、配置也对，
但任务从未创建，自然收不到数据。三路串口分工现已定死：

| 串口 | 外设 | 引脚 |
|---|---|---|
| **USART1** | **GPS（DX-GP22-A）** | PB6 / PB7（重映射） |
| USART2 | HC-08 BLE | PA2 / PA3 |
| USART3 | DIY IMU 链路 | PB10 / PB11 |

**本次恢复 + 腾堆（IMU 那一侧一行未改）**：

| 改动 | 省 |
|---|---|
| `FATIGUE_TASK_STACK` 256 → 192 word | -256 B |
| `ALARM_TASK_STACK` 256 → 192 word | -256 B |
| `BLE_TASK_STACK` 256 → 192 word | -256 B |
| `configTIMER_QUEUE_LENGTH` 10 → 4（本工程无软件定时器） | -96 B |

**恢复后的堆账本**（`python tools\heap_ledger.py` 可直接算出来）：

```
堆 configTOTAL_HEAP_SIZE            = 8192 B   ← 2026-10-14 为接入 HC-08 再从 9280 降下来
                                                  （堆实测只需约 6.9 KB，仍余 2 KB）
7 个应用任务栈合计                    = 5376 B
7 个 TCB（heap_4 实测 96 B/个）        =  672 B
定时器队列（4 项）                     =  120 B
─────────────────────────────────────────────
合计 need = 6168 B  →  余量 3112 B ✓
```

> 说明：`configTOTAL_HEAP_SIZE` 是 heap_4.c 里一个**固定的 .bss 数组**，装不装任务都占这么多 RAM，
> 所以"少建一个任务"并不省 RAM，必须直接改这个值（这就是上一轮降它的原因）。
> 另外，空闲任务与定时器任务的栈/TCB 是 `cmsis_os2.c` 里的**静态数组**，**不占这块堆**。
> 真实余量以开机时 OLED 第 2 行的 `FreeHeap:` 为准（那一行就是为此加的，期望约 3 KB）。

**同时新增的联调设施**：OLED 加了「GPS 接收页」（见 §8.1），可在
`oled_task.c` 顶部用 `OLED_SCREEN_SEL` 在 IMU / 按键 / GPS 三个屏幕间切换；
`gps_task.c` 顶部新增开关 `GPS_SEND_PCAS_CONFIG`，**默认 0 = 只收不发**，
首次联调时先纯接收，避免"命令写坏导致模块不说话"这个额外变量。

### 6.4.1 编译与链接验证工具：`tools\armcc_build_check.py`

本机 Keil UV4 在 DSH 沙箱里命令行构建会以 `0xC0000409` 退出（不是工程问题，未改动的工程同样如此），
所以做了这个脚本：**直接调用 `E:\Keil5\ARM\ARMCC\bin` 下的 armcc / armasm / armlink**，
从 `uvprojx` 里读文件清单、宏、包含路径、`Optim`、`OneElfS`、`MiscControls`，复刻 Keil 的编译链接过程。

```
python tools\armcc_build_check.py
```

> ⚠ 沙箱环境注意：受限模式下子进程写不进系统的临时目录，`armcc` 会因为建不了临时文件而**全线编译失败**
> （报 `cannot open embedded assembler output file ...\Temp\dsh-xxxx\...`）。
> 解决办法是把编译器的临时目录指到工作区内（工作区在 workspace-write 下可写）：
>
> ```powershell
> $env:TMP = "D:\同学们的毕设\江彬锋的毕设\Security Watch\MDK-ARM\_armcc_tmp"
> $env:TEMP = $env:TMP
> python tools\armcc_build_check.py
> ```

它解决三类问题：
1. **能不能编过**（59 个源文件逐个编译，失败项连日志一起打印）
2. **能不能链上**（真机 scatter：64 KB ROM / 20 KB RAM，容量不够会如实报出超出多少）
3. **占多少**（超出容量时自动用放大存储区复测，并给 `--info sizes` 的逐对象明细）

踩过的三个坑（都已在脚本里处理，值得记下来）：
- 编译选项在 `TargetArmAds/Cads/VariousControls` 下，不是 `Cads` 直接子节点；
- **必须传 `--split_sections`**（对应 Keil 的 `OneElfS=1`），否则未用函数连同 printf/浮点库一起被链进来，
  体积会虚高到 85 KB（真实值 33 KB）；
- **必须传 `--no-multibyte-chars`**（工程 MiscControls 里就有），否则 UTF-8 中文字符串会被按双字节解释、
  吞掉引号，`OLED_Data.c` 会报 "missing closing quote"。


### 6.5 PC 端算法自证：`tools\gps\verify_nmea_math.py`

`gps.c` 里没有浮点、没有 `printf`，坐标换算/校验和/字段下标全是整数与指针运算，
这类代码在板子上出错时只表现为"定位偏一点"或"永远没定位"，很难查。因此把同一套算法
用 Python 逐行复刻，**拿厂家 PDF 里的原句和原命令校验和做对照**：

```
python tools\gps\verify_nmea_math.py
```

当前结果（全部通过，可直接引用到论文的"测试与验证"章节）：

| 用例组 | 内容 | 结果 |
|---|---|---|
| 1 | `$PCA` 命令校验和，对照《串口UART应用指导》里 28 条带 `*CS` 的原命令 | 28/28 通过 |
| 2 | 协议规范里的 RMC/GGA 示例句：自算校验和 = 文档值（`*00` / `*6E`）；坐标与浮点参考完全一致（偏差 0） | 通过 |
| 3 | 20 万组随机坐标，整数换算 vs 浮点参考 | 最大偏差 1 个 e7 单位 ≈ **1.1 cm** |
| 4 | 字段下标（`s_fp[0]` 为语句头，时间/纬度从 1 起算） | 16/16 通过 |
| 5 | 端到端流水线仿真：噪声 + 正常句 + 篡改校验和 + 无关 GSV + 无效定位 | 行为符合预期 |

> 这个脚本在第一次运行时**真的抓出了一处缺陷**：RMC/GGA 的字段下标少算 1（把 `s_fp[0]` 当成了时间），
> 已修复。这正是"先在 PC 上证明算法、再上板"的价值。

---

## 7. CubeMX 工程改动 —— ✅ 已完成（2026-10-02）

> ⚠ 前提说明：**本机没有安装 STM32CubeMX**（见 `00-方案与计划.md` §1.4）。
> 所以这一节的工作由我**直接手改 `.ioc` + 按 CubeMX 6.16 的生成风格手写生成码**完成：
> `Core\Src\usart.c`、`Core\Inc\usart.h` 是照 `tim.c` 的排版/命名/`USER CODE` 分区写的，
> `dma.c` / `stm32f1xx_it.c` / `hal_conf.h` 的改动也严格照 CubeMX 会生成的写法。
> 结论已通过**真实 armcc/armlink 完整编译链接**验证（§6.4/§6.5），但**`.ioc` 需要你打开 CubeMX 复核一次**——
> 打开后若提示任何冲突，请把提示原样发我。

**已落地的配置（`.ioc` 里可自行核对）：**

| 项目 | 配置 |
|---|---|
| USART1（GPS） | Asynchronous，**重映射 PB6=TX / PB7=RX**，9600 8N1，DMA `USART1_RX` = **Circular**（`DMA1_Channel5`），NVIC 使能 USART1 + DMA1_CH5 |
| USART2（HC-08 BLE） | Asynchronous，PA2=TX / PA3=RX，9600 8N1，**不加 DMA**（改用空闲中断+中断接收），NVIC 使能 USART2 |
| USART3（节点 A 链路） | Asynchronous，PB10=TX / PB11=RX，115200 8N1，DMA `USART3_RX` = **Circular**（`DMA1_Channel3`），NVIC 使能 USART3 + DMA1_CH3 |
| 按键搬迁 | `KEY_3`：PA2 → **PA11**；`KEY_4`：PA3 → **PA12**（PA2/PA3 让给 USART2） |
| FreeRTOS | 删除空转 `defaultTask`；`configTOTAL_HEAP_SIZE=10240`；`configTIMER_TASK_STACK_DEPTH=128` |
| 其它 | `ProjectManager.HeapSize=0x0`（C 库堆清零）；Toolchain 保持 MDK-ARM V5；`functionlistsort` 补入三个 `MX_USARTx_UART_Init` |

**请在 CubeMX 里复核这几项**（打开 `.ioc` → 看是否有报错/冲突提示）：

1. Pinout 视图：PA11/PA12 显示 `KEY_3/KEY_4`（GPIO_Input + Pull-up）；PA2/PA3 属于 USART2；
   PB6/PB7 属于 USART1（且 USART1 处显示重映射）；PB10/PB11 属于 USART3。
2. Connectivity → USART1/2/3 的波特率 = 9600 / 9600 / 115200，DMA 请求与 NVIC 勾选如上一表。
3. FreeRTOS → Config parameters：`TOTAL_HEAP_SIZE=10240`、`TIMER_TASK_STACK_DEPTH=128`；
   Tasks and Queues 里**没有** `defaultTask`。
4. Project Manager → Toolchain/IDE 仍是 **MDK-ARM V5**。
5. 直接 `Generate Code` 后，与工程现状态对比：若 CubeMX 生成的 `usart.c` 与我写的版本有差异，
   **以 CubeMX 为准**（它才是真源），把差异告诉我，我同步调整 `gps_task.c` 的句柄与文档。

### 7.0 生成前必做项的执行情况

| # | 事项 | 状态 |
|---|---|---|
| 0.1 | 备份 `Core\` 与 `uvprojx` | ✅ 已备份到 `Security Watch\_backup_before_usart\`（注意：是改造**中途**的快照，不是原始工程；原始状态可按本文档逐条回退） |
| 0.2 | `.ioc` 的 `configTOTAL_HEAP_SIZE` 与头文件对齐 | ✅ `.ioc` 与 `FreeRTOSConfig.h` 现在都是 **10240**（原 `.ioc` 写 8192、头文件写 12288，两边不一致，重新生成必踩坑） |
| 0.3 | Toolchain = MDK-ARM V5 | ✅ 保持 `MDK-ARM V5.32` 未动 |
| 0.4 | 删掉 `defaultTask` | ✅ `.ioc` 的 `FREERTOS.Tasks01` 已删，`freertos.c` 里对应的句柄/属性/函数也一并清掉 |
| 0.5 | `configASSERT` 仍在 USER CODE 区 | ✅ 在第 142 行 `/* USER CODE BEGIN 1 */` 内，重新生成会保留 |


### 7.1 引脚搬迁：给 USART2 腾出 PA2/PA3

1. 芯片图上点 **PA11** → 选 `GPIO_Input`，在 **System Core → GPIO** 里设置：
   `GPIO mode = Input`、`Pull-up/Pull-down = Pull-up`、**User Label = `KEY_3`**。
2. 同样把 **PA12** 设为 `GPIO_Input` + Pull-up，**User Label = `KEY_4`**。
3. 把 **PA2、PA3** 在芯片图上右键 → `Reset_State`（取消原来的 KEY_3/KEY_4 GPIO 分配）。
   > 顺序很重要：先分配新脚、再释放旧脚，否则 USART2 会报 "pin conflict"。
4. ✅ 验收：`main.h` 里应变成 `KEY_3_Pin GPIO_PIN_11` / `KEY_4_Pin GPIO_PIN_12`；PA2/PA3 不再出现。

### 7.2 USART1 —— 接 GPS（9600，重映射到 PB6/PB7）

1. **Connectivity → USART1** → Mode = `Asynchronous`。
2. 在芯片图上用 **Ctrl + 左键拖拽** 把 USART1 的 PA9/PA10 拖到 **PB6(TX) / PB7(RX)**
   （或在 PB6 的右键菜单里选 `USART1_TX`）。CubeMX 会自动置 `USART1_REMAP`。
   > 之所以重映射：当时 PA10 被 WS2812（TIM1_CH3）占用，PB6/PB7 两个脚都空着。
> （ws2812b 现已删除，PA10 空闲，但接线已按 PB6/PB7 落地，不再改动。）
3. **Parameter Settings**：Baud Rate = **9600**，8 bits，None parity，1 stop bit。
4. **DMA Settings** → `Add` → 选 **USART1_RX** → Mode = **Circular**，Priority = `Medium`。
5. **NVIC Settings** → 勾 `USART1 global interrupt` 与 `DMA1 channel5 global interrupt`。
6. ✅ 验收：`.ioc` 里出现 `USART1.BaudRate=9600`、`PB6.Signal=USART1_TX`、`PB7.Signal=USART1_RX`、
   `Dma.Request*=USART1_RX`、`NVIC.USART1_IRQn=true`、`NVIC.DMA1_Channel5_IRQn=true`。

### 7.3 USART3 —— 接节点 A 的 IMU 数据链路（115200）

1. **Connectivity → USART3** → Mode = `Asynchronous`（默认引脚 PB10=TX / PB11=RX，无需搬动）。
2. Baud Rate = **115200**。
3. **DMA Settings** → `Add` → **USART3_RX** → Mode = **Circular**，Priority = `Medium`。
4. **NVIC** → 勾 `USART3 global interrupt` 与 `DMA1 channel3 global interrupt`。
5. ✅ 验收：PB10/PB11 显示为 USART3_TX/RX；DMA 请求为 USART3_RX。

### 7.4 USART2 —— 接 HC-08 BLE（9600）

1. **Connectivity → USART2** → Mode = `Asynchronous`（PA2=TX / PA3=RX，此时应已空出）。
2. Baud Rate = **9600**（HC-08 出厂值；若已改成 115200 就按模块实际值填）。
3. USART2_RX 用**中断接收**即可：DMA1_CH6 现虽空闲，但 BLE 速率 ≤960 B/s，中断更简单。
   接收用 `HAL_UART_Receive_IT()` 即可（9600 下每秒最多 960 字节中断，完全吃得消）。
   需要 DMA 发送时可加 `USART2_TX`（`DMA1_CH7`，空闲）。
4. **NVIC** → 勾 `USART2 global interrupt`。
5. ✅ 验收：PA2/PA3 显示为 USART2_TX/RX，且 CH6 仍只属于 TIM1_CH3。

### 7.5 中断优先级

FreeRTOS 用的是 `NVIC_PRIORITYGROUP_4`（4 位抢占、0 位子优先级）。
CubeMX 会给 UART/DMA 生成优先级（默认 5 及以下数值），**保持默认即可**；
只要不把 UART 中断优先级设得比 `configMAX_SYSCALL_INTERRUPT_PRIORITY` 更高（数值更小），
否则在中断里调用 FreeRTOS API 会出问题。本驱动不在 ISR 里调 FreeRTOS API，风险很低。

### 7.6 生成码自检清单（本次已逐项核对）

| 检查项 | 状态 |
|---|---|
| `Core/Src/usart.c`、`Core/Inc/usart.h` | ✅ 已按 CubeMX 6.16 风格生成，含 `MX_USART1/2/3_UART_Init()` 与 `HAL_UART_MspInit/DeInit` |
| `HAL_UART_MspInit` 放哪 | ✅ 放在 `usart.c` 内（本工程 CubeMX 版本就是这种布局：`tim.c` 里也是 `HAL_TIM_PWM_MspInit`），**不是** `stm32f1xx_hal_msp.c` |
| `Drivers/.../stm32f1xx_hal_uart.c/.h` | ✅ 已从节点 A 工程拷入（同一 HAL 版本 1.1.10），已加入 Keil 工程 `Drivers/STM32F1xx_HAL_Driver` 分组 |
| `Core/Inc/stm32f1xx_hal_conf.h` | ✅ `#define HAL_UART_MODULE_ENABLED` 已取消注释 |
| `Core/Src/stm32f1xx_it.c` | ✅ 含 `USART1/2/3_IRQHandler`、`DMA1_Channel3/5_IRQHandler`，并 extern 了 3 个 UART 与 2 个 DMA 句柄 |
| `Core/Src/dma.c` | ✅ `MX_DMA_Init()` 里加了 `DMA1_Channel3/5` 的 NVIC 配置 |
| `Core/Src/gpio.c` | ✅ **无需改动**（KEY_1~KEY_4 都在 GPIOA，CubeMX 重新生成也是同一段代码） |
| `Core/Inc/main.h` | ✅ `KEY_3_Pin=GPIO_PIN_11`、`KEY_4_Pin=GPIO_PIN_12` |
| `Core/Src/main.c` | ✅ 加 `usart.h`/`gps.h` 包含、三个 `MX_USARTx_UART_Init()`、`HAL_UARTEx_RxEventCallback()` 转发；**保持 GBK 编码**（字节校验：非合法 UTF-8，即未变码） |
| `Core/Inc/FreeRTOSConfig.h` | ✅ `configTOTAL_HEAP_SIZE=10240`、`configTIMER_TASK_STACK_DEPTH=128`（与 `.ioc` 一致） |
| `MDK-ARM/startup_stm32f103xb.s` | ✅ `Heap_Size 0x200 → 0x000`（无 malloc 使用者） |
| `Security Watch.uvprojx` | ✅ XML 可解析，分组 8 个、文件项 71 → **79**（+usart.c、+stm32f1xx_hal_uart.c、+gps.c/h、+gps_task.c/h） |

> `.ioc` 是我手改的，**没有经过 CubeMX 校验**；`Generate Code` 时若 CubeMX 报错或自动"修正"了某项配置，
> 以 CubeMX 为准并告诉我，我按它的生成结果同步（尤其是 `FREERTOS.configTIMER_TASK_STACK_DEPTH` 这个键，
> 不同 CubeMX 版本可能不认识，若不认识它会退回默认 256 → RAM 会多用 512 B，仍能装下，但余量变小）。

### 7.7 Keil 侧收尾（本次已完成）

1. ✅ 4 个自写文件已加入工程：`icode/driver` → `gps.c/gps.h`；`icode/task` → `gps_task.c/gps_task.h`。
2. ✅ `Core/Src/main.c` 的 `/* USER CODE BEGIN 4 */` 已补接收事件转发：
   ```c
   void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
   {
     GPS_RxEventCallback(huart, Size);   /* 其它串口会被驱动内部按 huart 过滤掉 */
   }
   ```
3. ✅ `gps_task.c` 顶部句柄 = `&huart1`，与 §4.3 分配一致。
4. ✅ 编译链接已用 `tools\armcc_build_check.py` 验证（59 个源文件 0 错、真机 scatter 链接成功），
   `Program Size` 实测值已填进 §6.4。
5. ⏳ **待你在 Keil 里做**：打开 `MDK-ARM\Security Watch.uvprojx` → F7 构建 → 记录 Keil 自己打印的
   `Program Size`（应与 §6.4 的 32,964 / 19,264 相差很小）；烧录后看 OLED 第 2 行的 `FreeHeap:` 数字
   （期望 1500~2000 之间，若接近 0 说明堆不够）。
6. ⏳ 若某任务"创建了但从不运行"：先看 OLED 的 `FreeHeap:`，再考虑把 `configASSERT` 改成点灯提示
   （当前是关中断死循环，屏幕全黑，极难查）。



---

## 8. 验证流程（从便宜到贵，逐级排除）

| 级别 | 做什么 | 通过标准 |
|---|---|---|
| **L0 台架（不用 MCU）** | USB-TTL 接模块 TXD/RXD，sscom 开 **9600 8N1** | 看到 `$GNRMC,...` / `$GNGGA,...` 周期输出；室外 30~60 s 内 `$GNRMC` 的 status 变 `A` |
| **L1 配置实测** | 在 sscom 里手发 `$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*CS`（校验和让工具算） | 只剩 GGA/RMC 在输出 → 确认参数个数写法（厂家文档有 14/15 两种版本） |
| **L2 电气** | 万用表量 GPS TXD 空闲电平、VDD 电压 | TXD ≈ 3.3 V 空闲高；VDD = 5 V（**不是 3.3 V**） |
| **L3 上板收数（★用 OLED「GPS 接收页」看）** | 把 `oled_task.c` 的 `OLED_SCREEN_SEL` 设成 `OLED_SCREEN_GPS` 后烧录 | 见下面 §8.1 的四层判读 |
| **L4 定位解算** | 同上，等 `FIX` 出现 | `FIX`、`Q1`、`SAT ≥ 4`、经纬度合理；`AGE` 在 0~2 s 之间滚动 |
| **L5 端到端** | 触发一次跌倒（或直接调 `alarm_task` 的确认分支），抓 BLE 报文 | JSON 里带 `"gps":{...}`、`v=1`；把经纬度丢进地图能落到正确位置 |
| **L6 时效性** | 进屋断星 5 分钟后再触发告警 | 报文 `v=0`、`fix_age ≈ 300000`，云端能提示"位置过时"——**这是论文里很有说服力的一组数据** |

### 8.1 GPS 接收页怎么读（联调就靠它，四层定位问题）

```
【第 1 页 / 3：链路 + 语句普查】   【第 2 页 / 3：卫星 · 天线】      【第 3 页 / 3：位置】
GPS LINK      ALIVE             GPS SATELLITE                  GPS POSITION
RX    00070392                  FIX  YES  Q 1                  LAT  30.1875525N
RMC  00158    GGA  00158         ST   11   used                LON 120.8166950E
GSV  00214    GSA  00316         VW   11   view                ALT     12.0 m
OTH  00632    CSE  0000          MX   39   dBHz                HDOP     2.90
LAST POS SENT                   AGE  0000 s                    SPD      0.0 km/h
$GNGGA,073028.00,3011.2         RMC A       GGA 1              UTC  07:30:28
UP/DN=PAGE 1/3                  UP/DN=PAGE 2/3                 UP/DN=PAGE 3/3
```

**第 1 页的"语句普查"是判断"模块到底吐了哪些语句"的唯一直接手段**：
经纬度只在 **GGA/RMC** 里，GSV 只给卫星与信噪比，GSA 只有几何精度。
**`RMC`/`GGA` 计数恒为 0 → 定位永远出不来**，说明模块的输出语句集被改乱了
（常见于之前在 PC 上用 sscom 发过 `$PCAS03`，该配置保存在模块备电 RAM 里）。
`gps_task.c` 已内置**自动修复**：检测到"有字节进来但 RMC/GGA 恒为 0"时，
自动发一次"全部语句打开"（`$PCAS03,1,1,1,1,1,1,1,1,1,1,,,1,1*02`），最多 3 次、间隔 5 s。

同一页的 `LAST POS SENT` 显示的是**最近一条 RMC/GGA 原文**（不是"最近一条任意语句"）：
实测 `GPTXT`（天线状态提示）占了 1478 条里的 632 条，若显示"任意语句"，
排查字段问题时几乎永远看不到定位语句。字段解析出问题时，把这一行原文发出来即可定位。

**翻页：按板上"上/下"两个键**（KEY_1=上=上一页、KEY_2=下=下一页，循环；长按可连发快速翻）。
翻页用的是按键驱动里为导航单独做的锁存 `Key_TakeNav()`，**不占用事件队列** ——
`alarm_task` 那条"长按返回 = 取消告警"的队列消费因此完全不受影响。

**首板实测记录（2026-10-13）**：
- 第 1 版照片（室内）：`ALIVE`、`RX 118528`、`CSE 0000`，最新语句恰好是 `$GNGSA,...`。
- 第 2 版照片（室外，**正式通过**）：
  - 第 2 页：**`FIX YES` / `Q1` / `ST 11 used` / `VW 06 view` / `MX 39 dBHz` / `RMC A  GGA 1`**
  - 第 3 页：**`LAT 30.18755N` / `LON 120.81670E` / `ALT 12.0 m` / `HDOP 2.90` / `SPD 0.0 km/h`**
  - 第 1 页：`RX 70392`、`RMC 158`、`GGA 158`、`GSV 214`、`GSA 316`、`OTH 632`、`CSE 0000`
  → **串口 → NMEA → 字段解析 → 定点显示 → 定位有效判定，全链路打通**（杭州一带坐标、MX 39 dBHz 说明天线良好）。

⚠ **复盘：这一轮抓出的三个真 bug（都值得写进论文的"调试与验证"一节）**

| # | 现象 | 根因 | 修法 |
|---|---|---|---|
| 1 | 有正确经纬度却一直 `FIX NO` / `Q 0`，告警 `v` 恒为 0 | `valid` 判定写成 `has_rmc && status && quality>0`，**要求必须收到 RMC** | 改为 `(has_rmc && status) \|\| (has_gga && quality>0)` |
| 2 | `UTC` 恒为 `00:00:00`（`no time`） | `GPS_ParseUtcMs()` 对定长 2 位字段用了通用的 `GPS_ParseUint()`（吃掉所有数字）：`"073028.00"` → `hh=73028` → 范围检查 `hh>23` 直接 return 0。**日期 `ddmmyy` 同理**（`mm=724>12`） | 新增 `GPS_Parse2()`：定长 2 位、非法返回 0xFF；UTC 与日期都改用它 |
| 3 | `ST 11 used` 却 `VW 06 view`（参与定位比可见还多，自相矛盾） | GPS 与北斗各自发 `GSV`（`$GPGSV`/`$GBGSV`），原实现用"最后一句"覆盖，两个星座互相盖掉 | `s_view[4]` 按星座分别记录，`VW` = 各星座之和 |

**方法论教训（比 bug 本身更重要）**：`tools/gps/verify_nmea_math.py` 当时**没能发现 #2**，
因为脚本里的镜像按"我打算怎么写"（`s[0:2]` 切片）实现，而 C 里实际用的是通用解析器 ——
**镜像与实现语义分叉，验证就失效了**。现已把镜像逐句对齐 C，并加了回归护栏
（断言"通用解析器会把 `073028.00` 读成 73028、定长解析器读成 7"，谁把实现改回去就会报警）。

| 现象 | 结论 | 下一步 |
|---|---|---|
| **屏幕显示经纬度但 `FIX` 是 `NO`** | 定位数据其实有效，是 `valid` 判定或语句组合的问题 | 2026-10-13 已修：`valid` 原先要求必须收到 RMC；现改为 RMC 或 GGA 任一有效即可。看第 2 页新增的 `RMC A/V`、`GGA 0/1` 两格即可确认 |
| `UTC` 显示 `00:00:00`（或 `no time`） | 时间字段（RMC/GGA 的第 1 个字段）没解析出值 | 看第 1 页 `LAST SENTENCE` 原文：若 `$GNRMC,,V,...` 里时间是空的 → 模块还没时间同步（无定位时常见）；若有时间却没显示 → 是解析问题，把原文发我按字段下标改 |
| `RX` 停在 0 | 一个字节都没收到 | 查 GPS **TXD → PB7**、**RXD → PB6**、共地、**VDD=5 V**（底板 3.6~6 V，3.3 V 不工作）；再看模块 TX 灯是否闪 |
| `RX` 涨但 `SEN` 不涨、`CSE` 猛涨 | 收到的是噪声，凑不出合法校验和 | **波特率不对**（模块若不是 9600，用 sscom 发 `$PCAS01,1` 改回）或电平不对 |
| **`VW`/`MX` 都是 0（室内常见，**室外也出现就必须查**）** | `VW VW` 空 → 射频确实没收到信号 | 按顺序查：① **模块 VDD 是不是 5 V**（底板要 3.6~6 V，用 3.3 V 会出现"串口正常但灵敏度不足"这种最难查的现象）；② **天线实物**：板载陶瓷天线是否完整/开焊、有没有被金属或手挡住、若板子是 IPEX 座则必须插天线；③ **用 PC 直连模块**（USB-TTL + sscom 9600 8N1，或厂家 `MobiletekGNSSTool`）看有没有 GSV/信噪比 —— 这一步能把"模块/天线问题"与"我们固件问题"彻底分开 |
| `VW` 有值、`MX` 20~45，但仍 `NOF` | 天线在工作，只是还没凑够定位条件 | 继续等（冷启动 30~60 s 起）；`ST ≥ 4` 且 `Q` 变 1 才出 `FIX` |
| 模块反复输出 `$GPTXT,...,ANTENNA OPEN*25` | **正常，不是故障** | 技术手册 §2.12.3：**用无源（陶瓷）天线时模块就报开路** —— DX-GP22-A 板载陶瓷天线即属此类。判断天线好坏请看 `VW`/`MX`，别看这条 |
| `FIX` 出现但经纬度是 0 | 解析异常 | 把 `LST` 那两行原文发我，对照字段下标排查 |
| `OK` 变成 `--` | 5 s 内没收到任何字节，任务已在自动 `GPS_Reinit()` | 与 `RX` 停在 0 同样处理（多半接触不良或供电跌落） |

> **室内定位这件事要在论文里写成设计取舍**：GNSS 信号约 -130 dBm，混凝土/钢筋/金属镀膜玻璃会把它衰减到收不到，
> 所以"室内无定位"是物理限制而非缺陷。工程上的应对正是本方案已有的两条：
> ① 报文用 `v=0` + `fix_age` 表达"最后已知位置及其新鲜度"（屏幕上的 `A025s` 就是它）；
> ② 演示前先在室外/窗边完成首次定位，**保持模块供电**再进屋演示跌倒告警 —— 位置仍在，`fix_age` 会如实增长。
> 想让后续每次都秒级定位，给模块 **VBAT 接 3 V 备电**（热启动，TTFF 从 30~60 s 降到数秒）。

---

## 9. 风险与对策

| 风险 | 概率 | 影响 | 对策 |
|---|---|---|---|
| 室内/身上收不到星 | **高** | 演示时无位置 | 演示前室外预定位；报文用 `v=0 + fix_age` 表达"最后已知位置"；手机定位作为交叉校验（原方案保留） |
| 底板误用 3.3 V 供电 | 中 | 模块不启动/反复复位 | 严格 3.6~6 V，推荐 5 V；上电前万用表确认 |
| GPS 的 32 mA 拖垮续航 | 中 | 可穿戴形态撑不住 | VBAT 备电 + 周期性开关（`GPS_Power()`，冷启动由热启动替代）；论文里做"定位开启占空比 vs 续航"实验 |
| `$PCAS03` 参数个数写法不符被忽略 | 中 | 输出语句没精简，串口流量变大（功能不受影响） | L1 先实测；驱动对多余语句只会忽略，不影响定位解析 |
| 三路 UART 都开导致 Flash/RAM 超 | ~~低~~ **已发生并已解决** | — | 实测加完 GPS+UART 后 RAM 到 22.72 KB（超 2.24 KB），链接失败；按 §6.4 的 4 项裁剪后为 18.81 KB，余 1.22 KB。Flash 只用了 32.19 KB / 64 KB，很宽裕 |
| CubeMX 重新生成覆盖自写文件 | 低 | 丢代码 | 自写文件全在 `MDK-ARM\icode\` 下，CubeMX 不碰该目录；`freertos.c` 的修改都在 USER CODE 区 |
| **重新生成把 `configTOTAL_HEAP_SIZE` / 定时器栈改回默认** | 中 | 堆不足 → 任务静默创建失败 | `.ioc` 里已写入 `configTOTAL_HEAP_SIZE=10240` 与 `configTIMER_TASK_STACK_DEPTH=128`；生成后回查 `FreeRTOSConfig.h` 这两个值（旧版 CubeMX 若不认定时器栈这个键，会退回 256 → 多用 512 B RAM，仍装得下） |
| 重新生成把 GBK 的 `main.c` 重写成 UTF-8 | 中 | 已有的中文注释乱码 | 已备份 `_backup_before_usart\`；乱码只影响注释，把 `USER CODE` 区自己写的代码贴回即可 |
| DMA 通道冲突 | 低 | 接收不工作 | 已规避：USART2 接收不用 DMA；且 ws2812b 删除后 CH6 已释放（§7.4） |

---

## 10. 论文里可以直接用的素材

1. **设计取舍论证**：设备端自采定位 vs 手机定位——定位可用性、功耗、成本、隐私、脱离手机可用性五个维度对比（有了本模块，"手机中继"章节从"因为没有硬件"变成"有意让手机只做 4G 网关"）。
2. **资源占用实测表**：GPS 驱动 + UART HAL 的 Flash/RAM 增量（编译后填 §6.4）。
3. **定位性能实验**：冷启动 vs 热启动 TTFF（VBAT 有无）、开阔地/靠窗/室内三档的 `sat`、`hdop`、定位有效率与 `fix_age` 分布。
4. **链路可靠性实验**：长时间运行下 `GPS_GetSentenceCount()`、`GPS_GetCsumErrCount()`、`GPS_GetOverrunCount()` 统计（体现"校验和 + 环形缓冲 + 溢出丢弃"的工程价值）。
5. **报文设计**：定点整数标度、`fix_age` 语义、断连缓存补传机制。
6. **算法层面**：整数定点坐标换算（`e7 = deg*1e7 + (min_e5*5+1)/3`，误差 < 4 mm）可用一节讲"无 FPU MCU 上的数值处理"。

---

## 11. 待办清单（勾掉即完成）

- [x] 解析厂家 3 份 PDF（引脚/供电/功耗/波特率/$PCA 命令/NMEA 字段）
- [x] 审查厂家例程并列出不可照抄之处（§2.1）
- [x] 落地 GPS 驱动 `icode/driver/gps.c/h`（NMEA 解析 + 定点 + seqlock + $PCA + 诊断）
- [x] 落地 GPS 任务 `icode/task/gps_task.c/h`，并接入 `freertos.c`
- [x] PC 端算法自证脚本 `tools\gps\verify_nmea_math.py`（5 组用例全通过，并借此修掉字段下标缺陷）
- [x] 更新 `00-方案与计划.md`（§0 / §1.1 / §3(4) / §7 / §8 / §9 / §10）与 `ble_task` 报文注释
- [x] **改 `.ioc`**：USART1(重映射 PB6/PB7, 9600, DMA1_CH5) + USART2(PA2/PA3, 9600) + USART3(PB10/PB11, 115200, DMA1_CH3)、
      KEY_3/KEY_4 迁至 PA11/PA12、NVIC/DMA/functionlistsort、FreeRTOS 堆与定时器栈、C 库堆清零
- [x] **按 CubeMX 6.16 风格生成代码**：`usart.c/h`、`HAL_UART_MspInit`、`stm32f1xx_it.c` 中断、`dma.c` NVIC、
      `hal_conf.h` 打开 UART 模块、拷入 HAL UART 驱动、`main.c`(保 GBK) 加初始化与接收回调
- [x] **Keil 工程**：`uvprojx` 加入 usart.c / stm32f1xx_hal_uart.c / gps.c / gps.h / gps_task.c / gps_task.h（71→79 项）
- [x] **编译链接校验**：`tools\armcc_build_check.py` → 59 个源文件 0 错、真机内存布局链接成功
- [x] **RAM 超限修复**：22.72 KB → 18.81 KB（4 项裁剪，见 §6.4），并在 `MX_FREERTOS_Init()` 里加了 `FreeHeap:` 显示
- [ ] **在 CubeMX 里打开 `.ioc` 复核**（§7 的 5 条清单），必要时 Generate Code 并对比生成的 `usart.c`
- [ ] **在 Keil 里 F7 构建**并记录 `Program Size`（预期与 §6.4 的 32,964 / 19,264 接近）
- [ ] **L0 台架验证**：sscom 收到 NMEA（§8）
- [ ] **L1 实测 `$PCAS03`**，把是否接受写回 `gps.c` 注释
- [ ] L3~L6 逐级验证，并把数据记入 `docs\实验记录.md`

---

## 12. 本次改动清单（文件级，便于复核与回退）

### 12.1 新增文件

| 文件 | 作用 | 规模 |
|---|---|---|
| `Security Watch\MDK-ARM\icode\driver\gps.c` / `gps.h` | DX-GP22/ATGM336H NMEA0183 驱动（DMA+空闲中断接收、语句装配、校验和、RMC/GGA 整数解析、seqlock 快照、`$PCA` 配置、诊断、可选电源门控） | 1,986 B Flash / 624 B RAM |
| `Security Watch\MDK-ARM\icode\task\gps_task.c` / `gps_task.h` | GPS 任务（20 ms 排空缓冲、发配置命令、5 s 无字节自动重启接收） | 120 B Flash / 栈 512 B |
| `Security Watch\Core\Src\usart.c` / `Core\Inc\usart.h` | 三路串口初始化 + `HAL_UART_MspInit/DeInit`（CubeMX 6.16 风格） | 624 B Flash / 352 B RAM |
| `Security Watch\Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_uart.c`、`Inc\stm32f1xx_hal_uart.h`、`Inc\stm32f1xx_ll_usart.h` | 从节点 A 工程拷入（同 HAL 1.1.10），原工程未启用 UART 模块 | 2,106 B Flash |
| `tools\gps\verify_nmea_math.py` | NMEA 整数算法自证（对照厂家 PDF 原句） | — |
| `tools\armcc_build_check.py` | 不依赖 Keil 的完整编译+链接+占用核算 | — |
| `Security Watch\_backup_before_usart\` | 改造中途的备份快照（**不是**原始工程，仅作对照） | — |

### 12.2 修改文件

| 文件 | 改了什么 |
|---|---|
| `Security Watch.ioc` | 加 USART1/2/3（含 DMA/NVIC/引脚/波特率）；KEY_3/KEY_4 → PA11/PA12；PA2/PA3 → USART2；PB6/PB7 → USART1(重映射)；PB10/PB11 → USART3；删 `FREERTOS.Tasks01`；`configTOTAL_HEAP_SIZE=10240`；`configTIMER_TASK_STACK_DEPTH=128`；`ProjectManager.HeapSize=0x0`；`functionlistsort` 加三个 USART 初始化 |
| `Core\Src\main.c`（**保持 GBK**） | `#include "usart.h"` / `"gps.h"`；`MX_USART1/2/3_UART_Init()`；`HAL_UARTEx_RxEventCallback()` 转发 |
| `Core\Src\freertos.c` | `GPS_TASK_PRIO/STACK` 与 `GPS_Task_Init()`；删掉空转 `defaultTask`；OLED 栈 512→256、GPS 栈 192→128；`MX_FREERTOS_Init()` 里增加 OLED `FreeHeap:` 显示 |
| `Core\Inc\main.h` | `KEY_3_Pin=GPIO_PIN_11`、`KEY_4_Pin=GPIO_PIN_12` |
| `Core\Src\stm32f1xx_it.c` | 新增 `USART1/2/3_IRQHandler`、`DMA1_Channel3/5_IRQHandler` 与 extern 句柄 |
| `Core\Src\dma.c` | `MX_DMA_Init()` 增加 `DMA1_Channel3/5` 的 NVIC 配置 |
| `Core\Inc\stm32f1xx_hal_conf.h` | 打开 `HAL_UART_MODULE_ENABLED` |
| `Core\Inc\FreeRTOSConfig.h` | 堆 12288→10240、定时器栈 256→128（并写明依据） |
| `MDK-ARM\startup_stm32f103xb.s` | `Heap_Size 0x200 → 0x000` |
| `MDK-ARM\Security Watch.uvprojx` | 加入 usart.c、stm32f1xx_hal_uart.c、gps.c/h、gps_task.c/h |
| `MDK-ARM\icode\task\ble_task.c` / `ble_task.h` | 报文示例增加 `gps` 片段与标度说明（GPS 由设备端自采，不再靠手机） |
| `MDK-ARM\icode\task\gps_task.c` | 句柄指向 `&huart1`（与 §4.3 分配一致） |
| `00-方案与计划.md` | §0 / §1.1 / §3(4) / §7 / §8 / §9 / §10 与 GPS 相关的表述改为"设备端自采定位" |

### 12.3 想回退怎么办

| 想退掉 | 怎么做 |
|---|---|
| 只退 GPS 功能 | 把 `gps.c/h`、`gps_task.c/h` 从 Keil 工程移除，删掉 `freertos.c` 里的 `GPS_Task_Init` 与 `main.c` 里的转发即可；其余（三路串口）可留着 |
| 退掉整次改造 | 用 `_backup_before_usart\` 对照恢复 `Core\`、`.ioc`、`uvprojx`；或按 §12.1/§12.2 反着操作（.ioc 里删 USART、把 KEY_3/4 放回 PA2/PA3） |
| 退掉 RAM 裁剪 | 把 `configTOTAL_HEAP_SIZE` 调回 12288、定时器栈调回 256、startup `Heap_Size` 调回 0x200、`gps.h` 缓冲调回 256/512 —— 但**会重新超出 20 KB RAM**，必须同步砍掉 TinyML 滑窗（`tinyml_task.c` 的 `s_ring`，2,412 B）或换更小的窗口 |

