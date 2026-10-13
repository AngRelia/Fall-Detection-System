# Security Watch 工程骨架说明

来源：由原 `Smart Home` 智能家居工程清空改造而来（原工程已保留，未改动）。
对象：`Security Watch\`（可穿戴跌倒与疲劳检测系统的**主控节点 B** 固件骨架）

---

## 1. 本次做了什么

| 项目 | 改造前 | 改造后 |
|---|---|---|
| 工程名 | `Smart Home` | `Security Watch` |
| 器件 | STM32F103C8 | 不变（64 KB Flash / 20 KB RAM） |
| 编译器 | Keil AC5 (`uAC6=0`) | 不变 |
| `freertos.c` | **1295 行**，塞了 7 个任务 + 全部全局变量 | **165 行**，只集中定义优先级并调用各任务的 `_Task_Init()` |
| 自写驱动位置 | `MDK-ARM\icode\` | `MDK-ARM\icode\driver\` |
| 任务位置 | 全部挤在 `freertos.c` | `MDK-ARM\icode\task\` |
| 驱动数量 | 11 个 | 7 个（删掉 4 个与可穿戴无关的） |
| 空任务模板 | 无 | 6 个标准模板 |
| `configTOTAL_HEAP_SIZE` | 8192（不够） | **12288** |

---

## 2. 当前目录结构

```
Security Watch\
├── Security Watch.ioc              CubeMX 工程（已同步改名）
├── Core\
│   ├── Inc\                        CubeMX 生成的头文件
│   └── Src\
│       ├── main.c                  初始化外设 → osKernelInitialize → MX_FREERTOS_Init
│       ├── freertos.c              ★ 只定义优先级 + 调用各任务 _Task_Init()
│       └── ...                     gpio / tim / adc / rtc / dma / stm32f1xx_it
├── Drivers\                        ST 官方 HAL 与 CMSIS
├── Middlewares\Third_Party\FreeRTOS\   FreeRTOS 内核（CMSIS_RTOS_V2，heap_4）
└── MDK-ARM\
    ├── Security Watch.uvprojx      Keil 工程
    ├── icode\
    │   ├── driver\                 ★ 底层驱动（7 个）
    │   │   ├── delay.c/h           延时
    │   │   ├── OLED.c/h            软件 I2C OLED
    │   │   ├── OLED_Data.c/h       字库
    │   │   ├── key.c/h             按键
    │   │   ├── led.c/h             PWM 三路调光
    │   │   ├── beep.c/h            蜂鸣器（低电平触发）
    │   │   └── (ws2812b.c/h 已删除) WS2812 灯效驱动无任务调用，2026-10-14 移出工程并删除源码
    │   └── task\                   ★ 任务模块（6 个）
    │       ├── imu_recv_task.c/h   串口接收解析
    │       ├── tinyml_task.c/h     模型推理
    │       ├── fatigue_task.c/h    疲劳判定
    │       ├── alarm_task.c/h      告警与取消
    │       ├── ble_task.c/h        BLE 上报
    │       └── oled_task.c/h       显示
    └── startup_stm32f103xb.s
```

### 已删除的驱动（及原因）

| 驱动 | 删除原因 |
|---|---|
| `my_encoder` | 旋转编码器，依赖 TIM2 编码器接口；可穿戴项目无此交互 |
| `dht11` | 温湿度传感器，智能家居专用 |
| `light_sensor` | 光敏电阻 ADC，智能家居专用 |
| `mq2` | 可燃气体传感器，智能家居专用 |

> `gpio.c` 与 `main.h` 里仍留有这些模块的**引脚宏定义**（如 `DHT11_Pin`/`Encoder_button_Pin`），这是 **CubeMX 生成的代码，故意未动**：一来对编译无影响，二来保留 `.ioc` 与生成代码的一致性，日后在 CubeMX 里重新勾选引脚时不会冲突。要清理的话，正确做法是在 CubeMX 里改 `.ioc` 后重新生成，而不是手改这些文件。

---

## 3. 任务如何创建（本框架的核心）

### 职责划分

- **`freertos.c`**：定义优先级与栈大小，调用各任务的 `_Task_Init()`
- **`icode\task\xxx_task.c`**：自己实现 `_Task_Init()` 完成创建，并实现任务入口

### freertos.c 里集中定义

```c
/* 优先级 */
#define IMU_RECV_TASK_PRIO      osPriorityAboveNormal
#define ALARM_TASK_PRIO         osPriorityAboveNormal
#define TINYML_TASK_PRIO        osPriorityNormal
#define BLE_TASK_PRIO           osPriorityNormal
#define FATIGUE_TASK_PRIO       osPriorityBelowNormal
#define OLED_TASK_PRIO          osPriorityLow

/* 栈深度，单位 word（1 word = 4 字节） */
#define IMU_RECV_TASK_STACK     256
#define TINYML_TASK_STACK       256
#define FATIGUE_TASK_STACK      256
#define ALARM_TASK_STACK        256
#define BLE_TASK_STACK          256
#define OLED_TASK_STACK         512
```

然后在 `MX_FREERTOS_Init()` 里：

```c
  /* USER CODE BEGIN RTOS_THREADS */
  IMU_Recv_Task_Init(IMU_RECV_TASK_PRIO, IMU_RECV_TASK_STACK);
  TinyML_Task_Init(TINYML_TASK_PRIO, TINYML_TASK_STACK);
  Fatigue_Task_Init(FATIGUE_TASK_PRIO, FATIGUE_TASK_STACK);
  Alarm_Task_Init(ALARM_TASK_PRIO, ALARM_TASK_STACK);
  BLE_Task_Init(BLE_TASK_PRIO, BLE_TASK_STACK);
  OLED_Task_Init(OLED_TASK_PRIO, OLED_TASK_STACK);
  /* USER CODE END RTOS_THREADS */
```

### task 模块里的固定写法

```c
/* 任务句柄 */
TaskHandle_t AlarmTaskHandle = NULL;

void Alarm_Task_Init(osPriority_t prio, uint32_t stack)
{
  BaseType_t ret = xTaskCreate(Alarm_Task_Entry,
                               "Alarm",
                               (configSTACK_DEPTH_TYPE)stack,
                               NULL,
                               (UBaseType_t)prio,
                               &AlarmTaskHandle);
  configASSERT(ret == pdPASS);   /* 堆不足时静默失败，必须拦住 */
}

void Alarm_Task_Entry(void *argument)
{
  (void)argument;
  /* USER CODE BEGIN Alarm_Task_Entry_Init */
  /* USER CODE END Alarm_Task_Entry_Init */

  for (;;)
  {
    /* USER CODE BEGIN Alarm_Task_Entry_Loop */
    /* 你的代码写在这里 */
    /* USER CODE END Alarm_Task_Entry_Loop */
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
```

### 增删任务的步骤

**加一个任务**：
1. 在 `icode\task\` 复制一份现成模板，改名字
2. 把新 `.c/.h` 加进 Keil 工程的 `icode/task` 分组
3. 在 `freertos.c` 加两行宏定义（`XXX_TASK_PRIO` / `XXX_TASK_STACK`）+ 在 `RTOS_THREADS` 里加一行 `XXX_Task_Init(...)`
4. 确认 `configTOTAL_HEAP_SIZE` 还有足够余量

**改优先级**：只改 `freertos.c` 里那一行宏，不用碰任何任务文件。

---

## 4. 当前优先级分配与理由

| 任务 | 优先级 | 数值 | 理由 |
|---|---|---|---|
| `IMU_Recv` | `osPriorityAboveNormal` | 32 | 数据先落地，优先级低了会丢帧 |
| `Alarm` | `osPriorityAboveNormal` | 32 | 取消按键必须响应及时，晚了就成误报 |
| `TinyML` | `osPriorityNormal` | 24 | 推理耗时可控，单次目标 < 20 ms |
| `BLE` | `osPriorityNormal` | 24 | 上报允许有几十毫秒延迟 |
| `Fatigue` | `osPriorityBelowNormal` | 16 | 1~2 Hz 低频任务，不急 |
| `OLED` | `osPriorityLow` | 8 | 软件 I2C 刷屏最慢，绝不能抢 CPU |

`osPriority_t` 数值参考：`Low=8`、`BelowNormal=16`、`Normal=24`、`AboveNormal=32`、`High=40`、`Realtime=48`。

---

## 5. ⚠️ 堆容量（必须注意）

```
FreeRTOSConfig.h:  configTOTAL_HEAP_SIZE = 12288 字节   ← 已从 8192 调大
```

任务栈是从这个堆里**动态分配**的。实测测算：

```
6 个应用任务        6 × 512 = 3072
空闲任务            128 × 4 =  512
定时器任务          256 × 4 = 1024
6 个 TCB + 定时器队列       ≈  660
────────────────────────────────────
合计                         ≈ 5.3 KB
```

原来的 8192 字节在加满 6 个任务后余量已很紧，故提到 12288，留出约 1 KB 给队列/信号量。

**注意**：堆不足时 `xTaskCreate` 返回 `pdFAIL`，任务**静默不运行**——没有任何报错，极难排查。为此 `_Task_Init()` 里都加了 `configASSERT(ret == pdPASS)`。但当前 `configASSERT` 的实现是：

```c
#define configASSERT( x ) if ((x) == 0) {taskDISABLE_INTERRUPTS(); for( ;; );}
```

即**关中断后死循环**——会在 `OLED_Init()` 之前的启动阶段卡住，屏幕上什么都看不到。建议后续在 `freertos.c` 里重定义 `configASSERT`，改成点亮 LED 或打印提示，便于定位。

再加任务时若堆不够，把 `configTOTAL_HEAP_SIZE` 继续调大即可（F103 有 20 KB RAM，可到 15360）。

---

## 6. 待确认的两件事

1. **编译验证**：我在沙箱内无法启动 Keil UV4（命令行构建返回 `0xC0000409` 栈缓冲区溢出；用**未改动的原工程**做对照试验也是同样结果，确认是沙箱限制而非工程文件问题）。
   请你用 Keil 打开 `MDK-ARM\Security Watch.uvprojx`，直接 F7 编译。预期 0 Error；若报错请把错误贴给我。

2. **`led.c`（PWM 呼吸灯）**：需要一路三通道 PWM 定时器（原工程用 TIM3 CH2/3/4），保留但**当前无任务调用**。
   注：`ws2812b.c/h` 已于 2026-10-14 删除（同样无调用者，且其 640 B 静态缓冲与 PA10/DMA1_CH6 都是占用），
   需要灯环效果时从 git 历史或备份中取回即可。

---

## 7. 本次改造的校验结果

```
✓ 6 个任务的头文件声明与源文件定义完全匹配（_Task_Init / 句柄）
✓ freertos.c 中 6 个 _Task_Init 调用、对应的优先级宏、栈宏全部存在
✓ 所有 USER CODE 标记配对（每个文件 9 对）
✓ 无重复分区骨架
✓ 分区注释框宽度与 CubeMX 原生风格一致（80 字符）
✓ 旧 X-Macro 写法（SG_TASK_TABLE 等）已完全清除
✓ uvprojx / uvoptx 通过 XML 解析校验
```

## 8. 复现改造的脚本

`tools\rebuild_keil_project.ps1` 记录了工程改写逻辑（项目改名、分组拆分、文件编号重建、头文件路径更新）。要把另一份 Smart Home 风格工程改成同样的骨架，改一下脚本顶部的路径即可。
