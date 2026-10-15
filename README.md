# 可穿戴跌倒检测与定位告警系统（江彬锋 毕设）

基于 STM32F103C8T6 的可穿戴跌倒检测系统：**设备端本地推理判跌倒 → GPS 定位 → 蓝牙上报手机 → 云端地图打点**。
本仓库是主控节点（节点 B）的固件工程、驱动、文档与验证工具。

---

## 1. 系统结构

| 角色 | 硬件 | 说明 |
|---|---|---|
| **节点 A** | STM32 + ICM42688 + MMC5603 | 九轴姿态采集，通过 UART 把姿态/加速度帧发给节点 B（另一工程，路径在本仓库之外） |
| **节点 B**（本仓库） | STM32F103C8T6（64 KB Flash / 20 KB RAM） | 跌倒识别（TinyML 定点推理）、GPS 定位、蓝牙上报、OLED 交互、按键菜单、蜂鸣告警 |

**节点 B 的三路串口分工**（互不干扰，`main.c` 按 `huart->Instance` 分发接收回调）：

| 串口 | 引脚 | 外设 | 参数 |
|---|---|---|---|
| USART1 | PB6/PB7（重映射） | **GPS** DX-GP22-A（ATGM336H） | 9600 8N1，循环 DMA + 空闲中断 |
| USART2 | PA2/PA3 | **HC-08** BLE 透传 | 9600 8N1，空闲中断 + 中断接收（不用 DMA） |
| USART3 | PB10/PB11 | **节点 A 链路 / DIY IMU** | 115200 8N1，DMA + 空闲中断 |

## 2. 目录结构

```
├── 00-方案与计划.md            总方案、里程碑、RAM/引脚预算
├── docs/                       设计文档（见下表）
├── tools/                      验证与代码生成工具（Python，不需要 Keil）
├── Security Watch/             节点 B 固件工程（Keil MDK-ARM）
│   ├── Core/                   CubeMX 生成区（main.c / usart.c / gpio.c / freertos.c ...）
│   ├── Drivers/ Middlewares/   ST HAL / CMSIS / FreeRTOS 源码
│   ├── Security Watch.ioc      CubeMX 工程文件（本机未装 CubeMX，为手工维护）
│   └── MDK-ARM/
│       ├── Security Watch.uvprojx   Keil 工程
│       └── icode/                   ★ 本项目自己写的代码
│           ├── driver/   gps / hc08 / key / diy_imu / oled / led ...
│           └── task/     gps_task / ble_task / alarm_task / oled_task / tinyml_task ...
└── Smart Home/                 起步参考工程（江协科技智能家居），保留以便溯源
```

## 3. 文档索引（`docs/`）

| 文档 | 内容 |
|---|---|
| `GPS模块接入方案.md` | GPS 模块事实、UART/DMA 分配、驱动设计、CubeMX 步骤、验证分级 L0~L6、**实测记录与 bug 复盘** |
| `HC-08接入方案.md` | 蓝牙模块接线（⚠ 3.3 V）、AT 指令、通信协议、验证分级 L0~L4、排查表 |
| `按键接线与引脚分配.md` | 4 个按键接线（含四脚开关坑）、事件 API、**完整引脚分配表** |
| `OLED显示指标说明.md` | 屏幕上每个指标的含义/单位/正常范围/异常判读（速查） |
| `DIY_IMU接入说明.md` | 节点 A 链路帧协议、RAM 账本 |
| `Security Watch 工程骨架说明.md` | 各驱动/任务职责与依赖关系 |

## 4. 编译与验证

**Keil MDK-ARM**：打开 `Security Watch/MDK-ARM/Security Watch.uvprojx`，F7 编译，下载即可。
当前实测占用：**Flash 39.2 KB / RAM 18.7 KB**（上限 64 KB / 20 KB），FreeRTOS 堆余量约 2 KB。

**不装 Keil 也能验证**（`tools/`，需要 ARMCC 在 `E:\Keil5\ARM\ARMCC\bin`）：

```bash
python tools/armcc_build_check.py     # 真编译 + 真链接 + 报 Flash/RAM 占用
python tools/heap_ledger.py           # 从源码算 FreeRTOS 堆账本（余量是否够）
python tools/gps/verify_nmea_math.py  # NMEA 解析算法自证（含 20 万组随机模糊测试）
python tools/gen_key_wiring_figure.py # 生成按键接线图 PNG
```

## 5. 进度

| 模块 | 状态 |
|---|---|
| GPS（USART1） | ✅ 已上板定位成功（FIX / 11 颗星 / HDOP 2.9 / 坐标正确），三页接收页 + 语句普查 |
| 按键（PA0/PA1/PA11/PA12） | ✅ 接线与检测页已验证 |
| HC-08（USART2） | ✅ 已上板验证：手机端收到 34 字节（`AT` + `BOOT` 报文），链路打通；双向命令与告警上报待联调 |
| 跌倒识别（TinyML） | 🔧 驱动与权重已就位；**节点 A 发帧已修正为严格 100 Hz**（见 `docs/DIY_IMU接入说明.md` §5），待与告警状态机联调 |
| 告警状态机 / 云端 | ⏳ 待做 |
| 菜单/状态总览 | ✅ 四页（总览 / 节点A / GPS / BLE），上/下键翻页；**默认屏就是一屏看三条链路通没通**（见 `docs/菜单与交互设计.md`） |

## 6. 注意

- **文件编码**：`Core/Src/main.c`、`icode/driver/key.c`、`key.h` 是 **GBK**（Keil 默认编辑器），
  其余多为 UTF-8。改动这些文件请勿整文件转码，否则 Keil 里中文注释会乱码。
- **`ws2812b` 驱动已删除**（无任务调用，且占用 640 B 静态 RAM + PA10 + DMA1_CH6），
  需要灯环效果时从 git 历史取回。
- **`configTOTAL_HEAP_SIZE = 8192`**：`.ioc` 里必须同步该值，否则 CubeMX 重新生成会改回去。
- 本机**未安装 STM32CubeMX**，`.ioc` 与 CubeMX 风格代码由手工维护（改动时保持 6.16 版风格）。
