# -*- coding: utf-8 -*-
"""
heap_ledger.py —— 从源码直接算出 FreeRTOS 堆账本（不需要 Keil / armcc）

为什么需要它：
  本工程 RAM 只有 20 KB，FreeRTOS 堆（configTOTAL_HEAP_SIZE）是个**固定的 .bss 数组**，
  所以"多加一个任务/加大一个栈"到底会不会把堆用穿，必须能随时算清楚。
  这个脚本读两处真源：
    · Core\\Src\\freertos.c           —— 每个任务的栈大小宏 + **实际创建了哪些任务**
    · Core\\Inc\\FreeRTOSConfig.h    —— 堆大小 / 空闲栈 / 定时器任务与队列
  然后按 heap_4 的真实开销（每次分配 8 B 块头、8 B 对齐）算出占用与余量。

用法：
    python tools\\heap_ledger.py
"""

import io
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJ = os.path.join(ROOT, "Security Watch")
FREERTOS_C = os.path.join(PROJ, "Core", "Src", "freertos.c")
CONFIG_H = os.path.join(PROJ, "Core", "Inc", "FreeRTOSConfig.h")

TCB_BYTES = 96      # heap_4 实测：TCB + 8 B 块头（见 FreeRTOSConfig.h 的账本注释）
MALLOC_HDR = 8      # heap_4 每次分配的块头


def read(path):
    return io.open(path, encoding="utf-8", errors="replace").read()


def cfg(text, name, default=None):
    # 形如： #define configTOTAL_HEAP_SIZE   ((size_t)9280)
    #        #define configMINIMAL_STACK_SIZE ((uint16_t)128)
    # 注意用 (?<![A-Za-z_]) 排除强制转换里的数字（否则会从 uint16_t 里抓到 "16"）
    m = re.search(r"#define\s+%s\b[^\n]*?(?<![A-Za-z_])(\d+)" % re.escape(name), text)
    return int(m.group(1)) if m else default


def main():
    fc = read(FREERTOS_C)
    ch = read(CONFIG_H)

    # 1) 栈大小宏（键是完整宏名，如 IMU_RECV_TASK_STACK）
    stacks = {m.group(1): int(m.group(2))
              for m in re.finditer(r"#define\s+(\w+_TASK_STACK)\s+(\d+)", fc)}

    # 2) 实际创建了哪些任务（把 _Task_Init 调用里被注释掉的行排除）
    created = []
    for ln in fc.split("\n"):
        s = ln.strip()
        if s.startswith("//") or s.startswith("*") or s.startswith("/*"):
            continue
        m = re.search(r"\b(\w+)_Task_Init\s*\(", s)
        if m:
            created.append(m.group(1))

    # 3) 配置
    heap = cfg(ch, "configTOTAL_HEAP_SIZE")
    minstk = cfg(ch, "configMINIMAL_STACK_SIZE")
    tstack = cfg(ch, "configTIMER_TASK_STACK_DEPTH")
    tqlen = cfg(ch, "configTIMER_QUEUE_LENGTH")

    print("=== 本工程 FreeRTOS 堆账本 ===")
    print("堆 configTOTAL_HEAP_SIZE        = %s B" % heap)
    print("空闲任务栈 configMINIMAL_STACK_SIZE = %s word（静态数组，不占堆）" % minstk)
    print("定时器任务栈                     = %s word（静态数组，不占堆）" % tstack)
    print("定时器队列长度                   = %s 项" % tqlen)
    print("")

    total = 0
    print("%-12s %8s %10s" % ("任务", "栈(word)", "栈(B)"))
    for name in created:
        macro = "%s_TASK_STACK" % name.upper()
        w = stacks.get(macro)
        if w is None:
            print("%-12s %8s %10s" % (name, "?", "未找到宏 %s" % macro))
            continue
        total += w * 4
        print("%-12s %8d %10d" % (name, w, w * 4))
    n = len(created)
    print("-" * 32)
    print("%-12s %8s %10d   （%d 个应用任务）" % ("栈合计", "", total, n))

    tcb = n * TCB_BYTES
    q = (tqlen or 0) * 8 + MALLOC_HDR + 80      # 定时器队列：项 + 块头 + Queue_t(约 80)
    need = total + tcb + q
    print("%-12s %8s %10d   （每任务 %d B）" % ("TCB", "", tcb, TCB_BYTES))
    print("%-12s %8s %10d" % ("定时器队列", "", q))
    print("-" * 32)
    print("合计占用 need = %d B" % need)
    if heap:
        margin = heap - need
        print("堆 heap       = %d B" % heap)
        print("余量 margin   = %d B  -> %s" % (margin, "够用 ✓" if margin > 0 else "不够 ✗"))
        if 0 < margin < 512:
            print("⚠ 余量偏小（<512 B）：再加任务或加大栈前先重算这份账。")
    return 0 if (heap and need < heap) else 1


if __name__ == "__main__":
    sys.exit(main())
