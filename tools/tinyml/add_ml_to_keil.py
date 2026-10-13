"""
add_ml_to_keil.py —— 把 icode/ml 模块登记进 Keil 工程（幂等、可重复运行）

做三件事：
    1. 备份 uvprojx（.bak-<时间戳>），已存在同名备份则跳过
    2. 在 IncludePath 里补上 ./icode/ml
    3. 在 icode/task 分组之后插入 icode/ml 分组，登记 5 个源文件与 4 个头文件

为什么用脚本而不是手改：
    uvprojx 是 XML，但 Keil 对格式（缩进、CRLF、节点顺序）比较敏感。
    手改容易漏掉 IncludePath 或多写一个 </File>，而这类错误只在 Keil 里
    打开时才暴露，且报错信息含糊。脚本每次都能做同样的改动，并顺手做 XML 校验。

运行：
    E:\\anaconda3\\python.exe tools\\tinyml\\add_ml_to_keil.py
"""

import os
import re
import shutil
import sys
import time
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
UVPROJX = os.path.join(ROOT, "Security Watch", "MDK-ARM", "Security Watch.uvprojx")
ML_SUBDIR = r".\icode\ml"

# 组内文件：(文件名, FileType)  1 = C 源文件, 5 = 头文件
GROUP_FILES = [
    ("ml_infer.c", 1), ("ml_infer.h", 5),
    ("ml_features.c", 1), ("ml_features.h", 5),
    ("ml_weights.c", 1), ("ml_weights.h", 5),
    ("ml_selftest.c", 1), ("ml_selftest.h", 5),
    ("ml_golden.h", 5),
]


def main():
    if not os.path.isfile(UVPROJX):
        raise SystemExit("找不到 Keil 工程：%s" % UVPROJX)

    raw = open(UVPROJX, "rb").read()
    bom = raw.startswith(b"\xef\xbb\xbf")
    text = raw.decode("utf-8-sig")
    nl = "\r\n" if "\r\n" in text else "\n"
    print("工程文件：%s" % UVPROJX)
    print("编码：UTF-8%s，换行：%s" %
          ("(BOM)" if bom else "", "CRLF" if nl == "\r\n" else "LF"))

    group_exists = "<GroupName>icode/ml</GroupName>" in text
    if group_exists:
        print("→ icode/ml 分组已存在，本次只检查 IncludePath（幂等）")

    # ---- 0. 先确认磁盘上的文件真的都在 -----------------------------------
    ml_dir = os.path.join(ROOT, "Security Watch", "MDK-ARM", "icode", "ml")
    missing = [f for f, _ in GROUP_FILES
               if not os.path.isfile(os.path.join(ml_dir, f))]
    if missing:
        print("✗ 以下文件还不存在，请先跑 train_quantize.py：%s" % missing)
        return 1
    print("→ 待登记文件全部存在（%d 个）" % len(GROUP_FILES))

    # ---- 1. 备份 ---------------------------------------------------------
    bak = "%s.bak-%s" % (UVPROJX, time.strftime("%Y%m%d-%H%M%S"))
    shutil.copy2(UVPROJX, bak)
    print("→ 已备份：%s" % os.path.basename(bak))

    # ---- 2. IncludePath ---------------------------------------------------
    # ⚠ 这里必须**只**改目标级的那一条 IncludePath。
    #   uvprojx 里还有二十多处 <IncludePath></IncludePath>，那是每个文件自己的
    #   <FileOption> 里的"空 = 继承上级"标记。若把空的也填上 ./icode/ml，
    #   那些文件就只会在./icode/ml 里找头文件，连 stm32f1xx_hal.h 都找不到，
    #   整个工程会编译失败 —— 这个坑已经踩过一次，故用锚点严格限定。
    ANCHOR = "./icode/task"

    hits = []

    def add_include(m):
        path = m.group(1)
        if ANCHOR not in path:
            return m.group(0)            # 不是目标级的那一条，原样保留
        if "icode/ml" in path:
            hits.append("already")
            return m.group(0)
        hits.append("patched")
        return "<IncludePath>%s;./icode/ml</IncludePath>" % path.rstrip(";")

    text = re.sub(r"<IncludePath>([^<]*)</IncludePath>", add_include, text)

    if hits.count("patched") == 1:
        print("→ 目标级 IncludePath 已补上 ./icode/ml")
    elif hits.count("already") == 1:
        print("→ 目标级 IncludePath 已含 ./icode/ml，跳过")
    else:
        raise SystemExit("✗ 预期恰好改到 1 处目标级 IncludePath，实际 patched=%d "
                         "already=%d —— 工程结构可能已变，请人工检查"
                         % (hits.count("patched"), hits.count("already")))
    if text.count("./icode/ml") != 1:
        raise SystemExit("✗ 文件里出现了 %d 处 ./icode/ml，预期 1 处，已放弃写入"
                         % text.count("./icode/ml"))

    # ---- 3. 插入分组 -----------------------------------------------------
    if not group_exists:
        anchor = "          <GroupName>icode/task</GroupName>"
        pos = text.find(anchor)
        if pos < 0:
            raise SystemExit("找不到 icode/task 分组，无法定位插入点")

        # 找到该分组的 </Group>
        end = text.find("        </Group>", pos)
        if end < 0:
            raise SystemExit("找不到 icode/task 分组的结束标签")
        end += len("        </Group>")

        files_xml = "".join(
            "%s            <File>%s              <FileName>%s</FileName>%s"
            "              <FileType>%d</FileType>%s"
            "              <FilePath>%s\\%s</FilePath>%s"
            "            </File>%s"
            % (nl, nl, name, nl, ftype, nl, ML_SUBDIR, name, nl, nl)
            for name, ftype in GROUP_FILES)

        group_xml = (
            "%s        <Group>%s"
            "          <GroupName>icode/ml</GroupName>%s"
            "          <Files>%s%s          </Files>%s"
            "        </Group>"
            % (nl, nl, nl, nl, files_xml, nl))

        text = text[:end] + group_xml + text[end:]
        print("→ 已插入 icode/ml 分组（%d 个文件）" % len(GROUP_FILES))

    # ---- 4. 校验后再落盘 -------------------------------------------------
    try:
        ET.fromstring(text.encode("utf-8"))
    except ET.ParseError as e:
        print("✗ 改完的 XML 解析失败：%s" % e)
        print("  工程文件未改动，备份在 %s" % bak)
        return 1
    print("→ XML 解析校验通过")

    open(UVPROJX, "wb").write((b"\xef\xbb\xbf" if bom else b"") + text.encode("utf-8"))
    print("→ 已写回工程文件")
    print("\n完成。请在 Keil 里重新打开工程（或 File→Reload），确认左侧出现"
          " icode/ml 分组，然后 F7 编译。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
