# 把 proto.c / proto.h 注册进 Keil 工程 (幂等, 可重复运行)
# 参考 tools/tinyml/add_ml_to_keil.py 的做法: 先备份 uvprojx, 再插节点。
import os
import re
import shutil
import time

UVPROJX = r"D:\STM32Project\ICM42688+MMC5603 9 axis gyroscope\MDK-ARM\ICM42688+MMC5603 9 axis gyroscope.uvprojx"

NEW_FILES = [
    ("proto.c", "1", r".\icode\proto.c"),
    ("proto.h", "5", r".\icode\proto.h"),
]

def file_node(name, ftype, path, indent="            "):
    return (
        "%s<File>\n"
        "%s  <FileName>%s</FileName>\n"
        "%s  <FileType>%s</FileType>\n"
        "%s  <FilePath>%s</FilePath>\n"
        "%s</File>\n"
    ) % (indent, indent, name, indent, ftype, indent, path, indent)

def main():
    if not os.path.isfile(UVPROJX):
        raise SystemExit("找不到 Keil 工程: %s" % UVPROJX)

    raw = open(UVPROJX, "rb").read()
    text = raw.decode("utf-8")
    print("工程文件: %s" % UVPROJX)
    print("行尾: CRLF=%d, bareLF=%d" % (text.count("\r\n"), len(re.findall(r"(?<!\r)\n", text))))

    # 已经登记过就跳过
    if "<FileName>proto.c</FileName>" in text:
        print("-> proto.c 已登记, 本次跳过")
        return

    # 1) 定位 icode 分组
    m = re.search(r"<GroupName>icode</GroupName>\s*<Files>", text)
    if not m:
        raise SystemExit("找不到 icode 分组")
    anchor_end = m.end()

    # 2) 备份
    bak = "%s.bak-%s" % (UVPROJX, time.strftime("%Y%m%d-%H%M%S"))
    shutil.copy2(UVPROJX, bak)
    print("-> 已备份: %s" % os.path.basename(bak))

    # 3) 插入 (用 \n, 因为该分组内是 LF)
    block = "\n" + "".join(file_node(n, t, p) for n, t, p in NEW_FILES).rstrip("\n")
    text2 = text[:anchor_end] + block + text[anchor_end:]

    # 4) 幂等性自检: 每个文件名必须恰好出现一次
    for n, _, _ in NEW_FILES:
        c = text2.count("<FileName>%s</FileName>" % n)
        if c != 1:
            raise SystemExit("文件里 <FileName>%s</FileName> 出现 %d 次, 预期 1 次, 已放弃写入" % (n, c))

    open(UVPROJX, "wb").write(text2.encode("utf-8"))
    print("-> 已写入")
    for n, t, p in NEW_FILES:
        print("     %s (FileType=%s)  %s" % (n, t, p))
    print("完成。若 Keil 已打开该工程, 需重新加载才能看到新文件。")

if __name__ == "__main__":
    main()
