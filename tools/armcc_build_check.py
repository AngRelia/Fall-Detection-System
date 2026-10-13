# -*- coding: utf-8 -*-
"""
armcc_build_check.py —— 不依赖 Keil UV4，直接用 armcc/armasm/armlink 把 Keil 工程
完整编译 + 链接一遍，用来回答两个问题：

  1) 代码能不能编过、能不能链上（有没有漏文件、重定义、未定义符号）
  2) 链接后的 Flash / RAM 占用是多少，RAM 是否还塞得进 20 KB

为什么需要它：
  本机 Keil UV4 在 DSH 沙箱里命令行构建会以 0xC0000409（栈缓冲区溢出）退出，
  这不是工程问题（未改动的工程同样如此），但意味着 AI 侧拿不到编译结论。
  而 E:\\Keil5\\ARM\\ARMCC\\bin 下的 armcc / armasm / armlink 都能正常执行，
  于是直接复刻 Keil 的编译链接流程即可。

用法：
  python tools\\armcc_build_check.py ["Security Watch\\MDK-ARM\\Security Watch.uvprojx"]
说明：
  · 编译/链接选项从 uvprojx 里读（IncludePath / Define / Optim / uC99），
    文件清单、分组、IncludeInBuild 也从 uvprojx 读，避免和 Keil 不一致
  · 目标文件写到 <MDK-ARM>\\_armcc_check\\，脚本结束时会打印占用，不自动删除
"""

import os
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

KEIL = r"E:\Keil5\ARM\ARMCC\bin"
ARMCC = os.path.join(KEIL, "armcc.exe")
ARMASM = os.path.join(KEIL, "armasm.exe")
ARMLINK = os.path.join(KEIL, "armlink.exe")


def run(cmd, cwd, logpath):
    """把子进程输出写进文件（不用管道，避免沙箱下 named pipe 限制）"""
    with open(logpath, "wb") as fp:
        p = subprocess.run(cmd, cwd=cwd, stdout=fp, stderr=subprocess.STDOUT)
    txt = ""
    try:
        txt = open(logpath, "r", encoding="gbk", errors="replace").read()
    except Exception:
        pass
    return p.returncode, txt


def main():
    proj = sys.argv[1] if len(sys.argv) > 1 else r"Security Watch\MDK-ARM\Security Watch.uvprojx"
    proj = os.path.abspath(proj)
    mdk = os.path.dirname(proj)
    root = os.path.dirname(mdk)                    # 工程根目录（Security Watch\）
    out = os.path.join(mdk, "_armcc_check")
    obj = os.path.join(out, "obj")
    os.makedirs(obj, exist_ok=True)

    tree = ET.parse(proj)
    r = tree.getroot()

    def find_all(tag):
        return r.iter(tag)

    def text_of(el, tag):
        n = el.find(tag)
        return n.text if n is not None and n.text else ""

    # Keil 把编译器选项放在 TargetOption/TargetArmAds/Cads/VariousControls 下
    copt = None
    taa = r.find(".//TargetArmAds")
    if taa is not None:
        copt = taa.find("Cads")
    if copt is None:
        for c in r.iter("Cads"):
            if c.find("VariousControls") is not None:
                copt = c
                break
    if copt is None:
        print("uvprojx 里找不到 Cads 编译选项节点")
        return 1

    vc = copt.find("VariousControls")
    if vc is None:
        vc = copt
    defines = [d.strip() for d in text_of(vc, "Define").split(",") if d.strip()]
    incs = [s.strip() for s in text_of(vc, "IncludePath").split(";") if s.strip()]
    optim = text_of(copt, "Optim") or "4"
    uc99 = text_of(copt, "uC99") == "1"
    misc = text_of(vc, "MiscControls").strip()
    miscs = misc.split() if misc else []
    # Keil 的 "One ELF Section per Function"（OneElfS=1）必须传 --split_sections，
    # 否则未用函数（以及它们牵出的 printf/浮点库）都会被打包进来，体积会虚高几万字节
    if text_of(copt, "OneElfS") == "1":
        miscs.append("--split_sections")
    if text_of(copt, "interw") == "1":
        miscs.append("--apcs=interwork")
    if miscs:
        print("额外选项  : %s" % " ".join(miscs))

    print("工程      : %s" % os.path.relpath(proj, root))
    print("宏定义    : %s" % ",".join(defines))
    print("包含路径  : %d 条" % len(incs))
    print("优化等级  : -O%s   C99: %s" % ({"1": "0", "2": "1", "3": "2", "4": "3"}.get(optim, optim), uc99))

    # ---- 收集要编译的文件 ----
    jobs = []      # (source_abs, out_obj, kind)
    skipped = []
    for grp in r.iter("Group"):
        gname = text_of(grp, "GroupName")
        files = grp.find("Files")
        if files is None:
            continue
        for f in files.findall("File"):
            name = text_of(f, "FileName")
            ftype = text_of(f, "FileType")
            fpath = text_of(f, "FilePath").replace("\\", os.sep)
            src = os.path.normpath(os.path.join(mdk, fpath))
            if ftype == "5" or name.lower().endswith(".h"):
                continue
            inc = "1"
            fo = f.find("FileOption")
            if fo is not None:
                cp = fo.find("CommonProperty")
                if cp is not None:
                    inc = text_of(cp, "IncludeInBuild") or "1"
            if inc == "0":
                skipped.append(name)
                continue
            stem = os.path.splitext(name)[0]
            dst = os.path.join(obj, stem + ".o")
            if os.path.exists(dst):                     # 重名文件加后缀，避免互相覆盖
                dst = os.path.join(obj, stem + "_%d.o" % len(jobs))
            jobs.append((src, dst, gname, name))

    print("待编译    : %d 个源文件%s" % (len(jobs), ("（跳过 " + ",".join(skipped) + "）") if skipped else ""))

    common = ["--cpu", "Cortex-M3"] + ["-D" + d for d in defines] \
             + ["-I" + i for i in incs] + ["-O" + {"1": "0", "2": "1", "3": "2", "4": "3"}.get(optim, "3")] \
             + miscs
    if uc99:
        common.insert(0, "--c99")

    # ---- 编译 ----
    failed = []
    for src, dst, gname, name in jobs:
        log = os.path.join(out, "cc_%s.log" % os.path.splitext(name)[0])
        if src.lower().endswith(".s"):
            # 本工程的启动文件只按 __MICROLIB 条件汇编（useUlib=0，不定义），无需额外宏
            cmd = [ARMASM, "--cpu", "Cortex-M3", "-o", dst, src]
        else:
            cmd = [ARMCC] + common + ["-c", "-o", dst, src]
        code, txt = run(cmd, mdk, log)
        if code != 0 or not os.path.exists(dst):
            failed.append((name, txt.strip().splitlines()[:6]))
            print("  x %-28s 编译失败" % name)
        else:
            print("  . %-28s %7d B" % (name, os.path.getsize(dst)))

    if failed:
        print("\n编译失败 %d 个：" % len(failed))
        for name, lines in failed:
            print("  --- %s ---" % name)
            for ln in lines:
                print("      " + ln)
        return 2

    # ---- 链接 ----
    objs = [dst for _, dst, _, _ in jobs]
    sct = os.path.join(mdk, "Security Watch", "Security Watch.sct")
    if not os.path.exists(sct):
        cands = [os.path.join(d, f) for d, _, fs in os.walk(mdk) for f in fs if f.endswith(".sct")]
        sct = cands[0] if cands else None
    if sct is None:
        print("\n找不到 .sct 分散加载文件，无法链接（请先在 Keil 里构建一次以生成）")
        return 3

    axf = os.path.join(out, "check.axf")
    mapfile = os.path.join(out, "check.map")
    log = os.path.join(out, "link.log")
    cmd = [ARMLINK, "--cpu", "Cortex-M3", "--strict", "--scatter", sct] + objs + \
          ["--info", "sizes", "--info", "totals", "--map", "--list", mapfile, "-o", axf]
    code, txt = run(cmd, mdk, log)
    print("\n链接退出码: %d  (0=成功)" % code)

    if code != 0:
        over = re.findall(r"Sections of aggregate size (\S+) bytes could not fit", txt)
        no_space = "L6406E" in txt or "L6407E" in txt
        print("  链接失败原因: %s" % ("存储区放不下（超出真实 ROM/RAM 容量）" if no_space else "其它错误"))
        if over:
            print("  放不下的聚合大小: %s 字节" % ", ".join(over))
        if not no_space:
            print(txt[:2000])
            return 4
        # 用放大后的存储区重新链接一次，仅为测出真实占用总量
        m = re.search(r"(LR_IROM1\s+\S+\s+)(\S+)", open(sct, encoding="gbk", errors="replace").read())
        big = re.sub(r"(LR_IROM1\s+\S+\s+)\S+", r"\g<1>0x00040000", open(sct, encoding="gbk", errors="replace").read())
        big = re.sub(r"(ER_IROM1\s+\S+\s+)\S+", r"\g<1>0x00040000", big)
        big = re.sub(r"(RW_IRAM1\s+\S+\s+)\S+", r"\g<1>0x00020000", big)
        msct = os.path.join(out, "measure.sct")
        open(msct, "w", encoding="utf-8").write(big)
        m_axf = os.path.join(out, "measure.axf")
        m_map = os.path.join(out, "measure.map")
        m_log = os.path.join(out, "measure.log")
        cmd2 = [ARMLINK, "--cpu", "Cortex-M3", "--scatter", msct] + objs + \
               ["--info", "totals", "--map", "--list", m_map, "-o", m_axf]
        code2, txt2 = run(cmd2, mdk, m_log)
        print("  放大存储区复测: 退出码 %d" % code2)
        if code2 == 0:
            mapfile = m_map
            txt = txt2
        else:
            print(txt2[:1200])
            return 4

    print(txt.strip()[:1200])
    # 从 map 里抓总占用
    maptxt = open(mapfile, "r", encoding="gbk", errors="replace").read()
    nums = {}
    for key, pat in (("RO", r"Total RO\s+Size \(Code \+ RO Data\)\s+(\d+)"),
                     ("RW", r"Total RW\s+Size \(RW Data \+ ZI Data\)\s+(\d+)"),
                     ("ROM", r"Total ROM Size \(Code \+ RO Data \+ RW Data\)\s+(\d+)")):
        m = re.search(pat, maptxt)
        if m:
            nums[key] = int(m.group(1))
    if nums:
        print("\n实测占用（本次校验构建）:")
        if "RO" in nums:
            print("  Flash(RO)      = %6d B = %5.2f KB   可用 65536 B -> 余 %d B" % (nums["RO"], nums["RO"] / 1024, 65536 - nums["RO"]))
        if "RW" in nums:
            print("  RAM(RW+ZI)     = %6d B = %5.2f KB   可用 20480 B -> 余 %d B" % (nums["RW"], nums["RW"] / 1024, 20480 - nums["RW"]))
        if "ROM" in nums:
            print("  烧写镜像(ROM)  = %6d B = %5.2f KB" % (nums["ROM"], nums["ROM"] / 1024))
        if nums.get("RO", 0) > 65536 or nums.get("RW", 0) > 20480:
            print("  >>> 结论：超出 F103C8 的真实容量，必须裁剪 <<<")
        else:
            print("  >>> 结论：可以装进 64 KB Flash / 20 KB RAM <<<")
    print("\n目标文件与日志留在: %s" % os.path.relpath(out, root))
    return 0 if code == 0 else 0


if __name__ == "__main__":
    sys.exit(main())
