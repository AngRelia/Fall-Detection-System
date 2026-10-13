# -*- coding: utf-8 -*-
"""
verify_nmea_math.py —— 逐行复刻 gps.c 里的整数解析算法，用厂家文档里的真实语句验证。

为什么要有这个脚本：
  F103 无 FPU、本工程又刻意不引入 printf/浮点格式化，所以 gps.c 的坐标换算、
  校验和、速度换算全是整数运算。这些算法一旦有位运算/取整/字段下标错误，在板子上
  表现为"定位偏几十米"或"校验和永远不过"，极难定位。在 PC 上用 Python 复刻同一
  算法，拿《DX-GP22-NMEA0183协议规范》《串口UART应用指导》里的原句做对照，能提前拦住。

  本脚本第一次运行就抓出了 gps.c 的一个真实缺陷：RMC/GGA 的字段下标少算 1
  （切分后 s_fp[0] 是语句头 "GNRMC"，时间应从 s_fp[1] 取）。已修复。

用法：  python tools/gps/verify_nmea_math.py
"""

import random

# --------------------------------------------------------------------------
# 与 gps.c 一一对应的整数实现
# --------------------------------------------------------------------------


def hex_val(c):
    if '0' <= c <= '9':
        return ord(c) - ord('0')
    if 'A' <= c <= 'F':
        return ord(c) - ord('A') + 10
    if 'a' <= c <= 'f':
        return ord(c) - ord('a') + 10
    return 0xFF


def xor_body(s):
    cs = 0
    for ch in s:
        cs ^= ord(ch)
    return cs


def checksum_ok(s):
    """s 为不含 '$' 的语句，如 'GNRMC,...,V*00'"""
    cs = 0
    star = None
    for i, ch in enumerate(s):
        if ch == '*':
            star = i
            break
        cs ^= ord(ch)
    if star is None or star + 2 >= len(s):
        return False
    hi, lo = hex_val(s[star + 1]), hex_val(s[star + 2])
    if hi == 0xFF or lo == 0xFF:
        return False
    return (hi << 4) | lo == cs


def parse_uint(s):
    v = 0
    i = 0
    while i < len(s) and '0' <= s[i] <= '9':
        v = v * 10 + (ord(s[i]) - ord('0'))
        i += 1
    return v


def parse_dec(s, frac_digits):
    """返回 值 × 10^frac_digits"""
    v = 0
    i = 0
    while i < len(s) and '0' <= s[i] <= '9':
        v = v * 10 + (ord(s[i]) - ord('0'))
        i += 1
    fd = 0
    if i < len(s) and s[i] == '.':
        i += 1
        while i < len(s) and '0' <= s[i] <= '9' and fd < frac_digits:
            v = v * 10 + (ord(s[i]) - ord('0'))
            fd += 1
            i += 1
    while fd < frac_digits:
        v *= 10
        fd += 1
    return v


def parse_2(s):
    """复刻 gps.c 的 GPS_Parse2：**定长 2 位**解析，非法字符返回 0xFF。

    ★ 这个函数是踩坑之后才加进来的，请务必用它、不要退回 parse_uint：
      2026-10-13 实测 bug —— gps.c 里 UTC/日期的定长字段原本调用了
      GPS_ParseUint()（吃掉所有连续数字），于是 "073028.00" 得到 73028，
      再范围检查 hh>23 直接 return 0 → **UTC 永远显示 0**，而定位本身完全正常。
      本脚本当时没发现，是因为镜像用的是 s[0:2] 切片（按意图写），
      与 C 的真实实现（按通用解析器写）**语义分叉**了。
      教训：镜像必须逐句对齐 C 的实现，而不是对齐"我打算怎么写"。
    """
    if len(s) < 2 or not (s[0].isdigit() and s[1].isdigit()):
        return 0xFF
    return (ord(s[0]) - 48) * 10 + (ord(s[1]) - 48)


def parse_utc_ms(s):
    if not s or len(s) < 6:
        return 0
    hh = parse_2(s[0:2])
    mm = parse_2(s[2:4])
    ss = parse_2(s[4:6])
    frac = 0
    fd = 0
    if len(s) > 7 and s[6] == '.':
        i = 7
        while i < len(s) and '0' <= s[i] <= '9' and fd < 3:
            frac = frac * 10 + (ord(s[i]) - ord('0'))
            fd += 1
            i += 1
    while fd < 3:
        frac *= 10
        fd += 1
    if hh > 23 or mm > 59 or ss > 60:
        return 0
    return hh * 3600000 + mm * 60000 + ss * 1000 + frac


def parse_date(s):
    if not s or len(s) < 6:
        return 0
    dd = parse_2(s[0:2])
    mm = parse_2(s[2:4])
    yy = parse_uint(s[4:6])       # 最后一段，用通用解析器才与 C 一致
    if dd == 0 or mm == 0 or mm > 12:
        return 0
    return (2000 + yy) * 10000 + mm * 100 + dd


def parse_coord_e7(s, neg):
    """'2236.40101' → 度 × 1e7（整数）"""
    dot = 0
    int_digits = 0
    while dot < len(s) and s[dot] != '.':
        int_digits += 1
        dot += 1
    if int_digits < 4:
        return 0
    deg_digits = int_digits - 2
    deg = 0
    for i in range(deg_digits):
        deg = deg * 10 + (ord(s[i]) - ord('0'))
    min_int = 0
    for i in range(deg_digits, int_digits):
        min_int = min_int * 10 + (ord(s[i]) - ord('0'))
    if min_int > 59:
        return 0
    frac = 0
    fd = 0
    if dot < len(s) and s[dot] == '.':
        q = dot + 1
        while q < len(s) and '0' <= s[q] <= '9' and fd < 5:
            frac = frac * 10 + (ord(s[q]) - ord('0'))
            fd += 1
            q += 1
    while fd < 5:
        frac *= 10
        fd += 1
    mm_e5 = min_int * 100000 + frac
    e7 = deg * 10000000 + (mm_e5 * 5 + 1) // 3
    return -e7 if neg else e7


def knots_to_cms(spd_str):
    """节 → cm/s，与 gps.c 的 (ParseDec(s,2)*5144)/10000 完全一致"""
    return (parse_dec(spd_str, 2) * 5144) // 10000


def exact_e7(ddmm_float, neg):
    """参考实现：用浮点算准确值（仅用于比对）"""
    deg = int(ddmm_float / 100)
    minutes = ddmm_float - deg * 100
    v = (deg + minutes / 60.0) * 1e7
    return int(-v if neg else v)


# --------------------------------------------------------------------------
# 厂家资料里的原句
# --------------------------------------------------------------------------
RMC_DOC = "GNRMC,073028.600,A,2236.40101,N,11349.73472,E,0.00,0.00,090724,,,A,V"
RMC_DOC_CS = 0x00     # 《NMEA0183协议规范》RMC 示例：*00
GGA_DOC = "GNGGA,073028.600,2236.40101,N,11349.73472,E,1,19,0.8,14.2,M,-4.0,M,,"
GGA_DOC_CS = 0x6E     # 《NMEA0183协议规范》GGA 示例：*6E


# --------------------------------------------------------------------------
# 1) $PCA 命令校验和：对照《串口UART应用指导》里给了 *CS 的原句
# --------------------------------------------------------------------------
def test_cmd_checksums():
    cases = [
        ("PCAS01,0", 0x1C), ("PCAS01,1", 0x1D), ("PCAS01,2", 0x1E),
        ("PCAS01,3", 0x1F), ("PCAS01,4", 0x18), ("PCAS01,5", 0x19),
        ("PCAS02,1000", 0x2E), ("PCAS02,500", 0x1A),
        ("PCAS02,200", 0x1D), ("PCAS02,100", 0x1E),
        ("PCAS04,1", 0x18), ("PCAS04,2", 0x1B), ("PCAS04,3", 0x1A),
        ("PCAS04,4", 0x1D), ("PCAS04,5", 0x1C), ("PCAS04,6", 0x1F),
        ("PCAS04,7", 0x1E),
        ("PCAS05,2", 0x1A), ("PCAS05,5", 0x1D),
        ("PCAS06,0", 0x1B), ("PCAS06,2", 0x19), ("PCAS06,3", 0x18),
        ("PCAS06,5", 0x1E),
        ("PCAS10,0", 0x1C), ("PCAS10,1", 0x1D), ("PCAS10,2", 0x1E),
        ("PCAS10,3", 0x1F),
        ("PCAS03,0,0,0,0,0,0,0,0,0,0,,,0,0", 0x02),   # 文档：全部停止输出
    ]
    bad = []
    for body, want in cases:
        got = xor_body(body)
        if got != want:
            bad.append((body, want, got))
    print("[1] $PCA 命令校验和：%d 个用例，失败 %d 个" % (len(cases), len(bad)))
    for body, want, got in bad:
        print("    x %-46s 文档=%02X 计算=%02X" % (body, want, got))

    off = "PCAS03,0,0,0,0,0,0,0,0,0,0,,,0,0"
    on = "PCAS03,1,1,1,1,1,1,1,1,1,1,,,1,1"
    mine = "PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0"
    print("    文档全关: $%s*%02X   文档全开: $%s*%02X"
          % (off, xor_body(off), on, xor_body(on)))
    print("    说明：'0'×12 与 '1'×12 的异或都为 0（偶数个），所以文档两条都标 *02 是对的")
    print("    驱动实际发送: $%s*%02X   （只留 GGA+RMC，14 个参数）"
          % (mine, xor_body(mine)))
    return len(bad) == 0


# --------------------------------------------------------------------------
# 2) 真实语句：校验和 + 坐标 + 时间 + 速度
# --------------------------------------------------------------------------
def test_real_sentences():
    ok = True
    print("[2] 厂家示例语句解析")
    for name, body, want in (("RMC", RMC_DOC, RMC_DOC_CS), ("GGA", GGA_DOC, GGA_DOC_CS)):
        got = xor_body(body)
        flag = "OK" if got == want else "FAIL"
        print("    %s 校验和 文档=%02X 计算=%02X  %s" % (name, want, got, flag))
        if got != want:
            ok = False
        if not checksum_ok(body + "*%02X" % want):
            print("    x %s 校验和判定函数未通过" % name)
            ok = False
    # 反例：改一个字符必须判不过
    if checksum_ok(RMC_DOC.replace("073028", "073029") + "*00"):
        print("    x 篡改语句竟然通过校验")
        ok = False

    lat, lon = parse_coord_e7("2236.40101", False), parse_coord_e7("11349.73472", False)
    lat_ref, lon_ref = exact_e7(2236.40101, False), exact_e7(11349.73472, False)
    print("    纬度: 整数=%d 浮点参考=%d 偏差=%d e7 单位" % (lat, lat_ref, lat - lat_ref))
    print("    经度: 整数=%d 浮点参考=%d 偏差=%d e7 单位" % (lon, lon_ref, lon - lon_ref))
    if abs(lat - lat_ref) > 1 or abs(lon - lon_ref) > 1:
        ok = False

    utc = parse_utc_ms("073028.600")
    exp_utc = 7 * 3600000 + 30 * 60000 + 28 * 1000 + 600
    print("    UTC 毫秒=%d（期望 %d）" % (utc, exp_utc))
    ok = ok and utc == exp_utc

    d = parse_date("090724")
    print("    日期=%d（期望 20240709）" % d)
    ok = ok and d == 20240709

    cms = knots_to_cms("12.34")
    print("    速度 12.34 kn → %d cm/s（期望约 634）" % cms)
    ok = ok and abs(cms - 634) <= 1

    if parse_coord_e7("2236.40101", True) != -lat:
        print("    x 南纬取负失败")
        ok = False
    if parse_coord_e7("11349.73472", True) != -lon:
        print("    x 西经取负失败")
        ok = False

    for weird in ("", ",", "..", "abcd", "9999.99999"):
        parse_coord_e7(weird, False)
    print("    异常字段健壮性：通过")
    return ok


# --------------------------------------------------------------------------
# 3) 随机模糊测试：整数换算 vs 浮点参考
# --------------------------------------------------------------------------
def test_fuzz(n=200000):
    random.seed(20240709)
    worst = 0
    for _ in range(n):
        lat = random.random() < 0.5
        deg = random.randint(0, 89) if lat else random.randint(0, 179)
        minutes = random.randint(0, 59)
        frac = random.randint(0, 99999)
        s = "%0*d%02d.%05d" % (2 if lat else 3, deg, minutes, frac)
        neg = random.random() < 0.5
        got = parse_coord_e7(s, neg)
        ref = exact_e7(float("%d.%05d" % (deg * 100 + minutes, frac)), neg)
        d = abs(got - ref)
        worst = max(worst, d)
        if d > 1:
            print("    x %s neg=%s 整数=%d 参考=%d" % (s, neg, got, ref))
            return False
    print("[3] 随机模糊测试 %d 组，最大偏差 %d 个 e7 单位（≈ %.1f cm）"
          % (n, worst, worst * 1.11))
    return worst <= 1


# --------------------------------------------------------------------------
# 4) 字段下标：s_fp[0] 是语句头，所以时间/纬度都是从 1 起算
# --------------------------------------------------------------------------
def test_field_order():
    """这是本次抓出的真实缺陷所在：必须按 index 0 = 语句头来取字段"""
    checks = []

    r = RMC_DOC.split(',')
    # gps.c: utc=1 status=2 lat=3 N/S=4 lon=5 E/W=6 spd=7 cog=8 date=9
    checks.append((r[0][-3:] == "RMC", "RMC 语句头后 3 字符"))
    checks.append((r[1] == "073028.600", "RMC 时间 = s_fp[1]"))
    checks.append((r[2] == "A", "RMC status = s_fp[2]"))
    checks.append((r[3] == "2236.40101" and r[4] == "N", "RMC 纬度/半球 = s_fp[3]/[4]"))
    checks.append((r[5] == "11349.73472" and r[6] == "E", "RMC 经度/半球 = s_fp[5]/[6]"))
    checks.append((r[7] == "0.00", "RMC 速度 = s_fp[7]"))
    checks.append((r[8] == "0.00", "RMC 航向 = s_fp[8]"))
    checks.append((r[9] == "090724", "RMC 日期 = s_fp[9]"))

    g = GGA_DOC.split(',')
    # gps.c: utc=1 lat=2 N/S=3 lon=4 E/W=5 quality=6 sats=7 hdop=8 alt=9
    checks.append((g[0][-3:] == "GGA", "GGA 语句头后 3 字符"))
    checks.append((g[1] == "073028.600", "GGA 时间 = s_fp[1]"))
    checks.append((g[2] == "2236.40101" and g[3] == "N", "GGA 纬度/半球 = s_fp[2]/[3]"))
    checks.append((g[4] == "11349.73472" and g[5] == "E", "GGA 经度/半球 = s_fp[4]/[5]"))
    checks.append((g[6] == "1", "GGA 定位质量 = s_fp[6]"))
    checks.append((g[7] == "19", "GGA 卫星数 = s_fp[7]"))
    checks.append((g[8] == "0.8", "GGA HDOP = s_fp[8]"))
    checks.append((g[9] == "14.2", "GGA 海拔 = s_fp[9]"))

    bad = [name for ok, name in checks if not ok]
    print("[4] 字段下标校验：%d 项，失败 %d 项 %s"
          % (len(checks), len(bad), ("-> " + ", ".join(bad)) if bad else ""))

    extra = []
    if parse_uint(g[7]) != 19:
        extra.append("卫星数")
    if parse_dec(g[8], 2) != 80:
        extra.append("HDOP(0.8→80)")
    if parse_dec(g[9], 2) != 1420:
        extra.append("海拔(14.2→1420cm)")
    if extra:
        print("    x 数值解析失败：%s" % ",".join(extra))
    return not bad and not extra


# --------------------------------------------------------------------------
# 5) 端到端流水线仿真：复刻 GPS_FeedByte + 切分 + 解析
# --------------------------------------------------------------------------
def sim_pipeline(stream):
    """复刻 gps.c 的 GPS_FeedByte / GPS_SplitFields / GPS_ParseLine"""
    fix = {"has_rmc": 0, "has_gga": 0, "status": 0, "quality": 0}
    line = []
    bad = 0
    csum_err = 0
    sentences = 0

    def parse_line(text):
        nonlocal csum_err, sentences
        if not checksum_ok(text):
            csum_err += 1
            return
        sentences += 1
        body = text.split('*')[0]
        f = body.split(',')
        nf = len(f)
        if f[0][-3:] == "RMC" and nf >= 10:
            fix["utc_ms"] = parse_utc_ms(f[1])
            fix["status"] = 1 if f[2][:1] == 'A' else 0
            fix["lat_e7"] = parse_coord_e7(f[3], f[4][:1] == 'S')
            fix["lon_e7"] = parse_coord_e7(f[5], f[6][:1] == 'W')
            fix["speed_cms"] = knots_to_cms(f[7])
            fix["course_cdeg"] = parse_dec(f[8], 2)
            fix["date"] = parse_date(f[9])
            fix["has_rmc"] = 1
        elif f[0][-3:] == "GGA" and nf >= 10:
            fix["quality"] = parse_uint(f[6])
            fix["sats"] = parse_uint(f[7])
            fix["hdop_x100"] = parse_dec(f[8], 2)
            fix["alt_cm"] = parse_dec(f[9], 2)
            fix["has_gga"] = 1

    for ch in stream:
        if ch == '$':
            line = []
            bad = 0
            continue
        if ch == '\r':
            continue
        if ch == '\n':
            if bad == 0 and len(line) > 6:
                parse_line("".join(line))
            line = []
            bad = 0
            continue
        if bad:
            continue
        if not (0x20 <= ord(ch) <= 0x7E):
            continue
        if len(line) >= 95:
            bad = 1
            line = []
            continue
        line.append(ch)

    fix["valid"] = 1 if (fix["has_rmc"] and fix["status"] and fix["quality"] > 0) else 0
    return fix, sentences, csum_err


def test_pipeline():
    gsv = "GPGSV,3,1,11,01,05,040,45,02,15,120,40"
    rmc_invalid = "GNRMC,073029.000,V,0000.00000,N,00000.00000,E,0.00,0.00,090724,,,N,V"

    # 场景 A：噪声 + 正确 RMC/GGA + 篡改校验和的 RMC + 无关 GSV
    stream_a = ("\x00\x00\xff"                                        # 上电噪声
                + "$%s*%02X\r\n" % (RMC_DOC, RMC_DOC_CS)              # 正确语句
                + "$%s*%02X\r\n" % (GGA_DOC, GGA_DOC_CS)              # 正确语句
                + "$%s*7F\r\n" % RMC_DOC                              # 篡改校验和 → 必须被拒
                + "$%s*%02X\r\n" % (gsv, xor_body(gsv)))              # 无关语句，校验通过即忽略
    fa, sa, ea = sim_pipeline(stream_a)

    # 场景 B：再追加一条"RMC 报无效定位"
    fb, sb, eb = sim_pipeline(stream_a + "$%s*%02X\r\n" % (rmc_invalid, xor_body(rmc_invalid)))

    print("[5] 端到端流水线仿真")
    print("    A: 有效语句 %d 条（期望 3），校验和错误 %d 条（期望 1）" % (sa, ea))
    print("       lat=%d lon=%d q=%d sat=%d hdop=%d alt=%dcm date=%d v=%d"
          % (fa["lat_e7"], fa["lon_e7"], fa["quality"], fa["sats"],
             fa["hdop_x100"], fa["alt_cm"], fa["date"], fa["valid"]))
    print("    B: 追加无效定位后 status=%d lat=%d lon=%d v=%d（期望 0 / 0 / 0 / 0）"
          % (fb["status"], fb["lat_e7"], fb["lon_e7"], fb["valid"]))

    ok = True
    ok = ok and sa == 3 and ea == 1
    ok = ok and fa["lat_e7"] == parse_coord_e7("2236.40101", False)
    ok = ok and fa["lon_e7"] == parse_coord_e7("11349.73472", False)
    ok = ok and fa["quality"] == 1 and fa["sats"] == 19
    ok = ok and fa["hdop_x100"] == 80 and fa["alt_cm"] == 1420
    ok = ok and fa["date"] == 20240709 and fa["valid"] == 1
    ok = ok and fb["status"] == 0 and fb["lat_e7"] == 0 and fb["lon_e7"] == 0 and fb["valid"] == 0
    ok = ok and sb == 4 and eb == 1
    if not ok:
        print("    x 流水线断言未全部通过")
    return ok


# --------------------------------------------------------------------------
# 6) GSV 解析（可见卫星数 + 本轮最强信噪比）—— 联调时判断天线的关键指标
# --------------------------------------------------------------------------
def parse_gsv(fields, state):
    """复刻 gps.c 的 GPS_ParseGSV。fields[0] 是语句头，故卫星字段从 1 起算。"""
    nf = len(fields)
    if nf < 4:
        return state
    mx = 0
    i = 7
    while i < nf and i <= 19:
        snr = parse_uint(fields[i]) if i < nf else 0
        if snr > mx:
            mx = snr
        i += 4
    if fields[2][:1] == '1':          # 新一轮的第 1 句
        state["sats_view"] = parse_uint(fields[3])
        state["snr_max"] = mx
    elif mx > state["snr_max"]:
        state["snr_max"] = mx
    return state


def test_gsv():
    # 一轮 3 句 GSV（人造数据，字段位置与厂家协议规范一致）
    sweep = [
        "GPGSV,3,1,11,01,05,040,45,02,15,120,40,03,25,200,30,04,35,280,20",
        "GPGSV,3,2,11,05,45,320,35,06,55,010,25,07,65,090,15,08,75,180,05",
        "GPGSV,3,3,11,09,85,270,50",
    ]
    state = {"sats_view": 0, "snr_max": 0}
    for sent in sweep:
        state = parse_gsv(sent.split(","), state)
    print("[6] GSV 解析")
    print("    可见卫星 sats_view=%d（期望 11，取自第 1 句的字段[3]）" % state["sats_view"])
    print("    最强信噪比 snr_max=%d（期望 50，整轮取最大）" % state["snr_max"])
    ok = (state["sats_view"] == 11) and (state["snr_max"] == 50)

    # 新一轮开始时必须重置为本轮最大值，而不是继承上一轮
    state = parse_gsv("GPGSV,3,1,03,10,10,100,12".split(","), state)
    print("    新一轮第1句后 snr_max=%d（期望 12，说明已重置而非继承 50）" % state["snr_max"])
    ok = ok and state["snr_max"] == 12 and state["sats_view"] == 3

    # 无信号场景：可见 0、信噪比全空 → 天线/射频确实没收到东西
    state = parse_gsv("GPGSV,1,1,00,,,,,,,,,,,,,,,,,".split(","), state)
    print("    无信号句后 sats_view=%d snr_max=%d（期望 0 / 0）" % (state["sats_view"], state["snr_max"]))
    ok = ok and state["sats_view"] == 0 and state["snr_max"] == 0
    if not ok:
        print("    x GSV 断言未全部通过")
    return ok


def test_utc_date():
    """[7] UTC/日期定长字段解析 —— 这组用例是为 2026-10-13 那个 UTC 永远为 0 的 bug 补的。"""
    print("[7] UTC / 日期字段解析")
    ok = True

    cases = [
        ("073028.00", 7 * 3600000 + 30 * 60000 + 28 * 1000),
        ("235959.999", 23 * 3600000 + 59 * 60000 + 59 * 1000 + 999),
        ("000000.00", 0),
        ("120000", 12 * 3600000),
        ("", 0),
        ("0730", 0),          # 不足 6 位 → 0
        ("xx3028.00", 0),     # 非法 → 0
    ]
    for s, want in cases:
        got = parse_utc_ms(s)
        flag = "ok " if got == want else "FAIL"
        if got != want:
            ok = False
        print("    %-12s -> %8d (期望 %8d) %s" % ('"%s"' % s, got, want, flag))

    d = parse_date("090724")
    print("    日期 \"090724\" -> %d (期望 20240709) %s" % (d, "ok " if d == 20240709 else "FAIL"))
    ok = ok and (d == 20240709)

    # 回归护栏：通用解析器在这类定长字段上是错的，若有人把 parse_2 改回 parse_uint，这里会报警
    bad = parse_uint("073028.00")
    good = parse_2("073028.00")
    print("    对照: 通用解析器读到 %d（曾经的错误写法，会被 hh>23 拒掉 → UTC 恒为 0）" % bad)
    print("          定长解析器读到 %d（正确写法）" % good)
    if not (bad == 73028 and good == 7):
        print("    FAIL: 两个解析器的行为与预期不符（镜像可能又和 C 分叉了）")
        ok = False
    return ok


if __name__ == "__main__":
    print("=== gps.c 整数算法验证（对照厂家资料原句）===")
    results = [
        test_cmd_checksums(),
        test_real_sentences(),
        test_fuzz(),
        test_field_order(),
        test_pipeline(),
        test_gsv(),
        test_utc_date(),
    ]
    print("=== 结果：%s ===" % ("全部通过" if all(results) else "有失败项，见上"))
    raise SystemExit(0 if all(results) else 1)
