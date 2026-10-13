# L1 自检: 验证 serial_logger.py 的解析器与固件打包约定一致。
# 手法: 按协议手工造帧 -> 用解析器解回来 -> 比对字段。
# 这能在没有硬件的情况下抓住: 偏移错、端序错、CRC 范围错、定点换算错。

import struct
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from serial_logger import parse_frame, crc16_modbus, ACC_SCALE, GYRO_SCALE, RPY_SCALE, FRAME_LEN


def build_frame(seq, t_ms, acc_raw, gyro_raw, rpy, flags):
    f = bytearray(FRAME_LEN)
    f[0] = 0xAA
    f[1] = 0x55
    f[2] = 0x01
    f[3] = seq
    struct.pack_into("<I", f, 4, t_ms)
    struct.pack_into("<3h", f, 8, *acc_raw)
    struct.pack_into("<3h", f, 14, *gyro_raw)
    struct.pack_into("<3h", f, 20, *rpy)
    struct.pack_into("<I", f, 26, flags)
    crc = crc16_modbus(bytes(f[2:30]))
    struct.pack_into("<H", f, 30, crc)
    return bytes(f)


print("=" * 72)
print("L1 自检: 造帧 -> 解析 往返一致性")
print("=" * 72)

cases = [
    # (acc_raw, gyro_raw, rpy(0.01度), flags)
    ((0, 0, 2048), (0, 0, 0), (0, 0, 0), 0x0000),
    ((1, -1, 2047), (100, -100, 0), (1234, -567, 8901), 0x003F),
    ((-32768, 32767, 0), (-32768, 32767, 0), (-18000, 18000, 0), 0xFFFFFFFF),
    ((12345, -23456, 30000), (12345, -23456, 30000), (9000, -9000, 17999), 0x0001),
]

allok = True
for idx, (acc, gyro, rpy, flags) in enumerate(cases):
    f = build_frame(idx * 7, 1000 + idx * 10, acc, gyro, rpy, flags)
    rec = parse_frame(f)
    ok = rec is not None
    if ok:
        # 逐字段核对
        exp_acc = tuple(v * ACC_SCALE for v in acc)
        exp_gyro = tuple(v * GYRO_SCALE for v in gyro)
        exp_rpy = tuple(v * RPY_SCALE for v in rpy)
        checks = [
            ("seq", rec["seq"], idx * 7),
            ("t_ms", rec["t_ms"], 1000 + idx * 10),
            ("flags", rec["flags"], flags),
            ("acc", tuple(round(v, 9) for v in rec["acc_g"]), tuple(round(v, 9) for v in exp_acc)),
            ("gyro", tuple(round(v, 6) for v in rec["gyro_dps"]), tuple(round(v, 6) for v in exp_gyro)),
            ("roll", round(rec["roll"], 6), round(exp_rpy[0], 6)),
            ("pitch", round(rec["pitch"], 6), round(exp_rpy[1], 6)),
            ("yaw", round(rec["yaw"], 6), round(exp_rpy[2], 6)),
        ]
        for name, got, want in checks:
            if got != want:
                print("  [!!] case%d 字段 %s 不符: got=%s want=%s" % (idx, name, got, want))
                ok = False
    else:
        print("  [!!] case%d 解析失败" % idx)
    allok = allok and ok
    print("  [%s] case%d acc=%s gyro=%s rpy=%s flags=0x%04X" % (
        "OK" if ok else "!!", idx, acc, gyro, rpy, flags))

print()
print("=" * 72)
print("负例: 畸形帧必须被拒绝")
print("=" * 72)
good = build_frame(1, 1000, (0, 0, 2048), (0, 0, 0), (0, 0, 0), 0)
negatives = [
    ("帧头错误", bytes([0x00, 0x55]) + good[2:]),
    ("类型错误", good[:2] + bytes([0x02]) + good[3:]),
    ("CRC 被改", good[:30] + bytes([(good[30] ^ 0xFF), good[31]])),
    ("长度不足", good[:20]),
    ("全 0xFF", bytes([0xFF] * 32)),
]
for name, f in negatives:
    r = parse_frame(f)
    ok = (r is None)
    allok = allok and ok
    print("  [%s] %s -> %s" % ("OK" if ok else "!!", name, "被拒绝" if r is None else "竟然通过了!"))

print()
print("=" * 72)
print("结论: %s" % ("解析器与协议一致 ✔" if allok else "存在问题 ✘"))
print("=" * 72)
sys.exit(0 if allok else 1)
