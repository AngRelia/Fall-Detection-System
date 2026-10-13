#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
serial_logger.py —— 节点 A 串口数据采集与协议校验

用途:
  1. 解析 32 字节定长帧 (见 docs/串口帧协议.md)
  2. 长稳测试: 统计帧率 / CRC 错误 / 丢帧
  3. 存 CSV, 供后续 TinyML 特征提取

用法:
  python serial_logger.py --port COM7                     # 只统计, 打印进度
  python serial_logger.py --port COM7 --csv out.csv       # 同时存 CSV
  python serial_logger.py --port COM7 --seconds 1800      # 跑 30 分钟长稳
  python serial_logger.py --port COM7 --dump 5            # 打印前 5 帧原始字节

依赖: pyserial  (pip install pyserial)
"""

import argparse
import csv
import struct
import sys
import time
from collections import deque

# 注意: pyserial 延迟到 main() 里导入。
# 这样本模块的解析函数(parse_frame/crc16_modbus)可以在没有 pyserial 的环境下
# 被 unit test 导入 —— 见 tools/test_proto_l1.py。

# ---------------------------------------------------------------- 协议常量
HDR0, HDR1 = 0xAA, 0x55
FRAME_LEN = 32
CRC_OFF = 30
CRC_START = 2
CRC_LEN = 28

ACC_SCALE = 16.0 / 32768.0      # 原始计数 -> g
GYRO_SCALE = 2000.0 / 32768.0   # 原始计数 -> dps
RPY_SCALE = 0.01                # -> 度

FLAG_BITS = [
    (0, "FUSION_READY"), (1, "MAG_ENABLED"), (2, "GYRO_AZ_DONE"),
    (3, "HEADING_SETTLING"), (4, "MAG_REF_VALID"), (5, "CALIB_OK"),
    (6, "MAG_INIT_ERR"),
]


def crc16_modbus(data: bytes) -> int:
    """多项式 0x8005(反射 0xA001), 初值 0xFFFF, 输入/输出反射, 结果异或 0"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def parse_frame(f: bytes):
    """解析一帧。返回 dict 或 None(格式/校验失败)"""
    if len(f) != FRAME_LEN or f[0] != HDR0 or f[1] != HDR1:
        return None
    if f[2] != 0x01:
        return None
    crc_want = struct.unpack_from("<H", f, CRC_OFF)[0]
    crc_got = crc16_modbus(f[CRC_START:CRC_START + CRC_LEN])
    if crc_want != crc_got:
        return None

    seq, ts = struct.unpack_from("<BI", f, 3)
    acc_raw = struct.unpack_from("<3h", f, 8)
    gyro_raw = struct.unpack_from("<3h", f, 14)
    rpy = struct.unpack_from("<3h", f, 20)
    flags = struct.unpack_from("<I", f, 26)[0]

    return {
        "seq": seq,
        "t_ms": ts,
        "acc_g": tuple(v * ACC_SCALE for v in acc_raw),
        "gyro_dps": tuple(v * GYRO_SCALE for v in gyro_raw),
        "roll": rpy[0] * RPY_SCALE,
        "pitch": rpy[1] * RPY_SCALE,
        "yaw": rpy[2] * RPY_SCALE,
        "flags": flags,
    }


def flags_str(flags: int) -> str:
    on = [n for b, n in FLAG_BITS if flags & (1 << b)]
    return ",".join(on) if on else "-"


def main():
    ap = argparse.ArgumentParser(description="节点 A 串口采集与协议校验")
    ap.add_argument("--port", required=True, help="串口号, 如 COM7 或 /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--csv", default=None, help="输出 CSV 路径")
    ap.add_argument("--seconds", type=float, default=0, help="采集时长(秒), 0=一直跑")
    ap.add_argument("--dump", type=int, default=0, help="打印前 N 帧原始字节后退出")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    try:
        import serial
    except ImportError:
        print("需要 pyserial:  pip install pyserial", file=sys.stderr)
        sys.exit(1)

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    print("已打开 %s @ %d" % (args.port, args.baud))

    buf = bytearray()
    n_ok = 0
    n_crc_err = 0
    n_resync = 0
    n_seq_gap = 0
    last_seq = None
    t0 = time.time()
    t_last_report = t0
    rate_win = deque(maxlen=100)      # 最近 100 帧的到达时刻, 算瞬时帧率
    ts_prev = None
    dt_list = deque(maxlen=2000)      # 帧间隔, 用于判断等间隔性

    fout = None
    writer = None
    if args.csv:
        fout = open(args.csv, "w", newline="", encoding="utf-8")
        writer = csv.writer(fout)
        writer.writerow(["t_ms", "seq", "ax_g", "ay_g", "az_g",
                         "gx_dps", "gy_dps", "gz_dps",
                         "roll", "pitch", "yaw", "flags"])

    dumped = 0
    try:
        while True:
            if args.seconds and (time.time() - t0) >= args.seconds:
                break
            chunk = ser.read(4096)
            if not chunk:
                continue
            buf.extend(chunk)

            # 找帧头 -> 定长切帧 -> 校验
            while True:
                # 定位帧头
                i = buf.find(bytes([HDR0, HDR1]))
                if i < 0:
                    if len(buf) > 1:
                        del buf[:-1]
                    break
                if i > 0:
                    n_resync += 1
                    del buf[:i]
                if len(buf) < FRAME_LEN:
                    break

                frame = bytes(buf[:FRAME_LEN])
                rec = parse_frame(frame)
                if rec is None:
                    n_crc_err += 1
                    del buf[:1]          # 丢掉这个假帧头, 继续找
                    continue
                del buf[:FRAME_LEN]
                n_ok += 1
                rate_win.append(time.time())

                if args.dump and dumped < args.dump:
                    dumped += 1
                    print("  帧%d: %s" % (dumped, frame.hex(" ").upper()))
                    print("        seq=%d t=%dms acc=%s gyro=%s rpy=(%.2f,%.2f,%.2f) flags=%s"
                          % (rec["seq"], rec["t_ms"],
                             tuple(round(v, 3) for v in rec["acc_g"]),
                             tuple(round(v, 1) for v in rec["gyro_dps"]),
                             rec["roll"], rec["pitch"], rec["yaw"],
                             flags_str(rec["flags"])))
                    if dumped >= args.dump:
                        return

                # 序号连续性与时间戳间隔
                if last_seq is not None:
                    exp = (last_seq + 1) & 0xFF
                    if rec["seq"] != exp:
                        n_seq_gap += 1
                last_seq = rec["seq"]
                if ts_prev is not None:
                    dt = (rec["t_ms"] - ts_prev) & 0xFFFFFFFF
                    if 0 < dt < 1000:
                        dt_list.append(dt)
                ts_prev = rec["t_ms"]

                if writer:
                    writer.writerow([rec["t_ms"], rec["seq"],
                                     "%.4f" % rec["acc_g"][0], "%.4f" % rec["acc_g"][1], "%.4f" % rec["acc_g"][2],
                                     "%.2f" % rec["gyro_dps"][0], "%.2f" % rec["gyro_dps"][1], "%.2f" % rec["gyro_dps"][2],
                                     "%.2f" % rec["roll"], "%.2f" % rec["pitch"], "%.2f" % rec["yaw"],
                                     rec["flags"]])

            # 周期性汇报
            now = time.time()
            if not args.quiet and (now - t_last_report) >= 5.0:
                elapsed = now - t0
                rate = n_ok / elapsed if elapsed > 0 else 0
                inst = 0.0
                if len(rate_win) >= 2:
                    span = rate_win[-1] - rate_win[0]
                    if span > 0:
                        inst = (len(rate_win) - 1) / span
                # 帧间隔统计
                if dt_list:
                    avg_dt = sum(dt_list) / len(dt_list)
                    mx = max(dt_list)
                    mn = min(dt_list)
                else:
                    avg_dt = mx = mn = 0
                print("[%6.0fs] 帧=%6d  平均=%.1fHz  瞬时=%.1fHz  CRC错=%d  重同步=%d  丢帧=%d  间隔=%dms[min%d max%d]"
                      % (elapsed, n_ok, rate, inst, n_crc_err, n_resync, n_seq_gap,
                         avg_dt, mn, mx))
                t_last_report = now
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        ser.close()
        if fout:
            fout.close()

    # ---------------------------------------------------------- 总结
    elapsed = time.time() - t0
    print("\n" + "=" * 72)
    print("采集总结")
    print("=" * 72)
    print("  时长          : %.1f s" % elapsed)
    print("  有效帧        : %d" % n_ok)
    print("  平均帧率      : %.2f Hz" % (n_ok / elapsed if elapsed > 0 else 0))
    print("  CRC 错误      : %d" % n_crc_err)
    print("  重同步次数    : %d" % n_resync)
    print("  序号跳变(丢帧): %d" % n_seq_gap)
    if dt_list:
        dts = sorted(dt_list)
        p50 = dts[len(dts) // 2]
        p99 = dts[int(len(dts) * 0.99)]
        print("  帧间隔 p50/p99: %d / %d ms   (min=%d max=%d)"
              % (p50, p99, dts[0], dts[-1]))
        print("  等间隔性      : %s" % (
            "良好" if (dts[-1] - dts[0]) <= 2 else "有抖动 (上位机需按 timestamp 重采样)"))
    print()
    ok = (n_crc_err == 0 and n_seq_gap == 0 and n_ok > 100)
    print("  验收(CRC=0 且 无丢帧): %s" % ("通过 ✔" if ok else "未通过 ✘"))
    if args.csv:
        print("  CSV 已保存    : %s" % args.csv)


if __name__ == "__main__":
    main()
