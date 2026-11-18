#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
上位机调试工具 —— 串口发指令 + 实时曲线

用法：
    python robot_monitor.py --port COM3
    python robot_monitor.py --port COM3 --plot

功能：
    1. 交互式发指令（V / S / Z / P / R / ?）
    2. 解析遥测帧，实时打印表格
    3. --plot 用 matplotlib 画速度曲线（需 matplotlib）

协议（与 firmware/Inc/protocol.h 一致）：
    发： V <v> <w>\\r\\n  等
    收： T,<ms>,<vL>,<vR>,<tgtL>,<tgtR>,<tickL>,<tickR>,<dutyL>,<dutyR>,<lost>
"""

import argparse
import sys
import time
from collections import deque

try:
    import serial
except ImportError:
    print("需要 pyserial: pip install pyserial")
    sys.exit(1)

try:
    import matplotlib
    import matplotlib.pyplot as plt
    HAS_PLOT = True
except ImportError:
    HAS_PLOT = False


FIELD_NAMES = [
    "ms", "vL", "vR", "tgtL", "tgtR",
    "tickL", "tickR", "dutyL", "dutyR", "lost",
]


class Telemetry:
    """一帧遥测"""

    __slots__ = FIELD_NAMES

    def __init__(self, values):
        for name, v in zip(FIELD_NAMES, values):
            setattr(self, name, v)

    def __repr__(self):
        return (f"v={self.vL:+.3f}/{self.vR:+.3f} "
                f"tgt={self.tgtL:+.3f}/{self.tgtR:+.3f} "
                f"duty={self.dutyL:+5d}/{self.dutyR:+5d}"
                + (" [LINK LOST]" if self.lost else ""))


def parse_line(line):
    """解析一行，返回 Telemetry 或 None"""
    line = line.strip()
    if not line or not line.startswith("T,"):
        return None

    parts = line.split(",")
    if len(parts) != 11:          # 'T' + 10 个字段
        return None

    try:
        vals = [float(x) for x in parts[1:]]
    except ValueError:
        return None

    # 整数字段还原成 int，方便看
    vals[0] = int(vals[0])
    vals[5] = int(vals[5])
    vals[6] = int(vals[6])
    vals[7] = int(vals[7])
    vals[8] = int(vals[8])
    vals[9] = int(vals[9])

    return Telemetry(vals)


def run_console(ser):
    """交互式控制台"""
    print("\n指令：")
    print("  v 0.3 0      线速度 0.3 m/s，角速度 0")
    print("  s            停车")
    print("  z            清零编码器")
    print("  p 1800 20000 0  改 PID")
    print("  q            退出\n")

    while True:
        try:
            cmd = input("cmd> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not cmd:
            continue
        if cmd.lower() in ("q", "quit", "exit"):
            break

        # 允许简写：直接输 "0.3 0" 等价于 "V 0.3 0"
        if cmd[0].isdigit() or cmd[0] in "+-.":
            cmd = "V " + cmd

        ser.write((cmd.upper() + "\r\n").encode())
        time.sleep(0.05)


def run_plot(ser, window_s=10.0):
    """实时曲线：速度 vs 时间"""
    if not HAS_PLOT:
        print("需要 matplotlib: pip install matplotlib")
        return

    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    n = 500
    t_buf = deque(maxlen=n)
    vl_buf = deque(maxlen=n)
    vr_buf = deque(maxlen=n)
    tl_buf = deque(maxlen=n)
    tr_buf = deque(maxlen=n)

    ln_vl, = ax1.plot([], [], 'b-', label='vL')
    ln_vr, = ax1.plot([], [], 'r-', label='vR')
    ax1.set_ylabel("speed (m/s)")
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)

    ln_tl, = ax2.plot([], [], 'b--', label='tgtL')
    ln_tr, = ax2.plot([], [], 'r--', label='tgtR')
    ax2.set_ylabel("target (m/s)")
    ax2.set_xlabel("time (s)")
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)

    print("实时曲线已启动，Ctrl+C 退出")
    t0 = time.time()

    buf = b""
    try:
        while True:
            buf += ser.read(ser.in_waiting or 1)
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                tm = parse_line(raw.decode(errors="ignore"))
                if tm is None:
                    continue

                t = time.time() - t0
                t_buf.append(t)
                vl_buf.append(tm.vL)
                vr_buf.append(tm.vR)
                tl_buf.append(tm.tgtL)
                tr_buf.append(tm.tgtR)

                ln_vl.set_data(t_buf, vl_buf)
                ln_vr.set_data(t_buf, vr_buf)
                ln_tl.set_data(t_buf, tl_buf)
                ln_tr.set_data(t_buf, tr_buf)

                if t_buf:
                    ax1.set_xlim(max(0, t - window_s), t + 0.5)
                    allv = list(vl_buf) + list(vr_buf) + list(tl_buf) + list(tr_buf)
                    lo, hi = min(allv), max(allv)
                    pad = max(0.05, (hi - lo) * 0.2)
                    ax1.set_ylim(lo - pad, hi + pad)

                fig.canvas.draw_idle()
                fig.canvas.flush_events()
    except KeyboardInterrupt:
        pass


def main():
    ap = argparse.ArgumentParser(description="两轮差速小车上位机调试工具")
    ap.add_argument("--port", required=True, help="串口，如 COM3 或 /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200, help="波特率，默认 115200")
    ap.add_argument("--plot", action="store_true", help="显示实时曲线")
    ap.add_argument("--log", help="把遥测帧存到这个文件（csv）")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    print(f"已连接 {args.port} @ {args.baud}")

    # 清掉缓冲里的启动信息
    ser.reset_input_buffer()
    time.sleep(0.2)

    log_f = None
    if args.log:
        log_f = open(args.log, "w", encoding="utf-8")
        log_f.write("time_s," + ",".join(FIELD_NAMES) + "\n")
        print(f"日志将写入 {args.log}")

    try:
        if args.plot:
            run_plot(ser)
        else:
            run_console(ser)
    except KeyboardInterrupt:
        pass
    finally:
        if log_f:
            log_f.close()
        ser.close()
        print("已断开")


if __name__ == "__main__":
    main()
