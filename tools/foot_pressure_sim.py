#!/usr/bin/env python3
"""
foot_pressure_sim.py — 足底压力传感器 MCU 串口模拟器

模拟 MCU 以 1000Hz 频率通过虚拟串口发送 19 字节固定帧。
帧格式: F2 01 C0 FF EE 0C [12B 大端数据] F1

用法:
    python3 foot_pressure_sim.py [--dev /dev/pts/1] [--rate 1000] [--csv adc.csv]

默认输出到 /dev/pts/1 (socat 虚拟串口对)。
csv 文件格式: 每行 6 个 uint16 AD 值, 逗号分隔, 循环发送。
无 csv 时使用正弦波模拟数据。
"""

import os
import sys
import time
import struct
import argparse
import math
import signal


FRAME_LEN = 19
HEAD = 0xF2
TAIL = 0xF1
SRC  = 0x01
DST  = 0xC0
FUNC = 0xFF
CMD  = 0xEE
DATA_LEN = 12  # 6 * uint16


def build_frame(adc_values):
    """构建 19 字节帧, adc_values 为 6 个 uint16 大端值."""
    buf = bytearray(FRAME_LEN)
    buf[0] = HEAD
    buf[1] = SRC
    buf[2] = DST
    buf[3] = FUNC
    buf[4] = CMD
    buf[5] = DATA_LEN
    # 6 x uint16 大端
    for i, val in enumerate(adc_values):
        v = int(val) & 0xFFFF
        buf[6 + i * 2]     = (v >> 8) & 0xFF
        buf[6 + i * 2 + 1] = v & 0xFF
    buf[18] = TAIL
    return bytes(buf)


def generate_sine_wave(t_ms, channel):
    """生成正弦波 AD 值 (0~4095), channel 决定相位偏移."""
    phase = channel * math.pi / 3.0
    raw = math.sin(t_ms * 0.01 + phase) * 2047.5 + 2047.5
    return int(max(0, min(4095, raw)))


def generate_static(channel):
    """生成静态测试值."""
    values = [1024, 2048, 3072, 500, 1500, 3500]
    return values[channel % 6]


def load_csv(csv_path):
    """加载 CSV 数据文件, 每行 6 个 uint16."""
    frames = []
    with open(csv_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',')
            if len(parts) >= 6:
                vals = [int(p.strip()) for p in parts[:6]]
                frames.append(vals)
    return frames


def main():
    parser = argparse.ArgumentParser(description='Foot Pressure MCU Simulator')
    parser.add_argument('--dev', default='/dev/pts/1',
                        help='Virtual serial device (default: /dev/pts/1)')
    parser.add_argument('--rate', type=int, default=1000,
                        help='Frame rate in Hz (default: 1000)')
    parser.add_argument('--csv', default=None,
                        help='CSV file with ADC values (6 per line)')
    parser.add_argument('--mode', default='sine', choices=['sine', 'static'],
                        help='Data generation mode (default: sine)')
    parser.add_argument('--count', type=int, default=0,
                        help='Stop after N frames (0=forever)')
    args = parser.parse_args()

    # 加载 CSV 或使用合成数据
    csv_frames = []
    if args.csv:
        csv_frames = load_csv(args.csv)
        if not csv_frames:
            print(f"ERROR: no valid frames in {args.csv}")
            sys.exit(1)
        print(f"Loaded {len(csv_frames)} frames from {args.csv}")
        args.mode = 'csv'

    # 打开串口
    print(f"Opening {args.dev} ...")
    fd = os.open(args.dev, os.O_RDWR | os.O_NOCTTY)
    if fd < 0:
        print(f"ERROR: cannot open {args.dev}")
        sys.exit(1)
    print(f"Opened {args.dev} OK, rate={args.rate}Hz")

    running = True

    def sig_handler(sig, frame):
        nonlocal running
        running = False
        print("\nStopping...")

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    period_s = 1.0 / args.rate
    frame_idx = 0
    t0 = time.monotonic()
    next_wake = t0 + period_s

    sent = 0
    errors = 0

    print("Sending frames... (Ctrl+C to stop)")

    while running:
        if args.count > 0 and sent >= args.count:
            break

        # 生成 AD 值
        if args.mode == 'csv':
            adc = csv_frames[frame_idx % len(csv_frames)]
        elif args.mode == 'sine':
            t_ms = (time.monotonic() - t0) * 1000.0
            adc = [generate_sine_wave(t_ms, i) for i in range(6)]
        else:  # static
            adc = [generate_static(i) for i in range(6)]

        # 构建并发送帧
        frame = build_frame(adc)
        try:
            n = os.write(fd, frame)
            if n != FRAME_LEN:
                errors += 1
        except OSError as e:
            errors += 1
            if errors <= 3:
                print(f"Write error: {e}")

        sent += 1
        frame_idx += 1

        # 每秒打印一次统计
        if sent % args.rate == 0:
            elapsed = time.monotonic() - t0
            actual_rate = sent / elapsed
            print(f"[{sent}] sent={sent} rate={actual_rate:.0f}Hz errors={errors}")

        # 精确定时
        next_wake += period_s
        now = time.monotonic()
        sleep_s = next_wake - now
        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            # 超时, 重新对齐
            next_wake = time.monotonic() + period_s

    elapsed = time.monotonic() - t0
    actual_rate = sent / elapsed if elapsed > 0 else 0
    print(f"\nDone. sent={sent} errors={errors} actual_rate={actual_rate:.0f}Hz elapsed={elapsed:.2f}s")

    os.close(fd)


if __name__ == '__main__':
    main()
