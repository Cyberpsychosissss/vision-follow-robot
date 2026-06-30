#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
can_logger.py  --  RUNS ON THE HOST (Jetson, Python 3, SocketCAN).

Standalone listen-only CAN logger. Writes raw_can.csv into a session folder that
is SHARED with the in-docker ros_camera_collector.py (e.g. under /apollo/data),
so the camera frames and CAN frames land in the same session and can be aligned
later by tools/sync_dataset.py (both stamp wall-clock time).

SAFETY: receive-only. No bus.send() anywhere. Never controls the vehicle.

Usage (host):
    python3 tools/can_logger.py --out /apollo/data/follow_dataset/run1 \
        --channel can0 --bitrate 500000

Stop with Ctrl+C (flushes and exits cleanly).
"""

import os
import csv
import sys
import time
import argparse


def now_ns():
    return int(time.time() * 1_000_000_000)


def main():
    ap = argparse.ArgumentParser(description="Listen-only SocketCAN logger.")
    ap.add_argument("--out", required=True,
                    help="session dir (same one passed to ros_camera_collector.py)")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--no-message-warn-sec", type=float, default=5.0)
    args = ap.parse_args()

    try:
        import can  # python-can (pin ==3.3.4 on Python 3.6)
    except Exception as e:
        sys.stderr.write("[ERROR] python-can not installed: %s\n"
                         "  pip3 install --user 'python-can==3.3.4'\n" % e)
        sys.exit(1)

    out = args.out
    if not os.path.isdir(out):
        os.makedirs(out)
    path = os.path.join(out, "raw_can.csv")
    new_file = not os.path.exists(path)
    fh = open(path, "a", newline="")
    w = csv.writer(fh)
    if new_file:
        w.writerow(["timestamp_ns", "can_timestamp", "arbitration_id",
                    "is_extended_id", "dlc", "data_hex"])
        fh.flush()

    # 'bustype=' works on both python-can 3.x (needed on Py3.6) and 4.x.
    try:
        bus = can.interface.Bus(channel=args.channel, bustype=args.interface)
    except TypeError:
        bus = can.interface.Bus(channel=args.channel, interface=args.interface)
    except Exception as e:
        sys.stderr.write("[ERROR] cannot open %s: %s\n"
                         "  Bring it up: sudo ip link set %s up type can bitrate 500000\n"
                         % (args.channel, e, args.channel))
        sys.exit(1)

    print("[can_logger] listening on %s (receive-only) -> %s" % (args.channel, path))
    print("[can_logger] Ctrl+C to stop.")
    count = 0
    last_msg = time.time()
    last_status = 0.0
    last_warn = 0.0
    try:
        while True:
            msg = bus.recv(timeout=0.5)
            now = time.time()
            if msg is None:
                if now - last_msg > args.no_message_warn_sec and now - last_warn > 5.0:
                    last_warn = now
                    print("[can_logger] WARN: no CAN message for %ds "
                          "(is the car powered / wired?)" % int(now - last_msg))
                continue
            last_msg = now
            w.writerow([now_ns(), "%.6f" % msg.timestamp,
                        hex(msg.arbitration_id), int(bool(msg.is_extended_id)),
                        msg.dlc, msg.data.hex()])
            fh.flush()
            count += 1
            if now - last_status > 5.0:
                last_status = now
                print("[can_logger] %d messages logged" % count)
    except KeyboardInterrupt:
        print("\n[can_logger] stopping...")
    finally:
        try:
            fh.flush(); fh.close()
        except Exception:
            pass
        try:
            bus.shutdown()
        except Exception:
            pass
        print("[can_logger] done. %d CAN messages -> %s" % (count, path))


if __name__ == "__main__":
    main()
