#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# bms_monitor.py — 读 CAN 上的 BMS 帧, 解码电压/电流/容量/充电标志, 写 runtime/bms.json (供 web_ui 显示电量)
# 只读 candump, 不发任何帧。web_ui 启动时会自动拉起它。
import os
import json
import time
import subprocess

RUNTIME = os.environ.get("FOLLOW_RUNTIME", "/home/nvidia/work/AutoApollo/apollo/follow_data/runtime")
BMS_FILE = os.path.join(RUNTIME, "bms.json")
# 48V 系统(13S 锂电)经验阈值, 仅用于粗略 SOC%
V_FULL = 54.6
V_EMPTY = 39.0


def read_once():
    try:
        out = subprocess.check_output(["timeout", "2", "candump", "-n", "80", "can0"],
                                      stderr=subprocess.DEVNULL).decode()
    except Exception:
        return None
    info = flag = None
    for ln in out.splitlines():
        p = ln.split()
        if len(p) < 11:
            continue
        d = [int(x, 16) for x in p[3:11]]
        if p[1] == "18C4E1EF":
            info = d
        elif p[1] == "18C4E2EF":
            flag = d
    if not info:
        return None
    v = (info[0] | (info[1] << 8)) * 0.01
    c = (info[2] | (info[3] << 8))
    if c >= 0x8000:
        c -= 0x10000
    cap = (info[4] | (info[5] << 8)) * 0.01
    charging = bool(flag and (flag[2] & 0x20))
    soc = max(0.0, min(100.0, (v - V_EMPTY) / (V_FULL - V_EMPTY) * 100.0))
    return {"ts": time.time(), "voltage_v": round(v, 2), "current_a": round(c * 0.01, 2),
            "remaining_ah": round(cap, 2), "charging": charging, "soc_pct": round(soc)}


def main():
    try:
        os.makedirs(RUNTIME)
    except OSError:
        pass
    while True:
        d = read_once()
        if d:
            tmp = BMS_FILE + ".tmp"
            with open(tmp, "w") as f:
                json.dump(d, f)
            os.replace(tmp, BMS_FILE)
        time.sleep(2)


if __name__ == "__main__":
    main()
