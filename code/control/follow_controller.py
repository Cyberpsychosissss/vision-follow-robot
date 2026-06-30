#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
follow_controller.py — 跟随控制器: 读 target.json → 控制律(与人保持 3~4m) → car_control 发 CAN → 写 follow_status.json

数据流(共享文件 IPC):
  目标源(容器内, 二选一):
    - parse_zkhy_obs.py --publish   (相机障碍物: 给真实米距 dist_m + 横向米 lateral_m, 推荐)
    - yolo_person.py    --publish   (YOLO: 给归一 off_x + 框高 box_h_norm, 无真实距离)
        都把目标写到 RUNTIME/target.json
  本控制器(宿主机):  读 target.json → 算(speed,steer) → car_control.CarController(can0) → 写 follow_status.json
  web_ui.py(宿主机): 读 follow_status.json + target.json → 显示跟随面板

控制律(保持 3~4m):
  距离(speed): 有真实米距时 —— dist>4m: 前进, 越远越快(夹硬限速)=FOLLOW; 3~4m: 停=HOLD; <3m: 停(太近)=STOP。
               只有 YOLO 框高时 —— 用 box_h_norm 当距离代理(需标定, 不准, 仅兜底)。
  转向(steer): 有横向米时 steer=atan2(lateral,dist) 的真实方位角×增益; 否则 steer=off_x×增益。死区防抖。
  目标丢失/数据过期 → 停 + SEARCH。

安全: 复用 car_control 的硬限速 + 死人开关; 默认 dry-run(不发帧)。--arm 才真发(需车轮架空+人在场)。
"""
from __future__ import print_function
import os
import sys
import time
import json
import math
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from car_control import CarController, MAX_SPEED, MAX_STEER, ABS_MAX_SPEED

# ---- 共享文件路径(宿主机视角; 容器内目标源写的是 /apollo/follow_data/runtime/, 同一物理目录) ----
RUNTIME = os.environ.get("FOLLOW_RUNTIME",
                         "/home/nvidia/work/AutoApollo/apollo/follow_data/runtime")
TARGET_FILE = os.path.join(RUNTIME, "target.json")
STATUS_FILE = os.path.join(RUNTIME, "follow_status.json")
CONFIG_FILE = os.path.join(RUNTIME, "follow_config.json")   # web 写保持距离, 运行时热读

# ---- 跟随参数(可调) ----
DESIRED_MIN = 2.0          # m  保持距离下限(更近就停)
DESIRED_MAX = 4.0          # m  保持距离上限(更远就追)
KV = 0.4                   # 速度增益: speed = KV*(dist-DESIRED_MAX), KV=0.4 → 超出1m即到限速
KP_BEARING = 1.5           # 转向增益(真实方位角, 度→度)
KSTEER_OFFX = 60.0         # 转向增益(YOLO 归一 off_x → 度)
STEER_DEADZONE_DEG = 1.5   # 方位角死区(度), 内不转
TARGET_TIMEOUT = 0.5       # s  target.json 超过这么旧就当丢目标
LOST_HOLD = 0.0            # 丢目标时车速(0=停)
LOOP_HZ = 20.0

# YOLO 兜底(无真实距离)用框高代理距离, 需按实测标定:
BH_FAR = 0.45              # 归一框高 < 此值 → 人远 → 前进
BH_NEAR = 0.75            # 归一框高 > 此值 → 人近 → 停


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def read_desired():
    """从 runtime/follow_config.json 读保持距离(web 可改, 热生效); 没有/非法返回 None。"""
    try:
        c = json.load(open(CONFIG_FILE))
        mn, mx = float(c['desired_min']), float(c['desired_max'])
        if 0.3 < mn < mx < 20:
            return mn, mx
    except Exception:
        pass
    return None


def read_target():
    try:
        with open(TARGET_FILE) as f:
            t = json.load(f)
    except Exception:
        return None
    if (time.time() - t.get("ts", 0)) > TARGET_TIMEOUT or not t.get("valid"):
        return None
    return t


def compute_cmd(t):
    """返回 (speed_mps, steer_ctrl_deg, state)。steer 用控制器约定(正=右), car_control 内部会取负成 fr。"""
    if t is None:
        return 0.0, 0.0, "SEARCH"

    dist = t.get("dist_m")
    lateral = t.get("lateral_m")
    off_x = t.get("off_x")
    box_h = t.get("box_h_norm")

    # ---- 转向 ----
    if lateral is not None and dist is not None and dist > 0.1:
        bearing_deg = math.degrees(math.atan2(lateral, dist))   # 正=人在右
        if abs(bearing_deg) < STEER_DEADZONE_DEG:
            steer = 0.0
        else:
            steer = KP_BEARING * bearing_deg
    elif off_x is not None:
        steer = 0.0 if abs(off_x) < 0.06 else KSTEER_OFFX * off_x
    else:
        steer = 0.0
    steer = _clamp(steer, -MAX_STEER, MAX_STEER)

    # ---- 速度(保持 3~4m) ----
    if dist is not None and dist > 0.1:
        if dist > DESIRED_MAX:
            speed = KV * (dist - DESIRED_MAX); state = "FOLLOW"
        elif dist < DESIRED_MIN:
            speed = 0.0; state = "STOP_NEAR"
        else:
            speed = 0.0; state = "HOLD"
    elif box_h is not None:                 # YOLO 兜底: 框高代理距离
        if box_h < BH_FAR:
            speed = KV * 1.0; state = "FOLLOW"
        elif box_h > BH_NEAR:
            speed = 0.0; state = "STOP_NEAR"
        else:
            speed = 0.0; state = "HOLD"
    else:
        speed = 0.0; state = "HOLD"

    speed = _clamp(speed, 0.0, MAX_SPEED)
    return speed, steer, state


def write_status(d):
    try:
        tmp = STATUS_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(d, f)
        os.replace(tmp, STATUS_FILE)
    except Exception:
        pass


def main():
    ap = argparse.ArgumentParser(description="跟随控制器(保持3-4m)")
    ap.add_argument("--arm", action="store_true", help="真发帧(默认dry-run)。车轮架空+人在场!")
    ap.add_argument("--steer-only", action="store_true", help="只转向不前进(速度锁0): 最安全的实车测试, 也可确认转向符号")
    ap.add_argument("--max-speed", type=float, default=MAX_SPEED)
    ap.add_argument("--channel", default="can0")
    args = ap.parse_args()

    try:
        os.makedirs(RUNTIME)
    except OSError:
        pass

    ctl = CarController(channel=args.channel, dry_run=not args.arm,
                        max_speed=args.max_speed, verbose=False)
    if args.arm:
        mode = "只转向(不前进, 速度锁0)" if args.steer_only else "前进+转向"
        print("⚠ ARMED 跟随[%s]: 真发帧到 %s, 保持 %.1f~%.1fm。%s 3秒后开始, Ctrl-C停。"
              % (mode, args.channel, DESIRED_MIN, DESIRED_MAX,
                 "人站相机前即可。" if args.steer_only else "车轮架空+人在场!"))
        time.sleep(3)
    else:
        print("DRY-RUN 跟随(不发帧): 保持 %.1f~%.1fm。读 %s" % (DESIRED_MIN, DESIRED_MAX, TARGET_FILE))
    ctl.start()

    period = 1.0 / LOOP_HZ
    last_log = 0.0
    try:
        while True:
            d = read_desired()                       # web 可热改保持距离
            if d:
                globals()['DESIRED_MIN'], globals()['DESIRED_MAX'] = d
            t = read_target()
            speed, steer, state = compute_cmd(t)
            if args.steer_only:                          # 只转向不前进: 速度锁0
                speed = 0.0
                if state == "FOLLOW":
                    state = "STEER_ONLY"
            ctl.set_cmd_from_follow(speed, steer)        # 内部取负成 fr 转向
            now = time.time()
            status = {
                "ts": now, "armed": bool(args.arm), "steer_only": bool(args.steer_only), "state": state,
                "cmd_speed": round(speed, 3), "cmd_steer": round(steer, 2),
                "target_valid": t is not None,
                "dist_m": (round(t["dist_m"], 2) if t and t.get("dist_m") is not None else None),
                "lateral_m": (round(t["lateral_m"], 2) if t and t.get("lateral_m") is not None else None),
                "off_x": (round(t["off_x"], 3) if t and t.get("off_x") is not None else None),
                "source": (t.get("source") if t else None),
                "desired": [DESIRED_MIN, DESIRED_MAX],
            }
            write_status(status)
            if now - last_log >= 1.0:
                ds = ("dist=%.2fm" % t["dist_m"]) if (t and t.get("dist_m") is not None) else \
                     (("box_h=%.2f" % t["box_h_norm"]) if (t and t.get("box_h_norm") is not None) else "无距离")
                print("[%s] %-9s %s steer=%+.1f speed=%.2f%s" %
                      (time.strftime("%H:%M:%S"), state, ds, steer, speed,
                       "" if args.arm else " (dry)"))
                last_log = now
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nCtrl-C 停。")
    finally:
        write_status({"ts": time.time(), "armed": False, "state": "OFF",
                      "cmd_speed": 0.0, "cmd_steer": 0.0, "target_valid": False})
        ctl.stop()


if __name__ == "__main__":
    main()
