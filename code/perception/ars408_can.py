#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ars408_can.py — 大陆 ARS408-21 毫米波雷达: 只读 CAN 解目标列表 → runtime/radar_objects.json

零成本接入
----------
这颗雷达出厂就装在车上、**上电即在 CAN 上实时喷目标列表**, 我们连驱动都不用装,
解码就能用。2026-07-20 只读普查时从总线上认出来的:

  ARS408 目标列表报文默认 ID 是 0x60A~0x60D, 而 **sensor_id 每加 1 全部报文偏移
  0x10**。本车总线上是 0x61A~0x61D → sensor_id = 1。旁证: 0x211 = 0x201+0x10
  (RadarState), 0x710 = 0x700+0x10 (VersionID), 全部自洽。

  0x61A = Obj_0_Status   : byte0 = 目标数(实测 14~16), byte1-2 = MeasCounter 单调递增
  0x61B = Obj_1_General  : byte0 = Obj_ID, 逐目标轮转发送(实测 00→0B)
  0x61C = Obj_2_Quality  : 各测量量的 rms + 存在概率(本模块暂未解, 见文末 TODO)
  0x61D = Obj_3_Extended : 长宽/朝向(本模块暂未解)

位域(Motorola 大端, 起始位为 MSB)已用真实报文验证:
  Obj_ID       0|8      Obj_DistLong 15|13 (0.2, -500)    Obj_DistLat 18|11 (0.2, -204.6)
  Obj_VrelLong 47|10 (0.25,-128)  Obj_VrelLat 53|9 (0.25,-64)
  Obj_DynProp  60|3               Obj_RCS     63|8 (0.5, -64)

验证时车停在室内静止场景, 解出来**所有目标 Vlong=+0.00 / Vlat≈0 / 动态属性=静止**
—— 速度与动态属性是两个独立字段同时符合预期, 比只验距离强得多。

用途定位
--------
250m / ±60°, 直接给距离+横向+径向速度+RCS, 对**运动目标**敏感且不受雨雾灰尘影响。
但分辨率低、静止目标易被滤除、金属结构室内多径鬼影多(实测见过 38m 处 -49.4m 横向、
RCS 仅 +8 的鬼影)。所以定位是 **远距目标预警 + 速度估计 + 给雷达/视觉做交叉验证**,
不做精细避障 —— 精细避障是激光雷达的活(见 lslidar_c16_udp.py)。

只读: 本模块只 `candump`, 不发任何 CAN 帧。
环境: 宿主机 python3.6, 无第三方依赖(连 numpy 都不需要)。
"""
from __future__ import print_function

import argparse
import json
import os
import subprocess
import sys
import time

RUNTIME = os.environ.get("FOLLOW_RUNTIME",
                         "/home/nvidia/work/AutoApollo/apollo/follow_data/runtime")
OUT_FILE = os.path.join(RUNTIME, "radar_objects.json")

ID_STATUS = "61A"      # Obj_0_Status
ID_GENERAL = "61B"     # Obj_1_General

DYN_PROP = {0: "moving", 1: "stationary", 2: "oncoming", 3: "stationary_candidate",
            4: "unknown", 5: "crossing_stationary", 6: "crossing_moving", 7: "stopped"}
# 跟人时真正关心的类别(排除静止杂波与疑似静止)
MOVING_DYN = (0, 2, 6)


def decode_general(d):
    """0x61B 8 字节 → 一个目标 dict。d 为 8 个 int。"""
    lon = (((d[1] << 5) | (d[2] >> 3)) & 0x1FFF) * 0.2 - 500.0
    lat = (((d[2] & 0x07) << 8) | d[3]) * 0.2 - 204.6
    vlon = (((d[4] << 2) | (d[5] >> 6)) & 0x3FF) * 0.25 - 128.0
    vlat = ((((d[5] & 0x3F) << 3) | (d[6] >> 5)) & 0x1FF) * 0.25 - 64.0
    dyn = d[6] & 0x07
    return {
        "id": d[0],
        "lon": round(lon, 2),          # 纵向距离 m (车头方向为正)
        "lat": round(lat, 2),          # 横向距离 m (左正右负, 与车体系一致待外参确认)
        "vlon": round(vlon, 2),        # 纵向相对速度 m/s
        "vlat": round(vlat, 2),        # 横向相对速度 m/s
        "dyn": dyn,
        "dyn_name": DYN_PROP.get(dyn, "?"),
        "rcs": round(d[7] * 0.5 - 64.0, 1),   # dBm²
    }


def decode_status(d):
    """0x61A → (目标数, 测量计数器)"""
    return d[0], (d[1] << 8) | d[2]


def _parse_line(ln):
    """candump 一行 → (can_id_str, [int,...]) ; 解不了返回 (None, None)"""
    f = ln.split()
    di = None
    for i, tok in enumerate(f):
        if tok.startswith("[") and tok.endswith("]"):
            di = i + 1
            break
    if di is None or di < 2:
        return None, None
    try:
        return f[di - 2], [int(x, 16) for x in f[di:]]
    except ValueError:
        return None, None


def write_json(d):
    try:
        if not os.path.isdir(RUNTIME):
            os.makedirs(RUNTIME)
        tmp = OUT_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(d, f)
        os.replace(tmp, OUT_FILE)
    except Exception:
        pass


def run(channel="can0", verbose=False, once=False):
    """持续读 candump 解目标列表。

    帧同步: ARS408 每个测量周期先发 Obj_0_Status(含目标数 N + MeasCounter),
    紧接着发 N 个 Obj_1_General。所以**收到 61A 就意味着上一帧收完了** ——
    以此切帧, 比按数量凑齐更稳(丢帧时不会把两帧混在一起)。
    """
    # 用**持久管道**而不是反复起 subprocess: 这颗雷达 ~37Hz, 每次重启 candump 都会
    # 丢掉起停之间的帧, 攒不齐一帧目标。(bms_monitor 那种反复调用的写法只适合低频 BMS)
    p = subprocess.Popen(["candump", channel], stdout=subprocess.PIPE,
                         stderr=subprocess.DEVNULL, universal_newlines=True)
    cur, expect, meas = [], 0, 0
    frames = 0
    try:
        for ln in p.stdout:
            cid, d = _parse_line(ln)
            if cid is None:
                continue
            if cid == ID_STATUS and len(d) >= 3:
                # 上一帧收完 → 落盘
                if cur or frames:
                    moving = [o for o in cur if o["dyn"] in MOVING_DYN]
                    out = {
                        "ts": time.time(), "source": "ars408",
                        "meas_counter": meas,
                        "n_reported": expect, "n_decoded": len(cur),
                        "n_moving": len(moving),
                        "objects": sorted(cur, key=lambda o: o["lon"]),
                    }
                    write_json(out)
                    frames += 1
                    if verbose:
                        print("[%s] 帧%d cnt=%d 报告%d/解出%d 运动%d  最近: %s"
                              % (time.strftime("%H:%M:%S"), frames, meas, expect,
                                 len(cur), len(moving),
                                 ("%.1fm/%+.1fm" % (out["objects"][0]["lon"],
                                                    out["objects"][0]["lat"]))
                                 if out["objects"] else "无"))
                        sys.stdout.flush()
                    if once and frames >= 1:
                        return out
                expect, meas = decode_status(d)
                cur = []
            elif cid == ID_GENERAL and len(d) >= 8:
                cur.append(decode_general(d))
                if len(cur) > 64:      # 防守: ID 异常时别把内存吃光
                    cur = cur[-64:]
    except KeyboardInterrupt:
        pass
    finally:
        try:
            p.terminate()
        except Exception:
            pass
    return None


def main():
    ap = argparse.ArgumentParser(description="ARS408 毫米波目标列表解码(只读 CAN)")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--verbose", action="store_true", help="每帧打一行")
    ap.add_argument("--once", action="store_true", help="解出一帧就退出(冒烟用)")
    a = ap.parse_args()
    out = run(a.channel, verbose=a.verbose or a.once, once=a.once)
    if a.once:
        if out is None:
            sys.exit("❌ 没解出任何一帧 —— can0 起来了吗? 雷达通电了吗?")
        print(json.dumps(out, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()

# TODO(P1 之后): 0x61C Obj_2_Quality 里有 Obj_ProbOfExist(存在概率), 是过滤室内多径
#   鬼影最直接的手段; 0x61D Obj_3_Extended 给长宽朝向。需要官方 dbc 核对位域后再加。
