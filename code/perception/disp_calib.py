#!/usr/bin/env python3
# disp_calib.py — 用相机自报的障碍物 distance_m 反推 视差→米 的标定常数。
#   原理: grabber 先用 guess focus 出 depth_latest.pgm(尺度未定但 ∝ 真实深度)。
#   在相机检到的障碍物 bbox 处量当前深度, 与相机自报的真实 distance_m 比 → ratio,
#   修正 focus = 当前focus * ratio (baseline 固定; 深度只看 focus*baseline 乘积)。
#   纯标准库, 容器 python3 直接跑。
import json, sys, struct, os

GRAB = sys.argv[1] if len(sys.argv) > 1 else "/apollo/follow_data/runtime/grab"


def read_pgm16(path):
    with open(path, "rb") as f:
        data = f.read()
    if data[:2] != b"P5":
        raise ValueError("not P5 pgm")
    idx, vals = 2, []
    while len(vals) < 3:
        while idx < len(data) and data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b"#":
            while idx < len(data) and data[idx:idx + 1] != b"\n":
                idx += 1
            continue
        s = idx
        while idx < len(data) and not data[idx:idx + 1].isspace():
            idx += 1
        vals.append(int(data[s:idx]))
    w, h, _maxv = vals
    idx += 1  # one whitespace after maxval
    pix = data[idx:idx + w * h * 2]
    arr = struct.unpack(">%dH" % (w * h), pix)  # 16-bit big-endian
    return w, h, arr


def main():
    obs = json.load(open(os.path.join(GRAB, "obstacles_latest.json")))
    cam = json.load(open(os.path.join(GRAB, "camera_status.json")))
    cur_focus = float(cam.get("focus", 0) or 0)
    if cur_focus <= 0:
        print("ERR: 当前 camera_status.focus<=0, grabber 要先用 --focus <guess> --baseline <b> 跑"); return
    obstacles = [o for o in obs.get("obstacles", []) if 0.5 < float(o.get("distance_m", 0)) < 50]
    if not obstacles:
        print("NO_OBSTACLE: 相机当前没检到障碍物, 让相机对着一个清晰物体(如前方的车)再跑"); return
    o = max(obstacles, key=lambda o: o["bbox"][2] * o["bbox"][3])  # 最大框 = 最可靠
    bx, by, bw, bh = o["bbox"]
    dist_m = float(o["distance_m"])
    w, h, arr = read_pgm16(os.path.join(GRAB, "depth_latest.pgm"))
    x0, x1 = max(0, int(bx + bw * .25)), min(w, int(bx + bw * .75))
    y0, y1 = max(0, int(by + bh * .25)), min(h, int(by + bh * .75))
    vals = [arr[yy * w + xx] for yy in range(y0, y1) for xx in range(x0, x1) if arr[yy * w + xx] > 0]
    if not vals:
        print("NO_DEPTH_IN_BBOX: 障碍物框内深度全 0(视差无效?)"); return
    vals.sort()
    med_mm = vals[len(vals) // 2]
    ratio = (dist_m * 1000.0) / med_mm
    print("障碍物: type=%s dist=%.3fm bbox=%s 框内有效深度像素=%d" % (o.get("type"), dist_m, o["bbox"], len(vals)))
    print("当前focus=%.1f 量得深度=%.0fmm 真实=%.0fmm ratio=%.4f" % (cur_focus, med_mm, dist_m * 1000, ratio))
    print("CORRECTED_FOCUS=%.2f" % (cur_focus * ratio))


main()
