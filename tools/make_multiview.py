#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_multiview.py — 车上 Demo + 手机第三人称 → 1920x1080 四宫格成片

    ┌──────────────┬──────────────┐
    │  摄像头       │   视差        │   ← 从车上 demo 里裁出来(坐标读 meta.json)
    ├──────────────┼──────────────┤
    │  手机第三人称  │  俯视 mini-map │   ← 第4格 P1 做出 mini-map 前先放数据栏占位
    └──────────────┴──────────────┘

时间对齐
--------
车上 demo 的每帧墙钟在 meta.json 的 first_frame_ts; 手机视频 PTS=0 对应的车钟由
tools/sync_phone.py 解出写在 <phone>.sync.json 的 t0_car。两者一减就是手机流要平移的量:

    d = first_frame_ts - t0_car      # 车 demo 起点在手机视频里的位置(秒)
    d > 0 → 手机先开的录, 裁掉手机前面 d 秒
    d < 0 → 车先开的录, 手机前面补 -d 秒黑帧

顺带把码率压下来: 车上 VideoWriter 用的是 mp4v, 一段几分钟的 demo 能到 800MB+;
这里 H.264 CRF 23 重编, 体积通常小一个数量级。

用法
----
  python3 tools/make_multiview.py --car demos/demo_20260720_140000.mp4 \
                                  --phone phone.mov -o out.mp4
  # meta.json / sync.json 默认按同名推断, 也可 --meta / --sync 显式指定
  # 没有手机视频时也能用(第3格留黑), 便于先验证车端布局:
  python3 tools/make_multiview.py --car demos/xxx.mp4 -o out.mp4

依赖: ffmpeg(Mac 上已有)。全部工作交给一条 filter_complex, 不逐帧解码。
"""
import argparse
import json
import os
import subprocess
import sys

CELL_W, CELL_H = 960, 540          # 每格; 总画布 1920x1080


def _load(path, what):
    try:
        with open(path) as f:
            return json.load(f)
    except Exception as e:
        sys.exit("读不了%s: %s (%s)" % (what, path, e))


def _crop(layout, key):
    """meta.layout[key] = [w,h,x,y] → ffmpeg crop 参数字符串"""
    w, h, x, y = layout[key]
    return "crop=%d:%d:%d:%d" % (w, h, x, y)


def _probe_dur(path):
    try:
        out = subprocess.check_output(
            ["ffprobe", "-v", "error", "-show_entries", "format=duration",
             "-of", "default=nw=1:nk=1", path], text=True).strip()
        return float(out)
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser(description="车上 Demo + 手机视角 → 四宫格成片")
    ap.add_argument("--car", required=True, help="车上 demo 视频 (demos/demo_*.mp4)")
    ap.add_argument("--meta", help="对应的 .meta.json (默认同名推断)")
    ap.add_argument("--phone", help="手机第三人称视频 (可省, 省则第3格留黑)")
    ap.add_argument("--sync", help="手机的 .sync.json (默认同名推断)")
    ap.add_argument("-o", "--out", required=True, help="输出 mp4")
    ap.add_argument("--panel4", default="sidebar", choices=["sidebar", "black", "minimap"],
                    help="第4格内容: sidebar=数据栏(默认, P1 前的占位) / black / minimap(P1 后)")
    ap.add_argument("--fps", type=float, default=30.0,
                    help="输出帧率(默认30: 车画面本就10fps, 但手机视角流畅度对观感重要)")
    ap.add_argument("--crf", type=int, default=23)
    ap.add_argument("--dry-run", action="store_true", help="只打印 ffmpeg 命令不执行")
    a = ap.parse_args()

    if not os.path.isfile(a.car):
        sys.exit("车上 demo 不存在: %s" % a.car)
    meta_path = a.meta or (os.path.splitext(a.car)[0] + ".meta.json")
    meta = _load(meta_path, "meta.json")
    layout = meta.get("layout") or {}
    for k in ("camera", "disparity", "sidebar"):
        if k not in layout:
            sys.exit("meta.json 缺 layout.%s —— 是不是旧版 web_ui 录的? 需 schema>=1" % k)

    car_dur = _probe_dur(a.car) or 0.0
    car_t0 = meta.get("first_frame_ts") or meta.get("start_ts")
    if car_t0 is None:
        sys.exit("meta.json 里没有 first_frame_ts/start_ts, 无法对齐")

    # ---------------- 组装 filter_complex ----------------
    ins = ["-i", a.car]
    parts = [
        "[0:v]%s,scale=%d:%d,setsar=1[cam]" % (_crop(layout, "camera"), CELL_W, CELL_H),
        "[0:v]%s,scale=%d:%d,setsar=1[dis]" % (_crop(layout, "disparity"), CELL_W, CELL_H),
    ]

    # --- 第3格: 手机 ---
    shift = None
    if a.phone:
        if not os.path.isfile(a.phone):
            sys.exit("手机视频不存在: %s" % a.phone)
        sync_path = a.sync or (os.path.splitext(a.phone)[0] + ".sync.json")
        sync = _load(sync_path, "sync.json(先跑 tools/sync_phone.py)")
        if "t0_car" not in sync:
            sys.exit("sync.json 里没有 t0_car")
        shift = car_t0 - float(sync["t0_car"])       # 车 demo 起点在手机视频里的位置
        if shift >= 0:
            ins = ["-ss", "%.4f" % shift, "-i", a.phone] + ins
            car_idx, ph_idx, pre = 1, 0, ""
        else:
            ins = ["-i", a.phone] + ins
            car_idx, ph_idx = 1, 0
            pre = "tpad=start_duration=%.4f:start_mode=add:color=black," % (-shift)
        parts = [p.replace("[0:v]", "[%d:v]" % car_idx) for p in parts]
        # 竖屏/异比例手机视频: 等比缩放后居中补黑, 不拉伸变形
        parts.append(
            "[%d:v]%ssetpts=PTS-STARTPTS,scale=%d:%d:force_original_aspect_ratio=decrease,"
            "pad=%d:%d:(ow-iw)/2:(oh-ih)/2:color=black,setsar=1[phone]"
            % (ph_idx, pre, CELL_W, CELL_H, CELL_W, CELL_H))
        p3 = "[phone]"
    else:
        car_idx = 0
        parts.append("color=c=black:s=%dx%d:d=%.3f,setsar=1[phone]"
                     % (CELL_W, CELL_H, max(car_dur, 0.1)))
        p3 = "[phone]"

    # --- 第4格 ---
    if a.panel4 == "sidebar":
        # 数据栏是 400x900 的竖条, 等比放进 16:9 格子后左右补黑(不拉伸)
        parts.append("[%d:v]%s,scale=%d:%d:force_original_aspect_ratio=decrease,"
                     "pad=%d:%d:(ow-iw)/2:(oh-ih)/2:color=black,setsar=1[p4]"
                     % (car_idx, _crop(layout, "sidebar"), CELL_W, CELL_H, CELL_W, CELL_H))
    elif a.panel4 == "minimap":
        if "minimap" not in layout:
            sys.exit("这段 demo 的 meta 里没有 layout.minimap —— mini-map 是 P1 才加的, "
                     "旧素材请用 --panel4 sidebar")
        parts.append("[%d:v]%s,scale=%d:%d,setsar=1[p4]"
                     % (car_idx, _crop(layout, "minimap"), CELL_W, CELL_H))
    else:
        parts.append("color=c=black:s=%dx%d:d=%.3f,setsar=1[p4]"
                     % (CELL_W, CELL_H, max(car_dur, 0.1)))

    parts.append("[cam][dis]%s[p4]xstack=inputs=4:layout=0_0|w0_0|0_h0|w0_h0,"
                 "fps=%g,format=yuv420p[v]" % (p3, a.fps))

    cmd = (["ffmpeg", "-y", "-loglevel", "warning", "-stats"] + ins +
           ["-filter_complex", ";".join(parts), "-map", "[v]"])
    if car_dur:
        cmd += ["-t", "%.3f" % car_dur]              # 输出时长以车上 demo 为准
    cmd += ["-c:v", "libx264", "-crf", str(a.crf), "-preset", "medium",
            "-movflags", "+faststart", a.out]

    print("车上 demo : %s  (%.1fs, %d 帧 @ %.0ffps)"
          % (os.path.basename(a.car), car_dur, meta.get("frames", 0), meta.get("fps", 0)))
    if a.phone:
        print("手机视角  : %s" % os.path.basename(a.phone))
        print("时间对齐  : 车 demo 起点落在手机视频的 %+.3f s %s"
              % (shift, "(裁掉手机开头)" if shift >= 0 else "(手机前面补黑)"))
    else:
        print("手机视角  : 无(第3格留黑)")
    print("第4格     : %s" % a.panel4)
    print()
    if a.dry_run:
        print(" ".join(("'%s'" % c if " " in c or ";" in c else c) for c in cmd))
        return
    r = subprocess.run(cmd)
    if r.returncode != 0:
        sys.exit("ffmpeg 失败 (rc=%d)" % r.returncode)
    sz = os.path.getsize(a.out) / 1e6
    src = os.path.getsize(a.car) / 1e6
    print("\n✔ 成片 → %s  (%.1f MB, 源 %.1f MB, %.1f×)" % (a.out, sz, src, src / max(sz, 1e-9)))


if __name__ == "__main__":
    main()
