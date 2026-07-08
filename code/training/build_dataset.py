#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""build_dataset.py —— 把采集的 run 目录变成可训练的对齐表(Mac 本地跑, 纯标准库)。

每个 run 做三件事:
  1. 解码 can.log 的 18C4D2EF 底盘反馈帧 → chassis.csv (ts, speed_mps, steer_deg)
  2. frames.csv 每帧图像按时间戳配最近邻底盘反馈(容差 ±TOL_CHASSIS)
     + 最近邻 target.csv 感知量(有就配, run_004 没有)→ aligned.csv
  3. 打印标签覆盖率/分布统计, 供训练前检查

用法:  python3 build_dataset.py [dataset_root]
输出:  <run>/chassis.csv  <run>/aligned.csv
"""
import os, sys, csv, bisect, math, statistics as st

DS_DEFAULT = os.path.expanduser('~/Desktop/follow_data/follow_data_collector/dataset')
FDBK_ID     = '18C4D2EF'   # 底盘反馈帧(与命令帧同位布局, 已实测解码合理)
VEL_RES     = 0.001        # m/s per LSB
STEER_RES   = 0.01         # deg per LSB
TOL_CHASSIS = 0.05         # s, 图像↔底盘反馈最近邻容差(反馈~95Hz, 50ms 足够)
TOL_TARGET  = 0.30         # s, 图像↔target.csv 容差(YOLO ~15Hz)


def decode_fdbk(dhex):
    d = bytes.fromhex(dhex)
    v = (d[0] >> 4) | (d[1] << 4) | ((d[2] & 0xF) << 12)
    s = ((d[2] >> 4) | (d[3] << 4) | ((d[4] & 0xF) << 12)) & 0xFFFF
    if s >= 0x8000:
        s -= 0x10000
    gear = d[0] & 0xF
    return gear, v * VEL_RES, s * STEER_RES


def load_chassis(run):
    """can.log → [(ts, speed, steer)], 顺带写 chassis.csv"""
    out = []
    with open(os.path.join(run, 'can.log')) as f:
        for line in f:
            if FDBK_ID not in line:
                continue
            try:
                ts = float(line.split('(')[1].split(')')[0])
                gear, v, s = decode_fdbk(line.strip().split('#')[1])
                out.append((ts, v, s))
            except (ValueError, IndexError):
                continue
    out.sort()
    with open(os.path.join(run, 'chassis.csv'), 'w') as f:
        f.write('ts,speed_mps,steer_deg\n')
        for ts, v, s in out:
            f.write('%.6f,%.3f,%.2f\n' % (ts, v, s))
    return out


def load_target(run):
    p = os.path.join(run, 'target.csv')
    if not os.path.exists(p):
        return []
    out = []
    with open(p) as f:
        for row in csv.DictReader(f):
            try:
                ts = float(row['ts_wall'])
            except (ValueError, KeyError):
                continue
            out.append((ts, row))
    out.sort(key=lambda x: x[0])
    return out


def nearest(ts_list, t):
    """返回 (index, |dt|); ts_list 已排序"""
    i = bisect.bisect_left(ts_list, t)
    best, bdt = None, float('inf')
    for j in (i - 1, i):
        if 0 <= j < len(ts_list):
            dt = abs(ts_list[j] - t)
            if dt < bdt:
                best, bdt = j, dt
    return best, bdt


def build_run(run):
    name = os.path.basename(run)
    chassis = load_chassis(run)
    target = load_target(run)
    cts = [c[0] for c in chassis]
    tts = [t[0] for t in target]

    rows, miss = [], 0
    with open(os.path.join(run, 'frames.csv')) as f:
        for row in csv.DictReader(f):
            try:
                ts = float(row['ts_wall'])
            except (ValueError, KeyError):
                continue
            fn = row['filename']
            ci, cdt = nearest(cts, ts)
            if ci is None or cdt > TOL_CHASSIS:
                miss += 1
                continue
            _, v, s = chassis[ci]
            dist = off = conf = ''
            tvalid = 0
            if target:
                ti, tdt = nearest(tts, ts)
                if ti is not None and tdt <= TOL_TARGET:
                    tr = target[ti][1]
                    if tr.get('valid') == '1':
                        tvalid = 1
                        dist, off, conf = tr.get('dist_m', ''), tr.get('off_x', ''), tr.get('conf', '')
            rows.append((fn, ts, v, s, tvalid, dist, off, conf))

    with open(os.path.join(run, 'aligned.csv'), 'w') as f:
        f.write('filename,ts_wall,speed_mps,steer_deg,tgt_valid,dist_m,off_x,conf\n')
        for r in rows:
            f.write('%s,%.6f,%.3f,%.2f,%d,%s,%s,%s\n' % r)

    sp = [r[2] for r in rows]
    stv = [r[3] for r in rows]
    tv = sum(r[4] for r in rows)
    print('%-8s 帧 %5d (丢 %d)  速度 μ=%.2f σ=%.2f  转向 σ=%.1f  停车(v<0.05) %4.1f%%  有目标 %4.1f%%' % (
        name, len(rows), miss, st.mean(sp), st.pstdev(sp), st.pstdev(stv),
        100.0 * sum(1 for v in sp if v < 0.05) / len(sp),
        100.0 * tv / len(rows)))
    return len(rows)


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else DS_DEFAULT
    total = 0
    for d in sorted(os.listdir(root)):
        run = os.path.join(root, d)
        if not d.startswith('run_') or not os.path.isdir(run):
            continue
        if not os.path.exists(os.path.join(run, 'can.log')):
            print('%-8s 跳过(无 can.log)' % d)
            continue
        total += build_run(run)
    print('合计可训练帧: %d' % total)


if __name__ == '__main__':
    main()
