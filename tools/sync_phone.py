#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sync_phone.py — 手机第三人称视频 ↔ 车钟 对时(多视角 Demo 第一步)

背景
----
车上 Demo 视频每一帧的墙钟都记在同名 CSV 的 ts 列里(且 meta.json 有起止), 但手机拍的
第三人称视角没有任何时间戳。解法是"电子场记板": 拍摄前先用手机拍车上面板的 /slate 页
3 秒 —— 那个页面显示的是**车钟**(页面加载时跟 /now 做过 NTP 式对时), 并把 unix 毫秒的
低 12 位编码成一排黑白色块。本脚本从手机视频里把它解出来, 算出

    t0_car = 手机视频 PTS=0 那一刻对应的车钟 unix 时间

之后 tools/make_multiview.py 只要把手机流平移 (T - t0_car) 就能与车上 demo 逐帧对齐。

为什么还需要人读一眼
------------------
色块只有 12 位 = 4096ms, 只能定"毫秒相位", 定不了绝对时刻。让人读一眼画面上的大号
明文时刻(精确到秒即可)就够了: 1 秒窗口 < 4096ms 模数 → 秒 + 相位 唯一确定毫秒。
不引 OCR/tesseract(Mac 上不擅自装东西), 人读一眼 5 秒钟的事, 且 100% 可靠。

用法
----
  # 第一步: 导出参考帧, 肉眼读上面的大号时刻
  python3 tools/sync_phone.py phone.mov --dump

  # 第二步: 把读到的时刻(时:分:秒, 毫秒不用管)喂回来
  python3 tools/sync_phone.py phone.mov --slate-time 13:37:51

  # 日期默认取视频文件的修改日期, 跨零点或文件被复制过时显式指定:
  python3 tools/sync_phone.py phone.mov --slate-time 13:37:51 --date 2026-07-20

输出 <video>.sync.json, 供 make_multiview.py 读。

依赖: numpy + opencv(Mac 上已有), 无需 ffmpeg 抽帧(cv2 直接读 mov/mp4)。
"""
import argparse
import json
import os
import sys
import time
import datetime as dt

import numpy as np
import cv2

CELLS = 16          # 与 web_ui.py SLATE_PAGE 的格子数一致
DATA_BITS = 12      # 格2~13
MOD = 1 << DATA_BITS  # 4096ms


# ---------------------------------------------------------------- 单帧解码
def _magenta(bgr):
    """洋红掩膜: R 高、B 高、G 明显低。阈值对拍屏偏色留足容差。"""
    b, g, r = cv2.split(bgr.astype(np.int16))
    m = ((r > 90) & (b > 90) & (g < 0.72 * np.minimum(r, b))).astype(np.uint8)
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))


def _order_quad(p):
    """四点按 左上/右上/右下/左下 排序(和为最小/最大 → 左上/右下; y-x 最小/最大 → 右上/左下)。"""
    s = p.sum(axis=1)
    d = (p[:, 1] - p[:, 0])
    return np.float32([p[np.argmin(s)], p[np.argmin(d)], p[np.argmax(s)], p[np.argmax(d)]])


def decode_slate(bgr):
    """从一帧里解出 12 位码。成功返回 dict, 失败返回 None。

    几何完全自适应, 不依赖页面的 vh/vw 尺寸 —— 只依赖两个视觉事实:
      ① 色条被洋红(255,0,255)边框圈住, 洋红在自然场景里极罕见;
      ② 格子非黑即白, 格间与内边距是中灰 → 沿列方向做 |亮度-灰| 投影,
         高的连通段就是格子, 正好应该有 16 段。

    先用 minAreaRect 把色条**摆正**再投影: 手机横屏拍摄很难端平, 直接用
    axis-aligned bbox 时实测旋转 8° 就解不出来了(列投影把相邻格子糊在一起)。
    摆正顺带也吃掉大部分透视。
    """
    if bgr is None or bgr.size == 0:
        return None
    mag = _magenta(bgr)
    if mag.sum() < 200:
        return None
    n, lab, stats, _c = cv2.connectedComponentsWithStats(mag, 8)
    if n < 2:
        return None
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    bx, by, bw, bh = (int(stats[i, k]) for k in (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP,
                                                 cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))

    # ---- ① 最小外接旋转矩形 → 透视矫正成正置矩形 ----
    cnts, _ = cv2.findContours((lab == i).astype(np.uint8),
                               cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    quad = _order_quad(cv2.boxPoints(cv2.minAreaRect(max(cnts, key=cv2.contourArea))))
    ew = int(round(max(np.linalg.norm(quad[1] - quad[0]), np.linalg.norm(quad[2] - quad[3]))))
    eh = int(round(max(np.linalg.norm(quad[3] - quad[0]), np.linalg.norm(quad[2] - quad[1]))))
    if ew < 120 or eh < 12 or ew < 4 * eh:   # 色条是细长横条, 比例不对就不是它
        return None
    M = cv2.getPerspectiveTransform(
        quad, np.float32([[0, 0], [ew - 1, 0], [ew - 1, eh - 1], [0, eh - 1]]))
    warp = cv2.warpPerspective(bgr, M, (ew, eh))

    # ---- ② 剥掉边框环, 取内部 ----
    # 内缩量按**实测边框厚度**算, 不用固定比例: 拍屏有透视/缩放时固定比例迟早剥不干净,
    # 剥不干净就会把边框或其外的黑底当成额外格子(实测切出 18 段而非 16 段)。
    wmag = _magenta(warp)
    row = wmag[eh // 2, :]
    bt = 1
    while bt < len(row) and row[bt]:
        bt += 1
    bt = max(2, min(bt, eh // 3))
    pad = int(round(bt * 1.6))
    ix0, ix1, iy0, iy1 = pad, ew - pad, pad, eh - pad
    if ix1 - ix0 < CELLS * 3 or iy1 - iy0 < 4:
        return None
    gray = cv2.cvtColor(warp, cv2.COLOR_BGR2GRAY)[iy0:iy1, ix0:ix1]
    x, y, w, h = bx, by, bw, bh          # 标注用(原图坐标系)

    # ---- ③ 列投影: 取中间 60% 行的中位数, 压掉拍屏的摩尔纹/反光 ----
    hh = gray.shape[0]
    band = gray[int(0.20 * hh):max(int(0.20 * hh) + 1, int(0.80 * hh)), :]
    col = np.median(band.astype(np.float32), axis=0)
    lo, hi = np.percentile(col, 3), np.percentile(col, 97)
    if hi - lo < 40:                        # 对比度太低(糊了/过曝) → 放弃这帧
        return None
    mid = 0.5 * (lo + hi)                   # 中灰(格间/内边距)的估计
    ext = np.abs(col - mid)
    on = ext > 0.40 * (hi - lo) / 2.0       # 高=格子, 低=格间

    # ---- ④ 切连通段, 必须正好 16 段 ----
    runs, s = [], None
    for j, v in enumerate(on):
        if v and s is None:
            s = j
        elif not v and s is not None:
            if j - s >= 2:
                runs.append((s, j))
            s = None
    if s is not None and len(on) - s >= 2:
        runs.append((s, len(on)))
    if len(runs) < CELLS:
        return None
    # 宽度过滤: 真格子等宽, 边缘残留的细条(边框/黑底/反光)宽度远小于中位宽 → 剔掉。
    # 这条比几何内缩更可靠, 是"多切出几段"这类故障的兜底防线。
    if len(runs) > CELLS:
        med = float(np.median([b_ - a_ for a_, b_ in runs]))
        runs = [rr for rr in runs if (rr[1] - rr[0]) >= 0.5 * med]
    if len(runs) != CELLS:
        return None

    # ---- ⑤ 每段取中心 60% 判黑白 ----
    bits = []
    for s0, s1 in runs:
        c0 = s0 + int(0.2 * (s1 - s0))
        c1 = max(c0 + 1, s1 - int(0.2 * (s1 - s0)))
        bits.append(1 if float(col[c0:c1].mean()) > mid else 0)

    # ---- ⑥ 同步头/尾 + 偶校验 ----
    if bits[0] != 1 or bits[1] != 0 or bits[CELLS - 1] != 1:
        return None
    data = bits[2:2 + DATA_BITS]
    par = 0
    for v in data:
        par ^= v
    if par != bits[2 + DATA_BITS]:
        return None
    code = 0
    for v in data:
        code = (code << 1) | v
    return {'code': code, 'bbox': (x, y, w, h)}


# ---------------------------------------------------------------- 扫视频
def scan(path, max_sec, verbose=True):
    """扫视频前 max_sec 秒, 返回 [(pts_sec, code)] 与一张参考帧。

    用 CAP_PROP_POS_MSEC 而不是 帧号/fps —— iPhone 常录 VFR(可变帧率), 按标称 fps
    换算时间会累积偏差, POS_MSEC 是真实呈现时间戳。
    """
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        sys.exit("打不开视频: %s" % path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
    hits, ref = [], None
    n = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        # ⚠ 必须在 read() **之后**取 POS_MSEC —— 实测(cv2 4.11/x264)此时它才等于刚读到
        # 那一帧的 PTS; 在 read() 之前取会给每帧少算一帧, 造成整段系统性偏移一帧
        # (30fps 下 33ms, 恰好是我们要控制的精度量级)。
        pts_ms = cap.get(cv2.CAP_PROP_POS_MSEC)
        if pts_ms / 1000.0 > max_sec:
            break
        d = decode_slate(frame)
        if d:
            hits.append((pts_ms / 1000.0, d['code'], n))
            if ref is None:
                ref = (pts_ms / 1000.0, frame.copy(), d)
        n += 1
    cap.release()

    # 防线: 某些后端(iPhone HEVC/mov)可能不给可用的 PTS。若时间戳不单调或整段为 0,
    # 退回 帧号/fps 换算, 并提示 —— 那时 VFR 视频会有累积偏差, 但总比全 0 强。
    bad_pts = len(hits) > 2 and (hits[-1][0] <= hits[0][0] or
                                 any(hits[k][0] < hits[k - 1][0] for k in range(1, len(hits))))
    if bad_pts and fps > 1:
        if verbose:
            print("  ⚠ 视频 PTS 不可用/不单调 → 退回 帧号÷fps 换算(VFR 视频会有累积偏差)")
        hits = [(idx / fps, code, idx) for _p, code, idx in hits]
        if ref is not None:
            ref = (hits[0][0], ref[1], ref[2])
    hits = [(p, c) for p, c, _i in hits]
    if verbose:
        print("扫描 %d 帧(前 %.0fs, 标称 %.2f fps) → 成功解码 %d 帧"
              % (n, max_sec, fps, len(hits)))
    return hits, ref, fps


# ---------------------------------------------------------------- 求解 t0
def solve_t0(hits, sec_anchor):
    """已知某绝对秒 sec_anchor(epoch 整秒, 人读明文得到) + 各帧的 12 位相位,
    求 t0_car = 手机 PTS=0 对应的车钟。

    原理: 真值落在 [S, S+1) 这 1000ms 窗口内, 而模数是 4096ms > 1000ms
          → 窗口内最多只有一个毫秒满足 floor(t*1000) & 0xFFF == code, 故唯一。
    容错: 人可能把秒读差 1(拍摄瞬间正好跳秒), 所以 S-1/S/S+1 都试, 取一致帧数最多的。
    """
    best = None
    for ds in (0, -1, 1, -2, 2):
        S = sec_anchor + ds
        for pts0, code0 in hits[:5]:                  # 用最早几帧当锚点各试一次
            for m in range(1000):
                if (S * 1000 + m) % MOD == code0:
                    t0 = (S + m / 1000.0) - pts0
                    good = sum(1 for p, c in hits
                               if abs(((int((t0 + p) * 1000) % MOD) - c + MOD // 2) % MOD - MOD // 2) <= 2)
                    if best is None or good > best[1]:
                        best = (t0, good)
                    break
    return best


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description="手机视频 ↔ 车钟 对时(电子场记板解码)")
    ap.add_argument("video")
    ap.add_argument("--slate-time", help="肉眼从参考帧读到的时刻 HH:MM:SS(毫秒不用管)")
    ap.add_argument("--date", help="日期 YYYY-MM-DD(默认取视频文件修改日期)")
    ap.add_argument("--max-sec", type=float, default=20.0, help="只扫前多少秒(默认20)")
    ap.add_argument("--dump", action="store_true", help="只导出参考帧供肉眼读时刻")
    ap.add_argument("-o", "--out", help="输出 json(默认 <video>.sync.json)")
    a = ap.parse_args()

    if not os.path.isfile(a.video):
        sys.exit("文件不存在: %s" % a.video)

    hits, ref, fps = scan(a.video, a.max_sec)
    if not hits:
        sys.exit("❌ 前 %.0fs 里一帧都没解出场记板。\n"
                 "   检查: 是否拍到了 /slate 页面? 洋红边框和色条是否完整入画、没糊没过曝?\n"
                 "   可以先 --max-sec 60 扩大搜索范围。" % a.max_sec)

    # 参考帧导出(带标注), 供肉眼读明文时刻
    ref_pts, ref_img, ref_d = ref
    ref_png = os.path.splitext(a.video)[0] + ".slate_ref.png"
    ann = ref_img.copy()
    x, y, w, h = ref_d['bbox']
    cv2.rectangle(ann, (x, y), (x + w, y + h), (0, 255, 0), 3)
    cv2.putText(ann, "code=0x%03X  pts=%.3fs" % (ref_d['code'], ref_pts),
                (x, max(30, y - 12)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    cv2.imwrite(ref_png, ann)

    if not a.slate_time:
        print("\n参考帧已导出 → %s" % ref_png)
        print("请打开它, 读出画面上那行**大号时刻**(时:分:秒即可), 然后重跑:")
        print("    python3 %s %s --slate-time HH:MM:SS" % (sys.argv[0], a.video))
        return

    # 组装人读到的绝对秒
    day = a.date or time.strftime("%Y-%m-%d", time.localtime(os.path.getmtime(a.video)))
    try:
        base = dt.datetime.strptime(day + " " + a.slate_time.strip(), "%Y-%m-%d %H:%M:%S")
    except ValueError:
        sys.exit("--slate-time 要写成 HH:MM:SS, --date 要写成 YYYY-MM-DD")
    sec_anchor = int(base.timestamp())

    got = solve_t0(hits, sec_anchor)
    if got is None:
        sys.exit("❌ 解不出: 读到的时刻与色块相位对不上。确认 --slate-time 读的是参考帧 %s "
                 "上那一行, 且 --date 正确。" % ref_png)
    t0, good = got

    # 质量评估: 用拟合斜率验证有没有丢帧/变速
    ts = np.array([p for p, _ in hits])
    pred = np.array([int((t0 + p) * 1000) % MOD for p in ts])
    obs = np.array([c for _, c in hits])
    err = ((pred - obs + MOD // 2) % MOD) - MOD // 2
    rate = good / float(len(hits))

    res = {
        'schema': 1,
        'video': os.path.abspath(a.video),
        't0_car': round(t0, 4),
        't0_car_str': dt.datetime.fromtimestamp(t0).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
        'frames_decoded': len(hits), 'frames_consistent': good,
        'consistency': round(rate, 4),
        'residual_ms_median': float(np.median(np.abs(err))),
        'residual_ms_p95': float(np.percentile(np.abs(err), 95)),
        'nominal_fps': round(fps, 3),
        'slate_time_read': a.slate_time, 'date': day,
        'ref_frame_png': ref_png,
    }
    out = a.out or (os.path.splitext(a.video)[0] + ".sync.json")
    with open(out, "w") as f:
        json.dump(res, f, indent=1, ensure_ascii=False)

    print("\n===== 对时结果 =====")
    print("  手机视频 PTS=0  ↔  车钟 %s" % res['t0_car_str'])
    print("  t0_car = %.4f" % t0)
    print("  一致帧 %d/%d (%.0f%%)  残差中位 %.0fms / p95 %.0fms"
          % (good, len(hits), rate * 100, res['residual_ms_median'], res['residual_ms_p95']))
    if rate < 0.9:
        print("  ⚠ 一致率偏低 —— 可能读错了秒, 或视频被重编码过。核对 %s" % ref_png)
    else:
        print("  ✔ 质量良好")
    print("  已写 → %s" % out)


if __name__ == "__main__":
    main()
