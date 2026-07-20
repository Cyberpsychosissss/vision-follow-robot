#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lslidar_c16_udp.py — 镭神 C16 十六线激光雷达: 宿主机直收 UDP 解点云(不经 ROS)

为什么不用官方驱动
------------------
车上 `/apollo/modules/drivers/lslidar_apollo/` 有完整的 ROS 驱动, 但**这台车的
C++ roscpp 必崩** —— `/apollo/data/log/lslidar_c16.out`(2026-06-25) 记着别人跑
官方驱动的下场: `ros::TimeNotInitializedException` + rosout 进程反复 exit -6,
与 StereoCamera 那个 `advertise()` segfault 同源。所以整条 ROS 路线是死路。

好在 C16 是**纯 UDP 广播设备**, 协议公开: 绑个 socket 就能收点云, 不需要 ROS、
不需要容器、不需要 roscpp。本模块就干这件事。

⚠ 端口是交叉的(照抄 launch 里的 msop/difop 命名会搞反, 2026-07-20 实测):
      雷达 :2369  →  Xavier :2368   = 点云 MSOP  (1206 字节)
      雷达 :2368  →  Xavier :2369   = 设备信息 DIFOP(包头 A5 FF 00 5A)
  所以我们**绑 2368 收点云**。

包格式(已用 95 个真实包验证, 1140 个 block 头全部 FF EE)
--------------------------------------------------------
  1206 = 12 block × 100 + 4 时间戳 + 2 厂商码(实测 37 10)
  block = FF EE | 方位角 uint16 小端(0.01°) | 32 × (距离 uint16 小端 ×0.01m + 强度 uint8)
  32 个通道 = **2 次发射 × 16 线**: scan j → 线号 j%16, 发射轮次 j//16
  第 2 次发射的方位角 = 本 block 与下一 block 方位角的中点(线性插值)

垂直角表取自车上解码器源码 `lslidar_decoder.h::SCAN_ALTITUDE`(权威, 别自己猜),
换算成度数是经典 VLP-16 交错排布:
  线 0..15 → -15, +1, -13, +3, -11, +5, -9, +7, -7, +9, -5, +11, -3, +13, -1, +15

坐标系
------
本模块输出的是**雷达自身坐标系**: x 前 / y 左 / z 上, 方位角 0° 指向 +x。
转到车体系需要外参 —— launch 里那组 `lidar_set_z=0.6` / 绕 Z 轴 180° 是别人配的
**从未验证过**, 且 2026-07-20 试图用地面回波自标定失败(整份数据里一个近场回波都
没有, 最近点 5.55m)。**外参必须做实物实验标定**(车前已知位置放纸箱看点云落在哪),
见计划书 P1.6。在标定完成前, 下游只应使用相对几何, 别信绝对高度。

环境: 宿主机 python 3.6.9 + numpy 1.13.3(实测), 无第三方依赖。
      **不要在宿主机 pip 装东西**(会连累 web_ui 的 cv2 3.2)。
"""
from __future__ import print_function

import argparse
import math
import os
import socket
import sys
import time

import numpy as np

# ---- 常量: 全部与车上 lslidar_decoder.h 对齐 ----
PACKET_SIZE = 1206
BLOCKS_PER_PACKET = 12
SIZE_BLOCK = 100
SCANS_PER_BLOCK = 32
SCANS_PER_FIRING = 16
FIRINGS_PER_BLOCK = 2
DISTANCE_RESOLUTION = 0.01      # m
BLOCK_FLAG = 0xEEFF             # 小端读 FF EE
MSOP_PORT = 2368                # ← 点云在这个口(见文件头的端口交叉说明)

# 弧度, 索引=线号(scan j 的线号 = j % 16)
SCAN_ALTITUDE = np.array([
    -0.2617993877991494,   0.017453292519943295,
    -0.22689280275926285,  0.05235987755982989,
    -0.19198621771937624,  0.08726646259971647,
    -0.15707963267948966,  0.12217304763960307,
    -0.12217304763960307,  0.15707963267948966,
    -0.08726646259971647,  0.19198621771937624,
    -0.05235987755982989,  0.22689280275926285,
    -0.017453292519943295, 0.2617993877991494,
], dtype=np.float64)
COS_ALT = np.cos(SCAN_ALTITUDE)
SIN_ALT = np.sin(SCAN_ALTITUDE)

MIN_RANGE = 0.15                # m, 与 launch 的 min_range 一致
MAX_RANGE = 130.0               # m, 解码器 DISTANCE_MAX


def parse_packets(buf):
    """把 N 个连续的 1206 字节包批量解成点云。

    参数 buf: bytes, 长度必须是 1206 的整数倍(不是则截断到整数倍)
    返回 dict:
        xyz       (M,3) float32  雷达系坐标, x前 y左 z上
        dist      (M,)  float32  斜距 m
        intensity (M,)  uint8
        ring      (M,)  uint8    线号 0..15
        azimuth   (M,)  float32  方位角 度

    全向量化, 无 python 循环(除 12 个 block 的展开) —— 一整圈约 3 万点,
    在 Xavier 上耗时 ~2ms, 10Hz 完全够用。
    """
    n = len(buf) // PACKET_SIZE
    if n == 0:
        return None
    a = np.frombuffer(buf[:n * PACKET_SIZE], dtype=np.uint8).reshape(n, PACKET_SIZE)

    # --- 校验 block 头, 整包有一个不对就丢掉这一包(避免半包/串包污染) ---
    h0 = a[:, [b * SIZE_BLOCK for b in range(BLOCKS_PER_PACKET)]]
    h1 = a[:, [b * SIZE_BLOCK + 1 for b in range(BLOCKS_PER_PACKET)]]
    good = np.all((h0 == 0xFF) & (h1 == 0xEE), axis=1)
    if not np.any(good):
        return None
    a = a[good]
    n = a.shape[0]

    # --- 方位角: 每 block 一个(对应第 1 次发射) ---
    az = np.empty((n, BLOCKS_PER_PACKET), dtype=np.float64)
    for b in range(BLOCKS_PER_PACKET):
        o = b * SIZE_BLOCK + 2
        az[:, b] = (a[:, o].astype(np.uint16) |
                    (a[:, o + 1].astype(np.uint16) << 8)) * 0.01

    # 第 2 次发射的方位角 = 与下一 block 的中点; 最后一个 block 用前一个增量外推
    nxt = np.empty_like(az)
    nxt[:, :-1] = az[:, 1:]
    nxt[:, -1] = az[:, -1] + (az[:, -1] - az[:, -2])
    d = nxt - az
    d[d < 0] += 360.0                    # 跨 360° 回绕
    az2 = az + d * 0.5
    # (每条激光在同一次发射内还有 2.304µs 的时间差 → 10Hz 下最大 0.12° 方位偏移,
    #  20m 处折合 4cm, 远小于 0.1m 栅格分辨率, 故不做逐线补偿)

    # --- 距离 + 强度 ---
    dist = np.empty((n, BLOCKS_PER_PACKET, SCANS_PER_BLOCK), dtype=np.float64)
    inten = np.empty((n, BLOCKS_PER_PACKET, SCANS_PER_BLOCK), dtype=np.uint8)
    for b in range(BLOCKS_PER_PACKET):
        o = b * SIZE_BLOCK + 4
        blk = a[:, o:o + SCANS_PER_BLOCK * 3].reshape(n, SCANS_PER_BLOCK, 3)
        dist[:, b, :] = (blk[:, :, 0].astype(np.uint16) |
                         (blk[:, :, 1].astype(np.uint16) << 8)) * DISTANCE_RESOLUTION
        inten[:, b, :] = blk[:, :, 2]

    # --- 每个 scan 的方位角: 前 16 个用 az, 后 16 个用 az2 ---
    az_full = np.empty((n, BLOCKS_PER_PACKET, SCANS_PER_BLOCK), dtype=np.float64)
    az_full[:, :, :SCANS_PER_FIRING] = az[:, :, None]
    az_full[:, :, SCANS_PER_FIRING:] = az2[:, :, None]

    ring = np.tile(np.arange(SCANS_PER_FIRING, dtype=np.uint8), FIRINGS_PER_BLOCK)
    ring_full = np.broadcast_to(ring, (n, BLOCKS_PER_PACKET, SCANS_PER_BLOCK))

    ok = (dist > MIN_RANGE) & (dist < MAX_RANGE)
    if not np.any(ok):
        return None
    dsel = dist[ok]
    rsel = ring_full[ok]
    asel = np.radians(az_full[ok])

    r_h = dsel * COS_ALT[rsel]                    # 水平投影距离
    xyz = np.empty((dsel.size, 3), dtype=np.float32)
    xyz[:, 0] = r_h * np.cos(asel)                # x 前
    xyz[:, 1] = r_h * np.sin(asel)                # y 左
    xyz[:, 2] = dsel * SIN_ALT[rsel]              # z 上
    return {
        'xyz': xyz,
        'dist': dsel.astype(np.float32),
        'intensity': inten[ok],
        'ring': rsel,
        'azimuth': np.degrees(asel).astype(np.float32),
        'packets': n,
    }


class C16Receiver(object):
    """绑 UDP 收包, 按方位角回绕切分整圈, 每满一圈回调一次。

    典型用法:
        rx = C16Receiver()
        for cloud in rx.revolutions():
            ...   # cloud 就是 parse_packets 的返回值, 一圈约 3 万点
    """

    def __init__(self, port=MSOP_PORT, bind='0.0.0.0', timeout=2.0):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.bind((bind, port))
        self.sock.settimeout(timeout)
        self.port = port

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass

    @staticmethod
    def _first_azimuth(pkt):
        return (pkt[2] | (pkt[3] << 8)) * 0.01

    def revolutions(self, max_revs=0):
        """生成器: 每转满一圈 yield 一次点云。max_revs=0 表示无限。"""
        buf = []
        last_az = None
        revs = 0
        while True:
            try:
                pkt, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                # 雷达停转/断线 → 让调用方自己决定怎么办(下游会因为文件变旧而降级)
                yield None
                continue
            if len(pkt) != PACKET_SIZE:
                continue
            az0 = self._first_azimuth(pkt)
            if last_az is not None and az0 < last_az - 180.0:   # 方位角回绕 = 转满一圈
                cloud = parse_packets(b''.join(buf))
                buf = []
                if cloud is not None:
                    cloud['ts'] = time.time()
                    yield cloud
                    revs += 1
                    if max_revs and revs >= max_revs:
                        return
            last_az = az0
            buf.append(pkt)
            if len(buf) > 400:            # 防守: 方位角异常时别把内存吃光
                buf = buf[-200:]


# ------------------------------------------------------------------ 自检 / 离线回放
def _stats(cloud, tag=""):
    xyz, d = cloud['xyz'], cloud['dist']
    print("  %s点数 %d (来自 %d 包)" % (tag, len(d), cloud['packets']))
    print("    距离  min %.2f  中位 %.2f  max %.2f m" % (d.min(), np.median(d), d.max()))
    print("    x[前] %+.1f ~ %+.1f   y[左] %+.1f ~ %+.1f   z[上] %+.1f ~ %+.1f m"
          % (xyz[:, 0].min(), xyz[:, 0].max(), xyz[:, 1].min(), xyz[:, 1].max(),
             xyz[:, 2].min(), xyz[:, 2].max()))
    for lo, hi, name in ((-90, -30, "右"), (-30, 30, "前"), (30, 90, "左")):
        m = (np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0])) >= lo) & \
            (np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0])) < hi)
        if m.sum():
            print("    %s向扇区: %5d 点, 最近 %.2f m" % (name, m.sum(), d[m].min()))


def main():
    ap = argparse.ArgumentParser(description="镭神 C16 UDP 点云解析(不经 ROS)")
    ap.add_argument("--replay", help="离线回放: 连续 1206 字节包的裸文件")
    ap.add_argument("--port", type=int, default=MSOP_PORT)
    ap.add_argument("--revs", type=int, default=3, help="在线模式收几圈就退出(0=不停)")
    ap.add_argument("--save", help="在线模式把收到的原始包存一份(供离线复现)")
    a = ap.parse_args()

    if a.replay:
        raw = open(a.replay, "rb").read()
        print("回放 %s: %d 字节 = %d 包" % (a.replay, len(raw), len(raw) // PACKET_SIZE))
        cloud = parse_packets(raw)
        if cloud is None:
            sys.exit("解析失败: 没有合法 block 头, 文件可能不是 C16 原始包")
        _stats(cloud)
        return

    print("绑 UDP :%d 收点云 (雷达从自己的 2369 口发过来)..." % a.port)
    rx = C16Receiver(port=a.port)
    saved = []
    try:
        for i, cloud in enumerate(rx.revolutions(max_revs=a.revs)):
            if cloud is None:
                print("  ⚠ 2 秒没收到包 —— 雷达没转/没通电/网线?")
                continue
            print("第 %d 圈 @ %.3f" % (i + 1, cloud['ts']))
            _stats(cloud)
    except KeyboardInterrupt:
        pass
    finally:
        rx.close()
    if a.save and saved:
        open(a.save, "wb").write(b"".join(saved))


if __name__ == "__main__":
    main()
