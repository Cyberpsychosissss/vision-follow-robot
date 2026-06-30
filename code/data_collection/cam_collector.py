# -*- coding: utf-8 -*-
# cam_collector.py (v2) —— 在 apollo_dev_nvidia 容器内 py2 rospy 跑。
# 一个节点同时录三路, 墙钟时间戳天然对齐:
#   /apollo/sensor/camera/obstacle/front_6mm (yuyv图) -> images/*.jpg + frames.csv   (原有)
#   /apollo/zkhy_obs (障碍物结构体)                    -> obstacles.csv               (新增)
#   /apollo/zkhy/depth (32FC1深度, 需先编 StereoCameraDepth) -> depth/*.png(uint16) + depth.csv (新增)
# 用法: python cam_collector.py <run目录>
import sys, os, time, signal, struct
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String

OUTDIR = sys.argv[1] if len(sys.argv) > 1 else '/apollo/follow_data/dataset/run_000'
IMGDIR = os.path.join(OUTDIR, 'images')
DEPTHDIR = os.path.join(OUTDIR, 'depth')
for _d in (IMGDIR, DEPTHDIR):
    try:
        os.makedirs(_d)
    except OSError:
        pass

import cv2
try:
    YCODE = cv2.COLOR_YUV2BGR_YUYV
except AttributeError:
    YCODE = cv2.COLOR_YUV2BGR_YUY2

# ---- OutputObstacles 布局 (同 parse_zkhy_obs.py; 先跑 obs_probe.cpp 核对 sizeof) ----
STRUCT_FMT = "<2f6B2xI9f8H3fB3x10f"
STRUCT_SIZE = struct.calcsize(STRUCT_FMT)
OBS_TYPE = {0: "INVALID", 1: "VEHICLE", 2: "PEDESTRIAN", 3: "CHILD", 4: "BICYCLE",
            5: "MOTO", 6: "TRUCK", 7: "BUS", 8: "OTHERS", 9: "ESTIMATED", 10: "CONTINUOUS"}
I_TYPE, I_AVGZ, I_LEFTX, I_RIGHTX, I_CX = 8, 10, 13, 14, 15
I_P1X, I_P1Y, I_P3X, I_P3Y = 18, 19, 22, 23


def _open_csv(path, header):
    new = (not os.path.exists(path)) or os.path.getsize(path) == 0
    f = open(path, 'a')
    if new:
        f.write(header)
        f.flush()
    return f


fcsv = _open_csv(os.path.join(OUTDIR, 'frames.csv'), 'seq,filename,ts_wall,ts_ros,width,height\n')
ocsv = _open_csv(os.path.join(OUTDIR, 'obstacles.csv'),
                 'ts_wall,type,type_name,dist_z_m,center_x_m,width_m,bx1,by1,bx2,by2\n')
dcsv = _open_csv(os.path.join(OUTDIR, 'depth.csv'), 'seq,filename,ts_wall,width,height,unit\n')

st = {'seq': 0, 'run': True, 'lt': time.time(), 'ls': 0, 'obs_msgs': 0, 'depth_seq': 0}


def cb_img(msg):
    if not st['run']:
        return
    w = msg.width; h = msg.height
    if w <= 0 or h <= 0:
        return
    d = np.frombuffer(msg.data, dtype=np.uint8)
    if d.size < w * h * 2:
        return
    tw = time.time()
    try:
        tr = msg.header.stamp.to_sec()
    except Exception:
        tr = 0.0
    bgr = cv2.cvtColor(d[:w * h * 2].reshape(h, w, 2), YCODE)
    st['seq'] += 1; seq = st['seq']
    fn = '%06d.jpg' % seq
    cv2.imwrite(os.path.join(IMGDIR, fn), bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    fcsv.write('%d,%s,%.6f,%.6f,%d,%d\n' % (seq, fn, tw, tr, w, h))
    if seq % 15 == 0:
        fcsv.flush()
    now = time.time()
    if now - st['lt'] >= 5.0:
        fps = (seq - st['ls']) / (now - st['lt'])
        sys.stderr.write('[cam] img=%d fps=%.1f obs_msgs=%d depth=%d\n'
                         % (seq, fps, st['obs_msgs'], st['depth_seq'])); sys.stderr.flush()
        st['lt'] = now; st['ls'] = seq


def cb_obs(msg):
    if not st['run'] or STRUCT_SIZE == 0:
        return
    data = msg.data
    if len(data) % STRUCT_SIZE != 0:
        return
    tw = time.time()
    n = len(data) // STRUCT_SIZE
    for i in range(n):
        f = struct.unpack(STRUCT_FMT, data[i * STRUCT_SIZE:(i + 1) * STRUCT_SIZE])
        tp = f[I_TYPE]
        # center_x: 驱动发布前取过负, 这里再取负 = 右为正 (与 parse_zkhy_obs 一致)
        ocsv.write('%.6f,%d,%s,%.3f,%.3f,%.3f,%d,%d,%d,%d\n' % (
            tw, tp, OBS_TYPE.get(tp, '?'), f[I_AVGZ], -1.0 * f[I_CX], f[I_RIGHTX] - f[I_LEFTX],
            f[I_P1X], f[I_P1Y], f[I_P3X], f[I_P3Y]))
    st['obs_msgs'] += 1
    if st['obs_msgs'] % 15 == 0:
        ocsv.flush()


def cb_depth(msg):
    if not st['run']:
        return
    w = msg.width; h = msg.height
    if w <= 0 or h <= 0:
        return
    arr = np.frombuffer(msg.data, dtype=np.float32)
    if arr.size < w * h:
        return
    tw = time.time()
    depth = arr[:w * h].reshape(h, w)
    # 存 uint16 PNG。SDK 原生值(疑毫米); 若确认是米, 这里改成 depth*1000。无效像素为0。
    u16 = np.clip(np.nan_to_num(depth), 0, 65535).astype(np.uint16)
    st['depth_seq'] += 1; seq = st['depth_seq']
    fn = '%06d.png' % seq
    cv2.imwrite(os.path.join(DEPTHDIR, fn), u16)
    dcsv.write('%d,%s,%.6f,%d,%d,%s\n' % (seq, fn, tw, w, h, 'raw_sdk'))
    if seq % 15 == 0:
        dcsv.flush()


def stop(*a):
    st['run'] = False
    rospy.signal_shutdown('stop')


signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)
rospy.init_node('follow_cam_collector', anonymous=True, disable_signals=True)
rospy.Subscriber('/apollo/sensor/camera/obstacle/front_6mm', Image, cb_img, queue_size=10)
rospy.Subscriber('/apollo/zkhy_obs', String, cb_obs, queue_size=10)
rospy.Subscriber('/apollo/zkhy/depth', Image, cb_depth, queue_size=5)   # 无此话题则静默无数据
sys.stderr.write('[cam] collecting -> %s (img+obstacles+depth)\n' % OUTDIR); sys.stderr.flush()
while not rospy.is_shutdown():
    time.sleep(0.2)
for _f in (fcsv, ocsv, dcsv):
    try:
        _f.flush(); _f.close()
    except Exception:
        pass
sys.stderr.write('[cam] DONE img=%d obs_msgs=%d depth=%d\n'
                 % (st['seq'], st['obs_msgs'], st['depth_seq'])); sys.stderr.flush()
