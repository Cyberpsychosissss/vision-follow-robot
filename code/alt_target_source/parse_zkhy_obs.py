#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
parse_zkhy_obs.py — 解析 ZKHY 双目相机的障碍物话题 /apollo/zkhy_obs

背景: 相机驱动 (camera_handler.cpp::processFrame, case Obstacle) 把一帧里所有
      OutputObstacles 结构体「原样二进制」打包进 std_msgs/String 发出:
        data = blockNum 个 OutputObstacles 结构体紧挨着 (无头, 见 .cc 第125行)
      所以  目标数 = len(data) / sizeof(OutputObstacles)。

⚠ 运行环境: 必须在 **apollo_dev_nvidia 容器内**, 用容器默认 python (2.7) + rospy:
    docker exec -it apollo_dev_nvidia bash
    source /apollo/scripts/apollo_base.sh
    python /apollo/follow_data/bin/parse_zkhy_obs.py
   (宿主机没装 ROS, 不能直接订阅)

⚠ 结构体布局: 下面 STRUCT_FMT 按 obstacleData.h 推算 (sizeof=128, 4字节对齐, enum=4字节,
   EXTEND_INFO_ENABLE 未定义)。**先在设备上跑 obs_probe.cpp 核对 sizeof/offsetof**,
   若 sizeof 不是 128 (例如开了 EXTEND_INFO → 132), 按探针结果改 STRUCT_FMT。

目的: 验证「相机自带行人检测 + 真实米数测距」够不够直接驱动跟随。
"""
from __future__ import print_function
import os
import json
import struct
import time

# RecognitionType 枚举 (obstacleData.h)
OBS_TYPE = {0: "INVALID", 1: "VEHICLE", 2: "PEDESTRIAN", 3: "CHILD", 4: "BICYCLE",
            5: "MOTO", 6: "TRUCK", 7: "BUS", 8: "OTHERS", 9: "ESTIMATED", 10: "CONTINUOUS"}
PERSON_TYPES = (2, 3)  # PEDESTRIAN, CHILD

# --publish 时把跟随目标写到共享文件, 供宿主机 follow_controller.py 读
RUNTIME = os.environ.get("FOLLOW_RUNTIME", "/apollo/follow_data/runtime")
TARGET_FILE = os.path.join(RUNTIME, "target.json")
# 驱动发布前对 real3DCenterX 取了负(见 camera_handler.cpp Obstacle 分支), 这里取回「右为正」。
# ⚠ 上车务必实地确认: 人站相机右侧时 lateral_m 应为正; 不对就翻这个符号。
OBS_LATERAL_SIGN = -1.0


def write_target(d):
    try:
        if not os.path.isdir(RUNTIME):
            os.makedirs(RUNTIME)
        tmp = TARGET_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(d, f)
        os.rename(tmp, TARGET_FILE)
    except Exception:
        pass

# OutputObstacles 内存布局 (little-endian)。字段顺序见 obstacleData.h。
#  2f   currentSpeed, frameRate
#  6B   trackId, trackFrameNum, stateLabel, classLabel, continuousLabel, fuzzyEstimationValid
#  2x   -> 对齐到 enum (4字节)
#  I    obstacleType (enum)
#  9f   avgDisp, avgDistanceZ, nearDistanceZ, farDistanceZ,
#       real3DLeftX, real3DRightX, real3DCenterX, real3DUpY, real3DLowY
#  8H   firstPointX/Y, secondPointX/Y, thirdPointX/Y, fourthPointX/Y  (像素bbox)
#  3f   fuzzyRelativeDistanceZ, fuzzyRelativeSpeedZ, fuzzyCollisionTimeZ
#  1B   fuzzyCollisionX
#  3x   -> 对齐
#  10f  fuzzy3DWidth, fuzzy3DCenterX/LeftX/RightX, fuzzy3DHeight, fuzzy3DUpY/LowY,
#       fuzzyRelativeSpeedCenterX/LeftX/RightX
STRUCT_FMT = "<2f6B2xI9f8H3fB3x10f"
STRUCT_SIZE = struct.calcsize(STRUCT_FMT)   # 期望 128

# 解包后各字段在 tuple 里的索引
I_TYPE, I_AVGZ, I_LEFTX, I_RIGHTX, I_CX = 8, 10, 13, 14, 15
I_P1X, I_P1Y, I_P3X, I_P3Y = 18, 19, 22, 23


def parse_obstacles(data):
    """data: bytes/str (msg.data)。返回 obstacle dict 列表。"""
    n = len(data) // STRUCT_SIZE
    out = []
    for i in range(n):
        chunk = data[i * STRUCT_SIZE:(i + 1) * STRUCT_SIZE]
        f = struct.unpack(STRUCT_FMT, chunk)
        out.append({
            "type": f[I_TYPE],
            "type_name": OBS_TYPE.get(f[I_TYPE], "?%d" % f[I_TYPE]),
            "dist_z": f[I_AVGZ],          # 纵向距离 (米)
            "center_x": f[I_CX],          # 横向位置 (米, 右为正)
            "width_m": f[I_RIGHTX] - f[I_LEFTX],
            "bbox": (f[I_P1X], f[I_P1Y], f[I_P3X], f[I_P3Y]),  # 左上x,y 右下x,y (像素)
        })
    return out


def pick_target(obstacles):
    """跟随目标 = 最近的行人 (没有行人则 None)。"""
    persons = [o for o in obstacles if o["type"] in PERSON_TYPES and 0.1 < o["dist_z"] < 50]
    if not persons:
        return None
    return min(persons, key=lambda o: o["dist_z"])


# ---------------- 离线自检 (无 ROS): 造一帧假数据走通解析 ----------------
def _selftest():
    print("=== 离线自检: STRUCT_SIZE =", STRUCT_SIZE, "(期望128) ===")
    if STRUCT_SIZE != 128:
        print("  ⚠ sizeof != 128, 先用 obs_probe.cpp 核对设备真实布局再改 STRUCT_FMT")
    # 造 2 个目标: 一个行人 dist=2.5m 右偏0.3m, 一个车 dist=8m
    vals = [0.0, 10.0, 1, 0, 0, 0, 0, 1, 2,            # ...type=2 PEDESTRIAN
            12.0, 2.5, 2.4, 2.6, 0.1, 0.5, 0.3, -0.8, 0.9,  # avgZ=2.5 centerX=0.3
            300, 200, 360, 200, 360, 480, 300, 480,
            0.0, 0.0, 0.0, 0, 0.4, 0.3, 0.1, 0.5, 1.7, -0.8, 0.9, 0.0, 0.0, 0.0]
    ped = struct.pack(STRUCT_FMT, *vals)
    vals[8] = 1; vals[10] = 8.0; vals[15] = -1.2   # VEHICLE dist=8 centerX=-1.2
    car = struct.pack(STRUCT_FMT, *vals)
    obs = parse_obstacles(ped + car)
    for o in obs:
        print("  %-10s dist=%.2fm  横向=%+.2fm  bbox=%s" %
              (o["type_name"], o["dist_z"], o["center_x"], o["bbox"]))
    t = pick_target(obs)
    print("  跟随目标:", "无" if not t else
          "行人 dist=%.2fm 横向=%+.2fm" % (t["dist_z"], t["center_x"]))
    print("自检通过 ✅" if t and abs(t["dist_z"] - 2.5) < 0.01 else "自检失败 ❌")


# ---------------- ROS 在线模式 ----------------
def _ros_main(publish=False):
    import rospy
    from std_msgs.msg import String

    state = {"last": 0.0, "frames": 0}

    def cb(msg):
        data = msg.data
        if len(data) % STRUCT_SIZE != 0:
            rospy.logwarn_throttle(2.0, "data len %d 不是 %d 的整数倍! 布局可能不对(跑 obs_probe 核对)"
                                   % (len(data), STRUCT_SIZE))
            return
        obs = parse_obstacles(data)
        state["frames"] += 1
        t = pick_target(obs)
        lateral = (OBS_LATERAL_SIGN * t["center_x"]) if t else None
        # 每帧写 target.json (供 follow_controller; 全速率, 不受打印节流影响)
        if publish:
            if t:
                write_target({"ts": time.time(), "valid": True, "source": "zkhy_obs",
                              "dist_m": float(t["dist_z"]), "lateral_m": float(lateral),
                              "n_persons": sum(1 for o in obs if o["type"] in PERSON_TYPES)})
            else:
                write_target({"ts": time.time(), "valid": False, "source": "zkhy_obs"})
        now = time.time()
        if now - state["last"] < 0.33:    # 限制到 ~3Hz 打印, 别刷屏
            return
        state["last"] = now
        peds = [o for o in obs if o["type"] in PERSON_TYPES]
        line = "obs=%d 行人=%d" % (len(obs), len(peds))
        if t:
            side = "右" if lateral > 0 else "左"
            line += " | 目标行人 dist=%.2fm 横向=%+.2fm(%s)" % (t["dist_z"], lateral, side)
        else:
            line += " | 无行人目标"
        others = [o for o in obs if o["type"] not in PERSON_TYPES][:2]
        for o in others:
            line += "  [%s %.1fm]" % (o["type_name"], o["dist_z"])
        print(line)

    rospy.init_node("zkhy_obs_parser", anonymous=True)
    rospy.Subscriber("/apollo/zkhy_obs", String, cb, queue_size=2)
    print("已订阅 /apollo/zkhy_obs%s, 等数据... (需相机驱动已启动; Ctrl-C 退出)"
          % ("  [--publish→%s]" % TARGET_FILE if publish else ""))
    rospy.spin()


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        _selftest()
    else:
        try:
            _ros_main(publish=("--publish" in sys.argv))
        except ImportError as e:
            print("没有 rospy (%s)。请在 apollo_dev_nvidia 容器内 source apollo_base.sh 后运行;" % e)
            print("或先跑离线自检:  python parse_zkhy_obs.py --selftest")
