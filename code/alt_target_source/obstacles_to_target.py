#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
obstacles_to_target.py — 把文件式障碍物感知 obstacles_latest.json → target.json 给 follow_controller。

为什么文件式: 这台车的 Apollo roscpp 的 C++ advertise() 会崩(BroadcastManager::registerPublisher),
所以相机数据走「SDK→写文件」而非 ROS 话题(fr07 已验证的链路)。本桥接读这些文件, 让我们的
跟随控制器(car_control + follow_controller)无需 ROS、不与相机抢连接, 直接吃真实感知。

输入(默认读 fr07 grabber 的输出, 也兼容我们自己 grabber 的同格式):
  obstacles_latest.json = {"timestamp_s":.., "count":N, "obstacles":[
     {"track_id","type"(PEDESTRIAN/CHILD/..),"distance_m","center_x_m"(右为正),"bbox":[x,y,w,h],..}, ...]}
输出: RUNTIME/target.json  (follow_controller 读)
  {"ts":墙钟, "valid":bool, "source":"obstacles_json", "dist_m":米, "lateral_m":右为正米, "n_persons":N}

⚠ center_x_m 符号: fr07 直接用 SDK 原生 real3DCenterX(右为正), 不取负。上车仍需实地确认
  「人站右边时 lateral_m 为正」, 不对就翻 LATERAL_SIGN。
"""
import os
import json
import time
import argparse

PERSON = {"PEDESTRIAN", "CHILD"}
LATERAL_SIGN = 1.0   # fr07 原生右为正; 若实测相反改 -1.0
RUNTIME = os.environ.get("FOLLOW_RUNTIME",
                         "/home/nvidia/work/AutoApollo/apollo/follow_data/runtime")
DEFAULT_OBS = "/home/nvidia/work/AutoApollo/apollo/data/fr07_tracking/obstacles_latest.json"


def pick_person(obstacles):
    persons = [o for o in obstacles
               if o.get("type") in PERSON and 0.1 < float(o.get("distance_m", 0)) < 50]
    if not persons:
        return None
    return min(persons, key=lambda o: float(o["distance_m"]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--obstacles", default=DEFAULT_OBS, help="obstacles_latest.json 路径")
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--max-age", type=float, default=1.0,
                    help="obstacles 文件 mtime 超过这么旧(秒)就当无感知")
    ap.add_argument("--once", action="store_true", help="只跑一次并打印(调试)")
    args = ap.parse_args()

    try:
        os.makedirs(RUNTIME)
    except OSError:
        pass
    target_file = os.path.join(RUNTIME, "target.json")
    period = 1.0 / args.hz

    while True:
        rec = {"ts": time.time(), "valid": False, "source": "obstacles_json"}
        try:
            age = time.time() - os.path.getmtime(args.obstacles)
            if age <= args.max_age:
                with open(args.obstacles) as f:
                    data = json.load(f)
                obs = data.get("obstacles", [])
                t = pick_person(obs)
                if t:
                    rec = {"ts": time.time(), "valid": True, "source": "obstacles_json",
                           "dist_m": float(t["distance_m"]),
                           "lateral_m": LATERAL_SIGN * float(t.get("center_x_m", 0.0)),
                           "n_persons": sum(1 for o in obs if o.get("type") in PERSON)}
        except Exception as e:
            rec["err"] = str(e)[:60]
        tmp = target_file + ".tmp"
        with open(tmp, "w") as f:
            json.dump(rec, f)
        os.replace(tmp, target_file)
        if args.once:
            print(json.dumps(rec, ensure_ascii=False))
            return
        time.sleep(period)


if __name__ == "__main__":
    main()
