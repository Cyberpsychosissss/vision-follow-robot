#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ros_camera_collector.py  --  RUNS INSIDE THE APOLLO DEV DOCKER (Python 2.7 + rospy)

Taps the Apollo front camera ROS topic and saves frames to disk, so we can collect
"vision follow-the-person" training images WITHOUT the vendor Dreamview UI as the
recorder. (Dreamview / the camera driver only needs to be running so the topic
publishes; the recording itself is this independent script.)

It is READ-ONLY: it only subscribes to an image topic. It never publishes, never
touches CAN, never controls the vehicle.

Pairs with tools/can_logger.py which runs on the HOST and logs raw_can.csv into
the SAME session folder (a path visible in both, e.g. under /apollo/data). Both
stamp wall-clock time so tools/sync_dataset.py can align them afterwards.

Usage (inside docker):
    python /apollo/ros_camera_collector.py \
        --out /apollo/data/follow_dataset/run1 \
        --topic /apollo/sensor/camera/obstacle/front_6mm \
        --fps 10 --scenario straight_follow

Stop with Ctrl+C (session_meta.json is written on exit). If stdin is a TTY:
    q = stop,  m = add marker,  s = cycle scenario
"""

import os
import sys
import csv
import json
import time
import argparse

import numpy as np

try:
    import cv2
    HAVE_CV2 = True
except Exception:
    HAVE_CV2 = False

import rospy
from sensor_msgs.msg import Image

# cv_bridge is used only as a fallback if manual decoding fails.
try:
    from cv_bridge import CvBridge
    _BRIDGE = CvBridge()
except Exception:
    _BRIDGE = None

SCENARIOS = ["straight_follow", "left_turn", "right_turn",
             "person_stop", "person_lost", "multi_person", "no_person"]


def now_ns():
    return int(time.time() * 1000000000)


def imgmsg_to_bgr(msg):
    """Decode a sensor_msgs/Image into a BGR numpy array (no cv_bridge needed)."""
    enc = (msg.encoding or "").lower()
    h, w, step = msg.height, msg.width, msg.step
    buf = np.frombuffer(msg.data, dtype=np.uint8)

    if enc in ("bgr8", "rgb8"):
        row = buf.reshape(h, step)[:, :w * 3]
        arr = row.reshape(h, w, 3)
        if enc == "rgb8":
            arr = arr[:, :, ::-1]            # RGB -> BGR
        return np.ascontiguousarray(arr)

    if enc in ("bgra8", "rgba8"):
        row = buf.reshape(h, step)[:, :w * 4]
        arr = row.reshape(h, w, 4)
        code = cv2.COLOR_RGBA2BGR if enc == "rgba8" else cv2.COLOR_BGRA2BGR
        return cv2.cvtColor(arr, code)

    if enc in ("mono8", "8uc1"):
        arr = buf.reshape(h, step)[:, :w]
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)

    if enc in ("yuyv", "yuv422", "yuv422_yuy2"):
        arr = buf.reshape(h, w, 2)
        return cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_YUYV)

    # last resort: let cv_bridge try
    if _BRIDGE is not None:
        return _BRIDGE.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    raise ValueError("unsupported image encoding: %r" % msg.encoding)


class StdinKeys(object):
    """Best-effort single-key reader (q/m/s). Disabled if stdin is not a TTY."""
    def __init__(self):
        self.enabled = False
        try:
            import termios  # noqa
            self.enabled = sys.stdin.isatty()
        except Exception:
            self.enabled = False

    def getch(self):
        if not self.enabled:
            return None
        import select
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if r:
            return sys.stdin.read(1)
        return None


class Collector(object):
    def __init__(self, args):
        self.args = args
        self.out = args.out
        self.images_dir = os.path.join(self.out, "images")
        if not os.path.isdir(self.images_dir):
            os.makedirs(self.images_dir)
        self.frames_path = os.path.join(self.out, "frames.csv")
        self.markers_path = os.path.join(self.out, "markers.csv")
        new_frames = not os.path.exists(self.frames_path)
        self.frames_fh = open(self.frames_path, "a")
        self.frames_w = csv.writer(self.frames_fh)
        if new_frames:
            self.frames_w.writerow(["timestamp_ns", "frame_index", "image_path",
                                    "scenario", "marker_id", "camera_source",
                                    "width", "height"])
            self.frames_fh.flush()
        new_markers = not os.path.exists(self.markers_path)
        self.markers_fh = open(self.markers_path, "a")
        self.markers_w = csv.writer(self.markers_fh)
        if new_markers:
            self.markers_w.writerow(["timestamp_ns", "marker_id", "marker_text",
                                     "scenario"])
            self.markers_fh.flush()

        self.frame_index = 0
        self.saved = 0
        self.marker_id = 0
        self.scenario = args.scenario
        self.save_period = 1.0 / max(0.1, float(args.fps))
        self.last_save = 0.0
        self.start = time.time()
        self.last_status = 0.0
        self.keys = StdinKeys()
        self.quality = int(args.jpeg_quality)
        self.topic = args.topic
        self.stopped = False
        self.last_warn = 0.0
        self.got_any = False

    def on_image(self, msg):
        now = time.time()
        if not self.got_any:
            self.got_any = True
            rospy.loginfo("first frame received from %s (%dx%d, %s)",
                          self.topic, msg.width, msg.height, msg.encoding)
        # throttle to the target save FPS
        if now - self.last_save < self.save_period:
            return
        self.last_save = now
        try:
            bgr = imgmsg_to_bgr(msg)
        except Exception as e:
            if now - self.last_warn > 2.0:
                self.last_warn = now
                rospy.logwarn("decode failed (%s): %s", msg.encoding, e)
            return
        ts = now_ns()
        self.frame_index += 1
        fname = "%08d_%d.jpg" % (self.frame_index, ts)
        fpath = os.path.join(self.images_dir, fname)
        try:
            ok = cv2.imwrite(fpath, bgr,
                             [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
            if not ok:
                raise IOError("imwrite returned False")
        except Exception as e:
            rospy.logerr("image write failed: %s", e)
            return
        h, w = bgr.shape[:2]
        self.frames_w.writerow([ts, self.frame_index,
                                os.path.join("images", fname),
                                self.scenario, self.marker_id, self.topic, w, h])
        self.frames_fh.flush()
        self.saved += 1

    def add_marker(self):
        self.marker_id += 1
        self.markers_w.writerow([now_ns(), self.marker_id,
                                 "marker_%d" % self.marker_id, self.scenario])
        self.markers_fh.flush()
        rospy.loginfo("marker %d added (scenario=%s)", self.marker_id, self.scenario)

    def cycle_scenario(self):
        try:
            i = SCENARIOS.index(self.scenario)
            self.scenario = SCENARIOS[(i + 1) % len(SCENARIOS)]
        except ValueError:
            self.scenario = SCENARIOS[0]
        rospy.loginfo("scenario -> %s", self.scenario)

    def spin(self):
        rospy.loginfo("subscribing to %s ; saving to %s", self.topic, self.out)
        rospy.Subscriber(self.topic, Image, self.on_image, queue_size=2)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.stopped:
            # keyboard
            ch = self.keys.getch()
            if ch:
                ch = ch.lower()
                if ch == "q":
                    self.stopped = True
                elif ch == "m":
                    self.add_marker()
                elif ch == "s":
                    self.cycle_scenario()
            now = time.time()
            # periodic status + "no data" warning
            if now - self.last_status > 5.0:
                self.last_status = now
                dur = now - self.start
                rospy.loginfo("saved=%d frames | scenario=%s | dur=%.0fs | %s",
                              self.saved, self.scenario, dur,
                              "OK" if self.got_any else "NO DATA YET - is the camera module running?")
            rate.sleep()
        self.close()

    def close(self):
        try:
            self.frames_fh.flush(); self.frames_fh.close()
            self.markers_fh.flush(); self.markers_fh.close()
        except Exception:
            pass
        meta = {
            "session_name": os.path.basename(self.out.rstrip("/")),
            "start_time": self.start,
            "end_time": time.time(),
            "duration_sec": round(time.time() - self.start, 2),
            "total_frames": self.saved,
            "camera_source": self.topic,
            "fps_target": float(self.args.fps),
            "safety_mode": "ros_image_subscriber_readonly",
        }
        try:
            with open(os.path.join(self.out, "camera_meta.json"), "w") as f:
                json.dump(meta, f, indent=2)
        except Exception:
            pass
        print("\n[done] saved %d frames to %s" % (self.saved, self.out))


def main():
    ap = argparse.ArgumentParser(description="Apollo camera ROS topic -> jpg "
                                             "(runs inside the Apollo docker).")
    ap.add_argument("--out", required=True,
                    help="session output dir (use a path visible on the host "
                         "too, e.g. /apollo/data/follow_dataset/run1)")
    ap.add_argument("--topic", default="/apollo/sensor/camera/obstacle/front_6mm")
    ap.add_argument("--fps", type=float, default=10.0, help="save fps")
    ap.add_argument("--scenario", default="straight_follow")
    ap.add_argument("--jpeg-quality", type=int, default=90)
    args = ap.parse_args()

    if not HAVE_CV2:
        sys.stderr.write("[ERROR] cv2 not importable in this docker python.\n")
        sys.exit(1)

    rospy.init_node("follow_camera_collector", anonymous=True,
                    disable_signals=True)
    c = Collector(args)
    try:
        c.spin()
    except KeyboardInterrupt:
        c.stopped = True
        c.close()


if __name__ == "__main__":
    main()
