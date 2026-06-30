#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_camera.py — quickly test which camera source works.

Usage:
    python3 scripts/check_camera.py                 # scan indices 0..5
    python3 scripts/check_camera.py 0               # test a single source
    python3 scripts/check_camera.py rtsp://192.168.1.251:554/stream
    python3 scripts/check_camera.py "v4l2src ! ... ! appsink"   # GStreamer

For each source it prints whether it opens, the resolution, and the measured
read FPS over ~2 seconds. If a window can be shown, it briefly previews the feed
(press q to close).
"""

import sys
import time

try:
    import cv2
except Exception:
    print("[ERROR] opencv-python not installed. pip3 install opencv-python")
    sys.exit(1)


def build_capture(src):
    if isinstance(src, str) and ("appsink" in src or "! " in src):
        return cv2.VideoCapture(src, cv2.CAP_GSTREAMER)
    if isinstance(src, str) and src.isdigit():
        src = int(src)
    return cv2.VideoCapture(src)


def test_source(src, show=True, seconds=2.0):
    print(f"\n--- Testing source: {src!r} ---")
    cap = build_capture(src)
    if not cap or not cap.isOpened():
        print("  [FAIL] could not open this source")
        if cap:
            cap.release()
        return False
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
    reported = cap.get(cv2.CAP_PROP_FPS) or 0
    print(f"  [OK]  opened. resolution={w}x{h}  reported_fps={reported:.1f}")

    n = 0
    t0 = time.time()
    last_frame = None
    while time.time() - t0 < seconds:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("  [WARN] a frame read failed")
            continue
        n += 1
        last_frame = frame
        if show:
            try:
                cv2.imshow("check_camera (press q)", frame)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
            except Exception:
                show = False  # headless; stop trying to display
    dt = max(1e-6, time.time() - t0)
    print(f"  measured read FPS: {n / dt:.1f}  ({n} frames in {dt:.1f}s)")
    cap.release()
    try:
        cv2.destroyAllWindows()
    except Exception:
        pass
    return last_frame is not None


def main():
    args = sys.argv[1:]
    if args:
        ok = test_source(args[0])
        sys.exit(0 if ok else 1)

    print("No source given — scanning USB indices 0..5 (no preview window)...")
    found = []
    for i in range(6):
        cap = build_capture(i)
        if cap and cap.isOpened():
            ok, _ = cap.read()
            if ok:
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
                print(f"  [OK]  index {i}: {w}x{h}")
                found.append(i)
            else:
                print(f"  [WARN] index {i}: opened but no frame")
        cap.release()
    if found:
        print(f"\nWorking USB camera indices: {found}")
        print(f"Set camera.source: {found[0]} in config.yaml, "
              f"or test the network camera with an RTSP URL.")
    else:
        print("\nNo USB cameras found. For the binocular network camera, try:")
        print("  python3 scripts/check_camera.py rtsp://192.168.1.251:554/stream")
        print("  python3 scripts/check_camera.py http://192.168.1.251:8080/?action=stream")


if __name__ == "__main__":
    main()
