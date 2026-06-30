#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
preview_dataset.py — play back a recorded session.

Shows each image in order with an overlay of timestamp / scenario / frame index,
and (if synced_samples.csv exists) the nearest CAN id and data_hex.

Usage:
    python3 tools/preview_dataset.py <session_dir>
    python3 tools/preview_dataset.py <session_dir> --fps 10
    python3 tools/preview_dataset.py <session_dir> --start 100

Keys:
    q       quit
    space   pause / resume
    .       step one frame (when paused)
    [ / ]   slower / faster playback
"""

import argparse
import csv
from datetime import datetime
from pathlib import Path

try:
    import cv2
except Exception:
    print("[ERROR] opencv-python not installed. pip3 install opencv-python")
    raise SystemExit(1)


def load_frames(session: Path):
    fcsv = session / "frames.csv"
    if not fcsv.exists():
        raise FileNotFoundError(f"frames.csv not found in {session}")
    rows = []
    with fcsv.open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            rows.append(r)
    rows.sort(key=lambda r: int(r["timestamp_ns"]))
    return rows


def load_synced(session: Path):
    scsv = session / "synced_samples.csv"
    if not scsv.exists():
        return {}
    out = {}
    with scsv.open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            out[r["image_path"]] = r
    return out


def main():
    ap = argparse.ArgumentParser(description="Play back a recorded session")
    ap.add_argument("session", help="session directory")
    ap.add_argument("--fps", type=float, default=10.0, help="playback fps")
    ap.add_argument("--start", type=int, default=0, help="start frame index")
    args = ap.parse_args()

    session = Path(args.session).expanduser()
    frames = load_frames(session)
    synced = load_synced(session)
    print(f"Loaded {len(frames)} frames. "
          f"{'synced_samples.csv found' if synced else 'no synced CAN data'}")
    print("Keys: q quit | space pause | . step | [ slower | ] faster")

    delay = max(1, int(1000 / max(0.1, args.fps)))
    paused = False
    i = max(0, args.start)
    n = len(frames)

    while 0 <= i < n:
        row = frames[i]
        img_path = session / row["image_path"]
        frame = cv2.imread(str(img_path))
        if frame is None:
            i += 1
            continue

        ts = int(row["timestamp_ns"])
        tstr = datetime.fromtimestamp(ts / 1e9).strftime("%H:%M:%S.%f")[:-3]
        lines = [
            f"frame {row['frame_index']}/{n}   {tstr}",
            f"scenario: {row.get('scenario','')}   marker: {row.get('marker_id','')}",
        ]
        srow = synced.get(row["image_path"])
        if srow:
            lines.append(f"CAN {srow.get('can_id','')}  "
                         f"{srow.get('can_data_hex','')}  "
                         f"(dt={srow.get('time_diff_ms','')} ms)")

        y = 26
        for line in lines:
            cv2.rectangle(frame, (6, y - 18), (6 + 9 * len(line), y + 6),
                          (0, 0, 0), -1)
            cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 1, cv2.LINE_AA)
            y += 26

        cv2.imshow("preview_dataset", frame)
        k = cv2.waitKey(0 if paused else delay) & 0xFF
        if k == ord("q"):
            break
        elif k == ord(" "):
            paused = not paused
        elif k == ord("."):
            paused = True
            i += 1
            continue
        elif k == ord("["):
            delay = min(2000, int(delay * 1.5) + 1)
        elif k == ord("]"):
            delay = max(1, int(delay / 1.5))

        if not paused:
            i += 1

    cv2.destroyAllWindows()
    print("Done.")


if __name__ == "__main__":
    main()
