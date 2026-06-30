#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sync_dataset.py — time-align camera frames with CAN messages.

For each image frame in frames.csv, find the nearest CAN message in raw_can.csv
(within an optional time window) and write a combined synced_samples.csv. This
is the table you later turn into training pairs (image -> steering/speed).

Usage:
    python3 tools/sync_dataset.py <session_dir>
    python3 tools/sync_dataset.py <session_dir> --window-ms 100
    python3 tools/sync_dataset.py <session_dir> --dbc vehicle.dbc   # optional decode

Output: <session_dir>/synced_samples.csv with columns:
    frame_timestamp_ns, image_path, nearest_can_timestamp_ns, time_diff_ms,
    can_id, can_data_hex, scenario
    (+ decoded signal columns if a DBC is supplied and cantools is installed)

Notes:
* CAN is NOT decoded by default — we just attach the nearest raw frame.
* If --dbc is given and cantools is available, we additionally try to decode the
  nearest message and append any decoded signals (e.g. speed/steering/throttle).
"""

import argparse
import bisect
import csv
from pathlib import Path

try:
    import cantools
    HAVE_CANTOOLS = True
except Exception:
    cantools = None
    HAVE_CANTOOLS = False


def load_frames(path: Path):
    rows = []
    with path.open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            try:
                rows.append({
                    "ts": int(r["timestamp_ns"]),
                    "image_path": r["image_path"],
                    "scenario": r.get("scenario", ""),
                })
            except Exception:
                continue
    rows.sort(key=lambda x: x["ts"])
    return rows


def load_can(path: Path):
    rows = []
    with path.open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            try:
                rows.append({
                    "ts": int(r["timestamp_ns"]),
                    "id": r["arbitration_id"],
                    "hex": r.get("data_hex", "") or "",
                })
            except Exception:
                continue
    rows.sort(key=lambda x: x["ts"])
    return rows


def nearest(can_ts_list, can_rows, target_ts):
    """Binary-search nearest CAN row to target_ts. Returns (row, diff_ns)."""
    if not can_ts_list:
        return None, None
    i = bisect.bisect_left(can_ts_list, target_ts)
    best = None
    best_diff = None
    for j in (i - 1, i, i + 1):
        if 0 <= j < len(can_ts_list):
            diff = abs(can_ts_list[j] - target_ts)
            if best_diff is None or diff < best_diff:
                best_diff = diff
                best = can_rows[j]
    return best, best_diff


def try_load_dbc(dbc_path):
    if not dbc_path:
        return None
    if not HAVE_CANTOOLS:
        print("[WARN] --dbc given but cantools not installed. "
              "pip3 install cantools. Continuing without decoding.")
        return None
    try:
        db = cantools.database.load_file(dbc_path)
        print(f"[OK] loaded DBC with {len(db.messages)} messages")
        return db
    except Exception as e:
        print(f"[WARN] failed to load DBC: {e}. Continuing without decoding.")
        return None


def decode_can(db, can_id_str, data_hex):
    """Return a dict of decoded signals, or {} if it cannot be decoded."""
    if db is None or not data_hex:
        return {}
    try:
        can_id = int(can_id_str, 16) if can_id_str.startswith("0x") \
            else int(can_id_str)
        msg = db.get_message_by_frame_id(can_id)
        decoded = msg.decode(bytes.fromhex(data_hex))
        return {f"sig_{k}": v for k, v in decoded.items()}
    except Exception:
        return {}


def main():
    ap = argparse.ArgumentParser(description="Sync frames.csv with raw_can.csv")
    ap.add_argument("session", help="session directory")
    ap.add_argument("--window-ms", type=float, default=None,
                    help="only attach CAN within +/- this window (ms). "
                         "Default: no limit (always take the nearest).")
    ap.add_argument("--dbc", default=None, help="optional DBC file for decoding")
    ap.add_argument("--out", default=None, help="output csv path")
    args = ap.parse_args()

    session = Path(args.session).expanduser()
    frames_csv = session / "frames.csv"
    raw_csv = session / "raw_can.csv"
    if not frames_csv.exists():
        print(f"[ERROR] {frames_csv} not found")
        return
    frames = load_frames(frames_csv)
    can_rows = load_can(raw_csv) if raw_csv.exists() else []
    can_ts = [r["ts"] for r in can_rows]
    print(f"Loaded {len(frames)} frames and {len(can_rows)} CAN messages")

    db = try_load_dbc(args.dbc)
    window_ns = int(args.window_ms * 1e6) if args.window_ms else None

    base_fields = ["frame_timestamp_ns", "image_path", "nearest_can_timestamp_ns",
                   "time_diff_ms", "can_id", "can_data_hex", "scenario"]
    # collect decoded signal names dynamically
    out_rows = []
    extra_fields = set()
    for fr in frames:
        row, diff = nearest(can_ts, can_rows, fr["ts"])
        if row is not None and window_ns is not None and diff is not None \
                and diff > window_ns:
            row, diff = None, None  # outside the window -> no CAN attached
        rec = {
            "frame_timestamp_ns": fr["ts"],
            "image_path": fr["image_path"],
            "nearest_can_timestamp_ns": row["ts"] if row else "",
            "time_diff_ms": round(diff / 1e6, 3) if diff is not None else "",
            "can_id": row["id"] if row else "",
            "can_data_hex": row["hex"] if row else "",
            "scenario": fr["scenario"],
        }
        if row and db is not None:
            decoded = decode_can(db, row["id"], row["hex"])
            for k, v in decoded.items():
                rec[k] = v
                extra_fields.add(k)
        out_rows.append(rec)

    fields = base_fields + sorted(extra_fields)
    out = Path(args.out) if args.out else session / "synced_samples.csv"
    with out.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in out_rows:
            w.writerow({k: r.get(k, "") for k in fields})

    # quick quality stats
    diffs = [r["time_diff_ms"] for r in out_rows if r["time_diff_ms"] != ""]
    matched = len(diffs)
    print(f"\n[OK] wrote {out}  ({len(out_rows)} rows, {matched} with CAN attached)")
    if diffs:
        diffs_sorted = sorted(diffs)
        med = diffs_sorted[len(diffs_sorted) // 2]
        print(f"  time_diff_ms: min={min(diffs):.2f} median={med:.2f} "
              f"max={max(diffs):.2f}")
    if db is None:
        print("  CAN stored as raw hex. Provide --dbc later to decode "
              "speed/steering/throttle/brake.")


if __name__ == "__main__":
    main()
