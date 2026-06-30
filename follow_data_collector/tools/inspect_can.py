#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
inspect_can.py — analyse raw_can.csv to help reverse-engineer the CAN protocol.

Goal: figure out which CAN IDs likely carry speed / steering / throttle / brake,
by looking at how often each ID appears and how much its payload changes while
the car is driven.

Usage:
    python3 tools/inspect_can.py <session_dir_or_raw_can.csv>
    python3 tools/inspect_can.py <path> --id 0x18              # focus one ID
    python3 tools/inspect_can.py <path> --top 15               # show N most active
    python3 tools/inspect_can.py <path> --out can_summary.csv  # output file

Outputs a can_summary.csv with, per CAN ID:
    can_id, count, duration_sec, freq_hz, dlc,
    n_unique_payloads, changed_bytes_count, change_score,
    first_data_hex, last_data_hex
"""

import argparse
import csv
from collections import defaultdict
from pathlib import Path


def find_csv(path: Path) -> Path:
    if path.is_dir():
        cand = path / "raw_can.csv"
        if cand.exists():
            return cand
        raise FileNotFoundError(f"raw_can.csv not found in {path}")
    return path


def load_rows(csv_path: Path):
    rows = []
    with csv_path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for r in reader:
            try:
                rows.append({
                    "ts_ns": int(r["timestamp_ns"]),
                    "id": r["arbitration_id"],
                    "dlc": int(r.get("dlc", 0) or 0),
                    "hex": r.get("data_hex", "") or "",
                })
            except Exception:
                continue
    return rows


def hex_to_bytes(h: str):
    try:
        return bytes.fromhex(h)
    except Exception:
        return b""


def analyse(rows):
    by_id = defaultdict(list)
    for r in rows:
        by_id[r["id"]].append(r)

    summary = []
    for can_id, items in by_id.items():
        items.sort(key=lambda x: x["ts_ns"])
        count = len(items)
        t0, t1 = items[0]["ts_ns"], items[-1]["ts_ns"]
        duration = (t1 - t0) / 1e9 if t1 > t0 else 0.0
        freq = (count / duration) if duration > 0 else 0.0

        payloads = [it["hex"] for it in items]
        unique = len(set(payloads))

        # per-byte variability: how many byte positions ever change
        max_len = max((len(hex_to_bytes(p)) for p in payloads), default=0)
        byte_changed = [False] * max_len
        byte_minmax = [(255, 0) for _ in range(max_len)]
        prev = None
        for p in payloads:
            b = hex_to_bytes(p)
            for i in range(len(b)):
                lo, hi = byte_minmax[i]
                byte_minmax[i] = (min(lo, b[i]), max(hi, b[i]))
            if prev is not None:
                for i in range(min(len(b), len(prev))):
                    if b[i] != prev[i]:
                        byte_changed[i] = True
            prev = b
        changed_bytes = sum(byte_changed)
        # change_score: total dynamic range summed over bytes (rough "activity")
        change_score = sum((hi - lo) for (lo, hi) in byte_minmax)

        summary.append({
            "can_id": can_id,
            "count": count,
            "duration_sec": round(duration, 2),
            "freq_hz": round(freq, 2),
            "dlc": items[0]["dlc"],
            "n_unique_payloads": unique,
            "changed_bytes_count": changed_bytes,
            "change_score": change_score,
            "first_data_hex": payloads[0],
            "last_data_hex": payloads[-1],
            "_byte_minmax": byte_minmax,
        })
    # sort by change activity then frequency
    summary.sort(key=lambda d: (d["changed_bytes_count"], d["change_score"],
                                d["freq_hz"]), reverse=True)
    return summary


def print_id_detail(rows, can_id):
    items = [r for r in rows if r["id"] == can_id]
    items.sort(key=lambda x: x["ts_ns"])
    print(f"\n=== Detail for CAN ID {can_id}  ({len(items)} frames) ===")
    print(" idx  t(s)     data_hex")
    t0 = items[0]["ts_ns"] if items else 0
    for i, it in enumerate(items[:200]):
        t = (it["ts_ns"] - t0) / 1e9
        print(f" {i:4d} {t:7.3f}  {it['hex']}")
    if len(items) > 200:
        print(f"  ... ({len(items) - 200} more)")


def main():
    ap = argparse.ArgumentParser(description="Analyse raw_can.csv to find "
                                             "likely speed/steering/throttle IDs.")
    ap.add_argument("path", help="session dir or raw_can.csv")
    ap.add_argument("--id", help="show detailed payload history for one CAN ID")
    ap.add_argument("--top", type=int, default=20, help="rows to display")
    ap.add_argument("--out", default=None, help="output summary CSV path")
    args = ap.parse_args()

    csv_path = find_csv(Path(args.path).expanduser())
    rows = load_rows(csv_path)
    if not rows:
        print("[ERROR] no CAN rows loaded from", csv_path)
        return
    print(f"Loaded {len(rows)} CAN frames from {csv_path}")

    if args.id:
        print_id_detail(rows, args.id)
        return

    summary = analyse(rows)

    print(f"\nDistinct CAN IDs: {len(summary)}")
    print(f"\nTop {min(args.top, len(summary))} most ACTIVE IDs "
          f"(most likely to encode motion signals):\n")
    hdr = f"{'CAN_ID':>10} {'count':>7} {'freq_hz':>8} {'dlc':>4} " \
          f"{'uniq':>6} {'chgBytes':>9} {'score':>7}  last_data"
    print(hdr)
    print("-" * len(hdr))
    for d in summary[:args.top]:
        print(f"{d['can_id']:>10} {d['count']:>7} {d['freq_hz']:>8.1f} "
              f"{d['dlc']:>4} {d['n_unique_payloads']:>6} "
              f"{d['changed_bytes_count']:>9} {d['change_score']:>7} "
              f"  {d['last_data_hex']}")

    print("\nHint: IDs with many changing bytes AND steady high frequency are "
          "good candidates for speed / steering / throttle / brake.")
    print("Drive a known pattern (e.g. only steer, then only accelerate) and "
          "re-run with --id <ID> to watch a single signal change.")

    out = Path(args.out) if args.out else csv_path.parent / "can_summary.csv"
    fields = ["can_id", "count", "duration_sec", "freq_hz", "dlc",
              "n_unique_payloads", "changed_bytes_count", "change_score",
              "first_data_hex", "last_data_hex"]
    with out.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for d in summary:
            w.writerow({k: d[k] for k in fields})
    print(f"\nSummary written to {out}")


if __name__ == "__main__":
    main()
