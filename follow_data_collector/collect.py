#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
follow_data_collector / collect.py
==================================

Interactive, multi-threaded data collector for a "vision follow-the-person"
intelligent-connected-vehicle training platform.

PHASE 1 SAFETY MODEL
--------------------
* The vehicle is driven MANUALLY by a human operator with a remote control.
* This program ONLY records data:  camera frames + raw CAN bus traffic.
* It is RECEIVE-ONLY on the CAN bus. It never calls bus.send().
  Every CAN-write path is guarded by safety.allow_can_send (which is False).

WHAT IT PRODUCES (per session)
------------------------------
dataset/<timestamp>_<session>/
    images/<frame_index>_<timestamp_ns>.jpg
    frames.csv         frame index <-> image <-> scenario/marker
    raw_can.csv        every received CAN frame (raw, undecoded)
    markers.csv        manual event markers
    errors.csv         every health issue (level/component/message/suggestion)
    collector.log      full session log
    config_used.yaml   exact config snapshot
    session_meta.json  summary written on exit (even on Ctrl+C)

The module is intentionally self-contained (no local package imports) so it can
be copied to a Jetson and run directly:  python3 collect.py --config config.yaml
"""

import argparse
import csv
import json
import logging
import os
import queue
import shutil
import signal
import sys
import threading
import time
import traceback
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


def now_ns():
    """now_ns() backport for Python 3.6 (Jetson / Ubuntu 18.04)."""
    return int(time.time() * 1000000000)

# ----------------------------------------------------------------------------
# Soft dependencies. We degrade gracefully instead of crashing on import.
# ----------------------------------------------------------------------------
try:
    import yaml
except Exception:  # pragma: no cover - PyYAML is required, but fail with guidance
    print("[ERROR] PyYAML is not installed. Run: pip3 install pyyaml")
    sys.exit(1)

try:
    import cv2
    HAVE_CV2 = True
except Exception:
    cv2 = None  # type: ignore
    HAVE_CV2 = False

try:
    import can  # python-can
    HAVE_CAN = True
except Exception:
    can = None  # type: ignore
    HAVE_CAN = False

try:
    from rich.console import Console
    from rich.live import Live
    from rich.table import Table
    from rich.panel import Panel
    from rich.layout import Layout
    from rich.text import Text
    HAVE_RICH = True
except Exception:
    HAVE_RICH = False


# ============================================================================
# Configuration
# ============================================================================

DEFAULT_CONFIG: Dict[str, Any] = {
    "camera": {
        "source": 0, "fps": 10, "width": 1280, "height": 720,
        "jpeg_quality": 90, "show_preview": True,
        "reconnect_attempts": 5, "max_consecutive_read_failures": 30,
    },
    "can": {
        "enabled": True, "channel": "can0", "bitrate": 500000,
        "interface": "socketcan", "listen_only": True, "no_message_warn_sec": 5,
    },
    "dataset": {
        "root_dir": "./dataset", "session_name": "follow_001",
        "default_scenario": "straight_follow",
        "scenarios": ["straight_follow", "left_turn", "right_turn",
                      "person_stop", "person_lost", "multi_person", "no_person"],
    },
    "logging": {"level": "INFO"},
    "runtime": {
        "countdown_sec": 3, "status_print_sec": 5,
        "camera_queue_max": 8, "writer_queue_max": 256, "disk_low_gb": 5,
    },
    "safety": {"allow_can_send": False, "require_manual_remote": True},
    "web": {"enabled": False, "host": "0.0.0.0", "port": 8080},
}


def deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge override into base (returns a new dict)."""
    out = dict(base)
    for k, v in (override or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def load_config(path: Optional[Path]) -> Tuple[dict, List[str]]:
    """Load config.yaml merged onto defaults. Returns (config, warnings)."""
    warnings: List[str] = []
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))  # deep copy
    if path is None:
        return cfg, warnings
    p = Path(path)
    if not p.exists():
        warnings.append(f"config file not found: {p} (using built-in defaults)")
        return cfg, warnings
    try:
        with p.open("r", encoding="utf-8") as f:
            user_cfg = yaml.safe_load(f) or {}
        cfg = deep_merge(cfg, user_cfg)
    except Exception as e:
        warnings.append(f"failed to parse {p}: {e} (using built-in defaults)")
    return cfg, warnings


def write_default_config(path: Path) -> None:
    """Write the default config to disk (used by the launcher / first run)."""
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(DEFAULT_CONFIG, f, sort_keys=False, allow_unicode=True)


# ============================================================================
# Health monitoring / error reporting
# ============================================================================

LEVELS = {"INFO": 10, "WARN": 20, "ERROR": 30, "CRITICAL": 40}

# Component-specific default suggestions (used when caller does not pass one).
DEFAULT_SUGGESTIONS = {
    "CAMERA": "Try camera source 0, 1, or check the RTSP/HTTP URL. "
              "Run scripts/check_camera.py to test sources.",
    "CAN": "Run 'ip -details link show can0'. Confirm the USB-CAN driver is "
           "loaded and bring it up: sudo ip link set can0 up type can bitrate 500000",
    "STORAGE": "Check output directory permission and free disk space. "
               "Change dataset.root_dir in config.yaml if needed.",
    "CONFIG": "Check config.yaml syntax (YAML). A default config can be regenerated.",
    "SYSTEM": "Check the session log for details.",
}


class Issue(object):
    def __init__(self, timestamp_ns, level, component, message,
                 suggestion="", exception_type="", exception_detail=""):
        self.timestamp_ns = timestamp_ns
        self.level = level
        self.component = component
        self.message = message
        self.suggestion = suggestion
        self.exception_type = exception_type
        self.exception_detail = exception_detail

    def as_row(self):
        return [str(self.timestamp_ns), self.level, self.component, self.message,
                self.suggestion, self.exception_type, self.exception_detail]


class HealthMonitor:
    """
    Central collector for problems and warnings.

    Every issue is:
      * appended to an in-memory ring buffer (shown on the live dashboard),
      * written to the Python logger (console + collector.log),
      * appended to errors.csv in the session folder.

    De-duplication: identical (component, message) issues are rate-limited so a
    repeating fault (e.g. "CAN no message") does not flood the logs.
    """

    def __init__(self, logger: logging.Logger, errors_csv: Optional[Path] = None,
                 ring_size: int = 12, dedup_window_sec: float = 3.0):
        self.logger = logger
        self.errors_csv = errors_csv
        self.ring: List[Issue] = []
        self.ring_size = ring_size
        self.dedup_window_sec = dedup_window_sec
        self._lock = threading.Lock()
        self._last_seen: Dict[Tuple[str, str], float] = {}
        self.counts = {"INFO": 0, "WARN": 0, "ERROR": 0, "CRITICAL": 0}
        self._csv_fh = None
        self._csv_writer = None
        if errors_csv is not None:
            self._open_csv(errors_csv)

    def _open_csv(self, path: Path) -> None:
        try:
            new_file = not path.exists()
            self._csv_fh = path.open("a", newline="", encoding="utf-8")
            self._csv_writer = csv.writer(self._csv_fh)
            if new_file:
                self._csv_writer.writerow(
                    ["timestamp_ns", "level", "component", "message",
                     "suggestion", "exception_type", "exception_detail"])
                self._csv_fh.flush()
        except Exception as e:  # pragma: no cover
            self.logger.error("Could not open errors.csv: %s", e)

    def attach_csv(self, path: Path) -> None:
        """Late-bind the errors.csv once the session folder exists."""
        with self._lock:
            self.errors_csv = path
            self._open_csv(path)

    def report(self, level: str, component: str, message: str,
               suggestion: str = "", exc: Optional[BaseException] = None) -> None:
        level = level.upper()
        component = component.upper()
        if not suggestion:
            suggestion = DEFAULT_SUGGESTIONS.get(component, "")
        exc_type, exc_detail = "", ""
        if exc is not None:
            exc_type = type(exc).__name__
            exc_detail = "".join(
                traceback.format_exception_only(type(exc), exc)).strip()

        now = time.time()
        key = (component, message)
        with self._lock:
            last = self._last_seen.get(key, 0.0)
            self._last_seen[key] = now
            self.counts[level] = self.counts.get(level, 0) + 1
            issue = Issue(now_ns(), level, component, message,
                          suggestion, exc_type, exc_detail)
            self.ring.append(issue)
            if len(self.ring) > self.ring_size:
                self.ring = self.ring[-self.ring_size:]

            # rate-limit duplicate console/log spam, but ALWAYS persist to csv
            suppressed = (now - last) < self.dedup_window_sec

            if self._csv_writer is not None:
                try:
                    self._csv_writer.writerow(issue.as_row())
                    self._csv_fh.flush()
                except Exception:
                    pass

        if not suppressed:
            log_fn = {"INFO": self.logger.info, "WARN": self.logger.warning,
                      "ERROR": self.logger.error,
                      "CRITICAL": self.logger.critical}.get(level, self.logger.info)
            tail = f" | fix: {suggestion}" if suggestion else ""
            if exc_detail:
                tail += f" | {exc_type}: {exc_detail}"
            log_fn("[%s] %s%s", component, message, tail)

    # convenience wrappers
    def info(self, c, m, s="", exc=None): self.report("INFO", c, m, s, exc)
    def warn(self, c, m, s="", exc=None): self.report("WARN", c, m, s, exc)
    def error(self, c, m, s="", exc=None): self.report("ERROR", c, m, s, exc)
    def critical(self, c, m, s="", exc=None): self.report("CRITICAL", c, m, s, exc)

    def recent(self) -> List[Issue]:
        with self._lock:
            return list(self.ring)

    def close(self) -> None:
        with self._lock:
            if self._csv_fh is not None:
                try:
                    self._csv_fh.flush()
                    self._csv_fh.close()
                except Exception:
                    pass
                self._csv_fh = None


# ============================================================================
# Logging setup
# ============================================================================

def setup_logging(level_name: str, logs_dir: Path,
                  session_log: Optional[Path] = None) -> logging.Logger:
    """Configure a logger that writes to console + a startup/session log file."""
    logs_dir.mkdir(parents=True, exist_ok=True)
    level = logging.DEBUG if str(level_name).upper() == "DEBUG" else logging.INFO

    logger = logging.getLogger("fdc")
    logger.setLevel(logging.DEBUG)
    logger.handlers.clear()
    logger.propagate = False

    fmt = logging.Formatter("%(asctime)s %(levelname)-8s %(message)s",
                            "%Y-%m-%d %H:%M:%S")

    # Console handler (kept quiet at WARNING+ when the rich dashboard is active;
    # the dashboard itself shows INFO-level status). Default INFO/DEBUG.
    ch = logging.StreamHandler(sys.stdout)
    ch.setLevel(level)
    ch.setFormatter(fmt)
    logger.addHandler(ch)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    startup_log = logs_dir / f"session_{ts}.log"
    fh = logging.FileHandler(startup_log, encoding="utf-8")
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(fmt)
    logger.addHandler(fh)
    logger.info("Logging to %s", startup_log)
    logger._fdc_logfile = str(startup_log)  # type: ignore[attr-defined]
    return logger


def add_session_file_handler(logger: logging.Logger, session_log: Path) -> None:
    fmt = logging.Formatter("%(asctime)s %(levelname)-8s %(message)s",
                            "%Y-%m-%d %H:%M:%S")
    fh = logging.FileHandler(session_log, encoding="utf-8")
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(fmt)
    logger.addHandler(fh)


# ============================================================================
# Shared statistics (read by the dashboard, written by the worker threads)
# ============================================================================

class Stats(object):
    def __init__(self):
        self.lock = threading.Lock()
        # session
        self.session_name = ""
        self.session_dir = ""
        self.scenario = ""
        self.marker_id = 0
        self.state = "idle"          # recording / paused / stopped / idle
        self.start_time = 0.0
        self.pause_accum = 0.0       # total paused seconds
        # camera
        self.camera_source = ""
        self.camera_opened = False
        self.frame_w = 0
        self.frame_h = 0
        self.fps_target = 0.0
        self.actual_capture_fps = 0.0
        self.actual_save_fps = 0.0
        self.frames_captured = 0
        self.frames_saved = 0
        self.dropped_frames = 0
        self.last_frame_ts_ns = 0
        self.last_saved_path = ""
        self.consecutive_read_failures = 0
        # CAN
        self.can_enabled = False
        self.can_channel = ""
        self.can_opened = False
        self.can_messages = 0
        self.can_rate = 0.0
        self.last_can_id = ""
        self.last_can_data_hex = ""
        self.last_can_ts_ns = 0
        self.can_error_count = 0
        # storage
        self.dataset_dir = ""
        self.disk_free_gb = 0.0
        self.session_size_mb = 0.0
        self.frames_csv_mb = 0.0
        self.raw_can_csv_mb = 0.0
        self.write_errors = 0
        # queues
        self.camera_q = 0
        self.writer_q = 0
        self.camera_q_max = 0
        self.writer_q_max = 0
        self.queue_overflow = 0

    def snapshot(self):
        with self.lock:
            return dict(self.__dict__)


# ============================================================================
# Camera capture thread
# ============================================================================

def build_capture(source: Any, width: int, height: int):
    """Open a cv2.VideoCapture from int index, URL, or GStreamer pipeline."""
    src = source
    cap = None
    # GStreamer pipeline heuristic: contains "appsink" or "! "
    if isinstance(src, str) and ("appsink" in src or "! " in src):
        cap = cv2.VideoCapture(src, cv2.CAP_GSTREAMER)
    else:
        # int index, or rtsp/http URL
        if isinstance(src, str) and src.isdigit():
            src = int(src)
        cap = cv2.VideoCapture(src)
        if isinstance(src, int):
            try:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            except Exception:
                pass
    return cap


class CameraThread(threading.Thread):
    """Continuously grabs frames and pushes (ts_ns, frame) to a bounded queue."""

    def __init__(self, cfg: dict, stats: Stats, health: HealthMonitor,
                 out_q: "queue.Queue", stop_evt: threading.Event,
                 pause_evt: threading.Event):
        super().__init__(name="CameraThread", daemon=True)
        self.cfg = cfg
        self.stats = stats
        self.health = health
        self.out_q = out_q
        self.stop_evt = stop_evt
        self.pause_evt = pause_evt
        self.cap = None
        self.source = cfg["camera"]["source"]

    def _open(self) -> bool:
        attempts = int(self.cfg["camera"].get("reconnect_attempts", 5))
        for i in range(max(1, attempts)):
            if self.stop_evt.is_set():
                return False
            self.cap = build_capture(self.source,
                                     int(self.cfg["camera"]["width"]),
                                     int(self.cfg["camera"]["height"]))
            if self.cap is not None and self.cap.isOpened():
                w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
                h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
                with self.stats.lock:
                    self.stats.camera_opened = True
                    self.stats.frame_w = w
                    self.stats.frame_h = h
                    self.stats.camera_source = str(self.source)
                self.health.info("CAMERA",
                                 f"camera opened ({w}x{h}) source={self.source}")
                return True
            self.health.warn("CAMERA",
                             f"open attempt {i+1}/{attempts} failed for "
                             f"source={self.source}")
            time.sleep(1.0)
        self.health.error("CAMERA",
                          f"could not open camera source={self.source}")
        with self.stats.lock:
            self.stats.camera_opened = False
        return False

    def run(self) -> None:
        if not self._open():
            return
        max_fail = int(self.cfg["camera"].get("max_consecutive_read_failures", 30))
        # rolling capture-fps measurement
        fps_window: List[float] = []
        save_period = 1.0 / max(0.1, float(self.cfg["camera"]["fps"]))
        last_save = 0.0

        while not self.stop_evt.is_set():
            if self.pause_evt.is_set():
                time.sleep(0.05)
                continue
            try:
                ok, frame = self.cap.read()
            except Exception as e:
                ok, frame = False, None
                self.health.error("CAMERA", "exception during read", exc=e)

            now = time.time()
            if not ok or frame is None:
                with self.stats.lock:
                    self.stats.consecutive_read_failures += 1
                    fails = self.stats.consecutive_read_failures
                    self.stats.camera_opened = False
                self.health.warn("CAMERA",
                                 f"frame read failed ({fails}/{max_fail})")
                if fails >= max_fail:
                    self.health.error("CAMERA",
                                      "too many consecutive read failures; "
                                      "attempting reconnect")
                    try:
                        if self.cap:
                            self.cap.release()
                    except Exception:
                        pass
                    if not self._open():
                        self.health.critical("CAMERA",
                                             "camera disconnected and reconnect failed")
                        return
                    with self.stats.lock:
                        self.stats.consecutive_read_failures = 0
                else:
                    time.sleep(0.05)
                continue

            ts_ns = now_ns()
            with self.stats.lock:
                self.stats.consecutive_read_failures = 0
                self.stats.frames_captured += 1
                self.stats.last_frame_ts_ns = ts_ns
                self.stats.camera_opened = True

            # capture-fps estimate
            fps_window.append(now)
            fps_window = [t for t in fps_window if now - t <= 1.0]
            with self.stats.lock:
                self.stats.actual_capture_fps = float(len(fps_window))

            # downsample to the SAVE fps
            if now - last_save >= save_period:
                last_save = now
                try:
                    self.out_q.put_nowait((ts_ns, frame))
                except queue.Full:
                    with self.stats.lock:
                        self.stats.dropped_frames += 1
                        self.stats.queue_overflow += 1
                    self.health.warn(
                        "STORAGE",
                        "image writer queue full; dropping frame",
                        "Image writer queue is growing. Disk may be too slow "
                        "or FPS is too high. Lower camera.fps or use faster storage.")
        try:
            if self.cap:
                self.cap.release()
        except Exception:
            pass


# ============================================================================
# Image writer thread (decouples disk I/O from capture)
# ============================================================================

class ImageWriterThread(threading.Thread):
    def __init__(self, cfg: dict, stats: Stats, health: HealthMonitor,
                 in_q: "queue.Queue", images_dir: Path,
                 frames_writer: "CsvWriter", stop_evt: threading.Event,
                 get_scenario, get_marker):
        super().__init__(name="ImageWriterThread", daemon=True)
        self.cfg = cfg
        self.stats = stats
        self.health = health
        self.in_q = in_q
        self.images_dir = images_dir
        self.frames_writer = frames_writer
        self.stop_evt = stop_evt
        self.get_scenario = get_scenario
        self.get_marker = get_marker
        self.frame_index = 0
        self._save_times: List[float] = []

    def run(self) -> None:
        quality = int(self.cfg["camera"].get("jpeg_quality", 90))
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        while not (self.stop_evt.is_set() and self.in_q.empty()):
            try:
                ts_ns, frame = self.in_q.get(timeout=0.2)
            except queue.Empty:
                continue
            self.frame_index += 1
            fname = f"{self.frame_index:08d}_{ts_ns}.jpg"
            fpath = self.images_dir / fname
            try:
                ok = cv2.imwrite(str(fpath), frame, encode_params)
                if not ok:
                    raise IOError("cv2.imwrite returned False")
                h, w = frame.shape[:2]
                scenario = self.get_scenario()
                marker = self.get_marker()
                self.frames_writer.write_row([
                    ts_ns, self.frame_index, str(Path("images") / fname),
                    scenario, marker, str(self.cfg["camera"]["source"]), w, h])
                now = time.time()
                self._save_times.append(now)
                self._save_times = [t for t in self._save_times if now - t <= 1.0]
                with self.stats.lock:
                    self.stats.frames_saved += 1
                    self.stats.last_saved_path = str(Path("images") / fname)
                    self.stats.actual_save_fps = float(len(self._save_times))
            except Exception as e:
                with self.stats.lock:
                    self.stats.write_errors += 1
                self.health.error(
                    "STORAGE", "image write failed",
                    "Check output directory permission and disk space.", exc=e)
            finally:
                self.in_q.task_done()


# ============================================================================
# CAN listener thread (RECEIVE ONLY)
# ============================================================================

class CanThread(threading.Thread):
    """
    SocketCAN receive-only listener.

    SAFETY: This class has NO send path. It never imports a sender, never calls
    bus.send(), and asserts safety.allow_can_send is False before starting.
    """

    def __init__(self, cfg: dict, stats: Stats, health: HealthMonitor,
                 raw_writer: "CsvWriter", stop_evt: threading.Event):
        super().__init__(name="CanThread", daemon=True)
        self.cfg = cfg
        self.stats = stats
        self.health = health
        self.raw_writer = raw_writer
        self.stop_evt = stop_evt
        self.bus = None

    def open_bus(self) -> bool:
        if not HAVE_CAN:
            self.health.warn("CAN", "python-can not installed; CAN disabled",
                             "pip3 install python-can")
            return False
        # Hard safety gate.
        if self.cfg["safety"].get("allow_can_send", False):
            self.health.critical(
                "CAN", "safety.allow_can_send is True; refusing to start",
                "Phase 1 is listen-only. Set safety.allow_can_send: false.")
            return False
        channel = self.cfg["can"]["channel"]
        bustype = self.cfg["can"].get("interface", "socketcan")
        try:
            # receive_own_messages=False; we never transmit. This is a pure listener.
            # 'bustype' is accepted by BOTH python-can 3.x (required on Python 3.6)
            # and 4.x (where it is a deprecated alias of 'interface'). Using it keeps
            # us compatible with python-can==3.3.4 on the Jetson.
            try:
                self.bus = can.interface.Bus(
                    channel=channel, bustype=bustype,
                    receive_own_messages=False)
            except TypeError:
                # very new python-can that dropped the 'bustype' alias
                self.bus = can.interface.Bus(
                    channel=channel, interface=bustype,
                    receive_own_messages=False)
            with self.stats.lock:
                self.stats.can_opened = True
                self.stats.can_channel = channel
            self.health.info("CAN", f"listening on {channel} (receive-only)")
            return True
        except Exception as e:
            with self.stats.lock:
                self.stats.can_opened = False
            self.health.error(
                "CAN", f"could not open CAN channel '{channel}'",
                "Run 'ip -details link show can0'. Bring it up: "
                "sudo ip link set can0 up type can bitrate "
                f"{self.cfg['can'].get('bitrate', 500000)}", exc=e)
            return False

    def run(self) -> None:
        if not self.open_bus():
            return
        warn_sec = float(self.cfg["can"].get("no_message_warn_sec", 5))
        last_msg_time = time.time()
        rate_window: List[float] = []
        while not self.stop_evt.is_set():
            try:
                msg = self.bus.recv(timeout=0.5)
            except Exception as e:
                with self.stats.lock:
                    self.stats.can_error_count += 1
                self.health.error("CAN", "error while receiving", exc=e)
                time.sleep(0.2)
                continue

            now = time.time()
            if msg is None:
                if now - last_msg_time > warn_sec:
                    self.health.warn(
                        "CAN",
                        f"no CAN message for {int(now - last_msg_time)}s",
                        "Confirm the vehicle is powered and CAN is wired. "
                        "Check 'candump can0'.")
                continue

            last_msg_time = now
            ts_ns = now_ns()
            data_hex = msg.data.hex()
            try:
                self.raw_writer.write_row([
                    ts_ns,
                    f"{msg.timestamp:.6f}",
                    hex(msg.arbitration_id),
                    int(bool(msg.is_extended_id)),
                    msg.dlc,
                    data_hex,
                ])
            except Exception as e:
                with self.stats.lock:
                    self.stats.write_errors += 1
                self.health.error("STORAGE", "raw_can.csv write failed", exc=e)

            rate_window.append(now)
            rate_window = [t for t in rate_window if now - t <= 1.0]
            with self.stats.lock:
                self.stats.can_messages += 1
                self.stats.last_can_id = hex(msg.arbitration_id)
                self.stats.last_can_data_hex = data_hex
                self.stats.last_can_ts_ns = ts_ns
                self.stats.can_rate = float(len(rate_window))

        try:
            if self.bus is not None:
                self.bus.shutdown()
        except Exception:
            pass


# ============================================================================
# CSV writer (thread-safe, flushed every row)
# ============================================================================

class CsvWriter:
    def __init__(self, path: Path, header: List[str]):
        self.path = path
        self._lock = threading.Lock()
        new_file = not path.exists()
        self._fh = path.open("a", newline="", encoding="utf-8")
        self._w = csv.writer(self._fh)
        if new_file:
            self._w.writerow(header)
            self._fh.flush()

    def write_row(self, row: List[Any]) -> None:
        with self._lock:
            self._w.writerow(row)
            self._fh.flush()  # flush every row: survive sudden power loss

    def size_mb(self) -> float:
        try:
            return self.path.stat().st_size / (1024 * 1024)
        except Exception:
            return 0.0

    def close(self) -> None:
        with self._lock:
            try:
                self._fh.flush()
                self._fh.close()
            except Exception:
                pass


# ============================================================================
# Keyboard helpers
# ============================================================================

class TerminalKeyReader(threading.Thread):
    """
    Non-blocking single-key reader from stdin (used when no preview window is
    shown). Uses termios cbreak mode on POSIX. Silently disabled if stdin is not
    a TTY (e.g. piped) or on platforms without termios.
    """

    def __init__(self, callback, stop_evt: threading.Event):
        super().__init__(name="KeyReader", daemon=True)
        self.callback = callback
        self.stop_evt = stop_evt
        self.enabled = False
        try:
            import termios, tty  # noqa: F401
            self.enabled = sys.stdin.isatty()
        except Exception:
            self.enabled = False

    def run(self) -> None:
        if not self.enabled:
            return
        import termios, tty, select
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while not self.stop_evt.is_set():
                r, _, _ = select.select([sys.stdin], [], [], 0.2)
                if r:
                    ch = sys.stdin.read(1)
                    if ch:
                        self.callback(ch.lower())
        except Exception:
            pass
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            except Exception:
                pass


# ============================================================================
# Dashboard (rich, with plain-print fallback)
# ============================================================================

class Dashboard:
    """Renders the live status panel. Uses rich if available."""

    def __init__(self, stats: Stats, health: HealthMonitor, cfg: dict,
                 detailed_getter):
        self.stats = stats
        self.health = health
        self.cfg = cfg
        self.detailed_getter = detailed_getter
        self.console = Console() if HAVE_RICH else None
        self.live: Optional["Live"] = None

    # -- rich rendering --------------------------------------------------
    def _render_rich(self):
        s = self.stats.snapshot()
        detailed = self.detailed_getter()

        def kv_table(title, rows, style="cyan"):
            t = Table(title=title, expand=True, show_header=False,
                      title_style=f"bold {style}", border_style=style)
            t.add_column("k", style="dim", no_wrap=True)
            t.add_column("v", overflow="fold")
            for k, v in rows:
                t.add_row(str(k), str(v))
            return t

        dur = self._duration(s)
        state_color = {"recording": "bold green", "paused": "bold yellow",
                       "stopped": "bold red", "idle": "dim"}.get(s["state"], "white")

        session_rows = [
            ("session", s["session_name"]),
            ("output", _short(s["session_dir"])),
            ("scenario", f"[bold magenta]{s['scenario']}[/]"),
            ("marker id", s["marker_id"]),
            ("state", f"[{state_color}]{s['state'].upper()}[/]"),
            ("duration", f"{dur:0.1f}s"),
        ]
        cam_rows = [
            ("source", s["camera_source"]),
            ("opened", _yn(s["camera_opened"])),
            ("resolution", f"{s['frame_w']}x{s['frame_h']}"),
            ("target fps", f"{s['fps_target']:0.1f}"),
            ("capture fps", f"{s['actual_capture_fps']:0.1f}"),
            ("save fps", f"{s['actual_save_fps']:0.1f}"),
            ("captured", s["frames_captured"]),
            ("saved", s["frames_saved"]),
            ("dropped", s["dropped_frames"]),
            ("last frame ts", _ts(s["last_frame_ts_ns"])),
        ]
        if detailed:
            cam_rows.append(("last image", _short(s["last_saved_path"], 40)))
        can_rows = [
            ("enabled", _yn(s["can_enabled"])),
            ("channel", s["can_channel"]),
            ("opened", _yn(s["can_opened"])),
            ("messages", s["can_messages"]),
            ("msg/s", f"{s['can_rate']:0.1f}"),
            ("last id", s["last_can_id"]),
            ("last data", s["last_can_data_hex"]),
            ("since last", self._since(s["last_can_ts_ns"])),
            ("errors", s["can_error_count"]),
        ]
        store_rows = [
            ("dataset dir", _short(s["dataset_dir"], 40)),
            ("disk free", f"{s['disk_free_gb']:0.1f} GB"),
            ("session size", f"{s['session_size_mb']:0.1f} MB"),
            ("frames.csv", f"{s['frames_csv_mb']:0.2f} MB"),
            ("raw_can.csv", f"{s['raw_can_csv_mb']:0.2f} MB"),
            ("write errors", s["write_errors"]),
        ]
        if detailed:
            store_rows += [
                ("camera q", f"{s['camera_q']}/{s['camera_q_max']}"),
                ("writer q", f"{s['writer_q']}/{s['writer_q_max']}"),
                ("q overflow", s["queue_overflow"]),
            ]

        # issues panel
        issues = self.health.recent()
        lines = []
        for it in issues[-8:]:
            color = {"INFO": "dim", "WARN": "yellow", "ERROR": "red",
                     "CRITICAL": "bold red"}.get(it.level, "white")
            sug = f"  -> {it.suggestion}" if it.suggestion else ""
            lines.append(f"[{color}]{it.level:8s}[/] [{it.component}] "
                         f"{it.message}{sug}")
        issues_text = "\n".join(lines) if lines else "[green]no issues[/]"

        keys = ("[bold]q[/] stop   [bold]p[/] pause/resume   [bold]m[/] marker   "
                "[bold]s[/] scenario   [bold]h[/] help   [bold]d[/] debug view")

        layout = Layout()
        layout.split_column(
            Layout(Panel(Text.from_markup(
                f"  follow_data_collector  |  SAFETY: listen-only CAN, no vehicle "
                f"control  |  {keys}", justify="left"),
                style="bold white on grey15"), size=3, name="head"),
            Layout(name="body", ratio=1),
            Layout(Panel(Text.from_markup(issues_text),
                         title="Issues & Warnings", border_style="yellow"),
                   size=11, name="issues"),
        )
        layout["body"].split_row(
            Layout(kv_table("Session", session_rows, "white")),
            Layout(kv_table("Camera", cam_rows, "cyan")),
            Layout(kv_table("CAN", can_rows, "green")),
            Layout(kv_table("Storage", store_rows, "blue")),
        )
        return layout

    # -- plain fallback --------------------------------------------------
    def _render_plain(self) -> str:
        s = self.stats.snapshot()
        dur = self._duration(s)
        lines = [
            "=" * 70,
            f" {s['session_name']}  state={s['state'].upper()}  "
            f"scenario={s['scenario']}  dur={dur:0.1f}s",
            f" CAM  open={_yn(s['camera_opened'])} {s['frame_w']}x{s['frame_h']} "
            f"cap={s['actual_capture_fps']:0.1f} save={s['actual_save_fps']:0.1f} "
            f"saved={s['frames_saved']} dropped={s['dropped_frames']}",
            f" CAN  open={_yn(s['can_opened'])} ch={s['can_channel']} "
            f"msgs={s['can_messages']} rate={s['can_rate']:0.1f} "
            f"lastid={s['last_can_id']} since={self._since(s['last_can_ts_ns'])}",
            f" DISK free={s['disk_free_gb']:0.1f}GB session={s['session_size_mb']:0.1f}MB "
            f"writeErr={s['write_errors']} qover={s['queue_overflow']}",
        ]
        issues = self.health.recent()[-4:]
        for it in issues:
            sug = f"  -> {it.suggestion}" if it.suggestion else ""
            lines.append(f"  [{it.level}] {it.component}: {it.message}{sug}")
        lines.append(" keys: q stop | p pause | m marker | s scenario | h help | d debug")
        lines.append("=" * 70)
        return "\n".join(lines)

    # -- helpers ---------------------------------------------------------
    def _duration(self, s: dict) -> float:
        if not s["start_time"]:
            return 0.0
        if s["state"] == "paused":
            return s.get("pause_accum", 0.0)
        return time.time() - s["start_time"] - s.get("pause_accum", 0.0)

    def _since(self, ts_ns: int) -> str:
        if not ts_ns:
            return "n/a"
        return f"{(now_ns() - ts_ns) / 1e9:0.1f}s"

    # -- lifecycle -------------------------------------------------------
    def start(self):
        if HAVE_RICH:
            self.live = Live(self._render_rich(), console=self.console,
                             refresh_per_second=4, screen=False)
            self.live.start()

    def update(self):
        if HAVE_RICH and self.live is not None:
            try:
                self.live.update(self._render_rich())
            except Exception:
                pass

    def print_plain(self):
        print(self._render_plain(), flush=True)

    def stop(self):
        if HAVE_RICH and self.live is not None:
            try:
                self.live.stop()
            except Exception:
                pass
            self.live = None


def _yn(b) -> str:
    return "yes" if b else "no"


def _short(p: str, n: int = 50) -> str:
    p = str(p or "")
    return p if len(p) <= n else "..." + p[-(n - 3):]


def _ts(ts_ns: int) -> str:
    if not ts_ns:
        return "n/a"
    return datetime.fromtimestamp(ts_ns / 1e9).strftime("%H:%M:%S")


# ============================================================================
# Preflight checks
# ============================================================================

class CheckResult(object):
    def __init__(self, name, status, detail=""):
        self.name = name
        self.status = status   # OK / WARN / FAIL
        self.detail = detail


def disk_free_gb(path: Path) -> float:
    try:
        usage = shutil.disk_usage(str(path))
        return usage.free / (1024 ** 3)
    except Exception:
        return -1.0


def can_device_state(channel: str) -> Tuple[bool, bool]:
    """Return (exists, is_up) for a CAN device by reading /sys/class/net."""
    base = Path("/sys/class/net") / channel
    exists = base.exists()
    is_up = False
    if exists:
        try:
            oper = (base / "operstate").read_text().strip()
            is_up = oper in ("up", "unknown")  # CAN often reports 'unknown' when up
            flags = base / "flags"
            if flags.exists():
                val = int(flags.read_text().strip(), 16)
                is_up = bool(val & 0x1) or is_up  # IFF_UP
        except Exception:
            pass
    return exists, is_up


def quick_camera_check(source: Any, timeout_open: float = 5.0) -> CheckResult:
    if not HAVE_CV2:
        return CheckResult("Camera open", "FAIL", "opencv-python not installed")
    try:
        cap = build_capture(source, 1280, 720)
        opened = cap is not None and cap.isOpened()
        if opened:
            ok, _ = cap.read()
            cap.release()
            if ok:
                return CheckResult("Camera open", "OK", f"source={source}")
            return CheckResult("Camera open", "WARN",
                               f"opened but no frame yet (source={source})")
        if cap is not None:
            cap.release()
        return CheckResult("Camera open", "FAIL", f"cannot open source={source}")
    except Exception as e:
        return CheckResult("Camera open", "FAIL", str(e))


def run_preflight(cfg: dict, health: HealthMonitor) -> List[CheckResult]:
    results: List[CheckResult] = []

    # camera
    results.append(quick_camera_check(cfg["camera"]["source"]))

    # CAN
    if cfg["can"].get("enabled", True):
        if not HAVE_CAN:
            results.append(CheckResult("CAN open", "WARN", "python-can missing"))
        else:
            exists, up = can_device_state(cfg["can"]["channel"])
            if not exists:
                results.append(CheckResult(
                    "CAN open", "WARN", f"{cfg['can']['channel']} not found"))
            elif not up:
                results.append(CheckResult(
                    "CAN open", "WARN", f"{cfg['can']['channel']} is DOWN"))
            else:
                results.append(CheckResult("CAN open", "OK",
                                           f"{cfg['can']['channel']} up"))
    else:
        results.append(CheckResult("CAN open", "WARN", "CAN disabled in config"))

    # output writable
    root = Path(cfg["dataset"]["root_dir"]).expanduser()
    try:
        root.mkdir(parents=True, exist_ok=True)
        test = root / ".write_test"
        test.write_text("ok")
        test.unlink()
        results.append(CheckResult("Output dir writable", "OK", str(root)))
    except Exception as e:
        results.append(CheckResult("Output dir writable", "FAIL", str(e)))

    # disk space
    free = disk_free_gb(root if root.exists() else Path("."))
    low = float(cfg["runtime"].get("disk_low_gb", 5))
    if free < 0:
        results.append(CheckResult("Disk space", "WARN", "could not determine"))
    elif free < low:
        results.append(CheckResult("Disk space", "WARN",
                                   f"{free:0.1f} GB free (< {low} GB)"))
    else:
        results.append(CheckResult("Disk space", "OK", f"{free:0.1f} GB free"))

    # config validity (already parsed if we got here)
    results.append(CheckResult("Config valid", "OK", ""))

    # safety invariants
    if cfg["safety"].get("allow_can_send", False):
        results.append(CheckResult("CAN send disabled", "FAIL",
                                   "allow_can_send is True!"))
    else:
        results.append(CheckResult("CAN send disabled", "OK", "listen-only"))
    results.append(CheckResult(
        "Manual remote required",
        "OK" if cfg["safety"].get("require_manual_remote", True) else "WARN", ""))

    # mirror failures into the health log
    for r in results:
        if r.status == "FAIL":
            comp = ("CAMERA" if "Camera" in r.name else
                    "CAN" if "CAN" in r.name else
                    "STORAGE" if "dir" in r.name or "Disk" in r.name else "CONFIG")
            health.error(comp, f"preflight FAIL: {r.name} ({r.detail})")
        elif r.status == "WARN":
            comp = ("CAN" if "CAN" in r.name else
                    "STORAGE" if "Disk" in r.name else "SYSTEM")
            health.warn(comp, f"preflight WARN: {r.name} ({r.detail})")
    return results


# ============================================================================
# The collector application
# ============================================================================

class Collector:
    def __init__(self, cfg: dict, logger: logging.Logger, health: HealthMonitor,
                 logs_dir: Path):
        self.cfg = cfg
        self.logger = logger
        self.health = health
        self.logs_dir = logs_dir
        self.stats = Stats()
        self.detailed = False
        self.web_ui = bool(cfg.get("web", {}).get("enabled", False))
        self._web_server = None
        self._scenario_lock = threading.Lock()
        self._scenario = cfg["dataset"].get("default_scenario", "straight_follow")
        self._marker_id = 0

    # scenario / marker accessors used by the writer thread
    def get_scenario(self) -> str:
        with self._scenario_lock:
            return self._scenario

    def get_marker(self) -> int:
        with self._scenario_lock:
            return self._marker_id

    def cycle_scenario(self) -> str:
        scenarios = self.cfg["dataset"].get("scenarios", [self._scenario])
        with self._scenario_lock:
            try:
                i = scenarios.index(self._scenario)
                self._scenario = scenarios[(i + 1) % len(scenarios)]
            except ValueError:
                self._scenario = scenarios[0]
            self.stats.scenario = self._scenario
            return self._scenario

    def add_marker(self, markers_writer: CsvWriter, text: str = "") -> int:
        with self._scenario_lock:
            self._marker_id += 1
            mid = self._marker_id
            scenario = self._scenario
        markers_writer.write_row([now_ns(), mid,
                                  text or f"marker_{mid}", scenario])
        with self.stats.lock:
            self.stats.marker_id = mid
        self.health.info("SYSTEM", f"marker {mid} added (scenario={scenario})")
        return mid

    # ------------------------------------------------------------------
    def collect(self, notes: str = "") -> None:
        cfg = self.cfg
        # ----- create session folder ---------------------------------
        root = Path(cfg["dataset"]["root_dir"]).expanduser()
        ts = datetime.now().strftime("%Y-%m-%d_%H%M%S")
        session_name = f"{ts}_{cfg['dataset']['session_name']}"
        session_dir = root / session_name
        images_dir = session_dir / "images"
        try:
            images_dir.mkdir(parents=True, exist_ok=True)
        except Exception as e:
            self.health.critical("STORAGE",
                                 f"cannot create session dir {session_dir}",
                                 "Check dataset.root_dir permission/space.", exc=e)
            return

        # attach session-scoped log + errors.csv
        add_session_file_handler(self.logger, session_dir / "collector.log")
        self.health.attach_csv(session_dir / "errors.csv")

        # snapshot config used
        with (session_dir / "config_used.yaml").open("w", encoding="utf-8") as f:
            yaml.safe_dump(cfg, f, sort_keys=False, allow_unicode=True)

        # CSV writers
        frames_writer = CsvWriter(session_dir / "frames.csv",
                                  ["timestamp_ns", "frame_index", "image_path",
                                   "scenario", "marker_id", "camera_source",
                                   "width", "height"])
        raw_writer = CsvWriter(session_dir / "raw_can.csv",
                               ["timestamp_ns", "can_timestamp", "arbitration_id",
                                "is_extended_id", "dlc", "data_hex"])
        markers_writer = CsvWriter(session_dir / "markers.csv",
                                   ["timestamp_ns", "marker_id", "marker_text",
                                    "scenario"])

        # init stats
        with self.stats.lock:
            self.stats.session_name = session_name
            self.stats.session_dir = str(session_dir)
            self.stats.dataset_dir = str(root)
            self.stats.scenario = self._scenario
            self.stats.fps_target = float(cfg["camera"]["fps"])
            self.stats.camera_source = str(cfg["camera"]["source"])
            self.stats.can_enabled = bool(cfg["can"].get("enabled", True))
            self.stats.can_channel = cfg["can"]["channel"]
            self.stats.camera_q_max = int(cfg["runtime"]["camera_queue_max"])
            self.stats.writer_q_max = int(cfg["runtime"]["writer_queue_max"])
            self.stats.state = "recording"
            self.stats.start_time = time.time()

        # ----- threads & queues --------------------------------------
        stop_evt = threading.Event()
        pause_evt = threading.Event()
        cam_q: "queue.Queue" = queue.Queue(
            maxsize=int(cfg["runtime"]["camera_queue_max"]))
        # NOTE: cam_q and writer pull from the same queue; we use one bounded
        # queue between capture and the image writer.
        writer_q = cam_q

        cam_thread = CameraThread(cfg, self.stats, self.health, cam_q,
                                  stop_evt, pause_evt)
        writer_thread = ImageWriterThread(cfg, self.stats, self.health, writer_q,
                                          images_dir, frames_writer, stop_evt,
                                          self.get_scenario, self.get_marker)
        can_thread = None
        if cfg["can"].get("enabled", True):
            can_thread = CanThread(cfg, self.stats, self.health, raw_writer, stop_evt)

        # ----- countdown ---------------------------------------------
        countdown = int(cfg["runtime"].get("countdown_sec", 3))
        if countdown > 0:
            for i in range(countdown, 0, -1):
                print(f"  Starting capture in {i} ... "
                      f"(operator: hold the remote, ready to e-stop)", flush=True)
                time.sleep(1.0)

        # ----- start everything --------------------------------------
        cam_thread.start()
        writer_thread.start()
        if can_thread is not None:
            can_thread.start()

        # keyboard handling
        show_preview = bool(cfg["camera"].get("show_preview", True)) and HAVE_CV2

        def handle_key(ch: str):
            if ch == "q":
                self.health.info("SYSTEM", "stop requested (q)")
                stop_evt.set()
            elif ch == "p":
                if pause_evt.is_set():
                    pause_evt.clear()
                    with self.stats.lock:
                        self.stats.state = "recording"
                        # resume: shift start so duration excludes the pause
                    self.health.info("SYSTEM", "resumed")
                else:
                    pause_evt.set()
                    with self.stats.lock:
                        self.stats.state = "paused"
                    self.health.info("SYSTEM", "paused")
            elif ch == "m":
                self.add_marker(markers_writer)
            elif ch == "s":
                new = self.cycle_scenario()
                self.health.info("SYSTEM", f"scenario -> {new}")
            elif ch == "d":
                self.detailed = not self.detailed
                self.health.info("SYSTEM",
                                 f"detailed debug view {'on' if self.detailed else 'off'}")
            elif ch == "h":
                self.health.info("SYSTEM",
                                 "keys: q stop | p pause | m marker | "
                                 "s scenario | d debug | h help")

        key_reader = None
        if not show_preview:
            key_reader = TerminalKeyReader(handle_key, stop_evt)
            key_reader.start()
            if not key_reader.enabled:
                self.health.warn("SYSTEM",
                                 "no TTY and no preview; press Ctrl+C to stop")

        dashboard = Dashboard(self.stats, self.health, cfg, lambda: self.detailed)
        dashboard.start()

        # ----- optional local web UI ---------------------------------
        if self.web_ui:
            try:
                from web_ui import WebDashboardServer
                port = int(cfg.get("web", {}).get("port", 8080))
                host = cfg.get("web", {}).get("host", "0.0.0.0")
                self._web_server = WebDashboardServer(
                    stats=self.stats, health=self.health,
                    preview_cache=_PREVIEW_CACHE, command_cb=handle_key,
                    detailed_getter=lambda: self.detailed,
                    host=host, port=port)
                self._web_server.start()
                self.health.info("SYSTEM",
                                 f"web UI at http://<this-host>:{port} "
                                 f"(control buttons do NOT touch CAN)")
            except Exception as e:
                self.health.warn("SYSTEM", "could not start web UI",
                                 "pip3 install nothing required (stdlib). "
                                 "Check the port is free.", exc=e)

        # ----- main loop ---------------------------------------------
        status_period = float(cfg["runtime"].get("status_print_sec", 5))
        last_status = 0.0
        last_disk = 0.0
        last_pause_start = None
        try:
            while not stop_evt.is_set():
                now = time.time()

                # track paused time accurately
                with self.stats.lock:
                    if self.stats.state == "paused" and last_pause_start is None:
                        last_pause_start = now
                    elif self.stats.state == "recording" and last_pause_start:
                        self.stats.pause_accum += now - last_pause_start
                        last_pause_start = None

                # update queue + storage stats
                if now - last_disk > 1.0:
                    last_disk = now
                    with self.stats.lock:
                        self.stats.camera_q = cam_q.qsize()
                        self.stats.writer_q = writer_q.qsize()
                        self.stats.disk_free_gb = disk_free_gb(session_dir)
                        self.stats.frames_csv_mb = frames_writer.size_mb()
                        self.stats.raw_can_csv_mb = raw_writer.size_mb()
                        self.stats.session_size_mb = _dir_size_mb(session_dir)
                    if self.stats.disk_free_gb >= 0 and \
                            self.stats.disk_free_gb < float(cfg["runtime"]["disk_low_gb"]):
                        self.health.warn(
                            "STORAGE",
                            f"disk space low: {self.stats.disk_free_gb:0.1f} GB",
                            "Free disk space or change dataset.root_dir in config.yaml.")

                # preview window + key handling
                if show_preview:
                    self._show_preview(handle_key, stop_evt)

                # dashboard refresh
                if HAVE_RICH:
                    dashboard.update()
                elif now - last_status >= status_period:
                    last_status = now
                    dashboard.print_plain()

                time.sleep(0.03 if show_preview else 0.1)

        except KeyboardInterrupt:
            self.health.warn("SYSTEM", "KeyboardInterrupt: shutting down safely")
        finally:
            self._shutdown(stop_evt, cam_thread, writer_thread, can_thread,
                           dashboard, key_reader, session_dir, frames_writer,
                           raw_writer, markers_writer, notes)

    # ------------------------------------------------------------------
    def _show_preview(self, handle_key, stop_evt) -> None:
        # grab the most recent frame for display without disturbing the writer
        # queue: we read the latest saved info from stats and re-show a frame by
        # peeking is non-trivial, so we display an overlay-only status frame if
        # no live frame is available. For simplicity we capture a light copy via
        # a module-level cache updated by the camera thread would be ideal; here
        # we draw an info canvas.
        import numpy as np
        s = self.stats.snapshot()
        # If we have a cached preview frame, show it; else a status canvas.
        frame = _PREVIEW_CACHE.get("frame")
        if frame is None:
            canvas = np.zeros((360, 640, 3), dtype="uint8")
        else:
            canvas = frame.copy()
        dur = (time.time() - s["start_time"] - s.get("pause_accum", 0.0)) \
            if s["start_time"] else 0.0
        overlay = [
            f"{s['state'].upper()}  scenario={s['scenario']}  marker={s['marker_id']}",
            f"save_fps={s['actual_save_fps']:0.1f} target={s['fps_target']:0.0f} "
            f"saved={s['frames_saved']} dropped={s['dropped_frames']}",
            f"CAN msgs={s['can_messages']} rate={s['can_rate']:0.1f} "
            f"id={s['last_can_id']}",
            f"dur={dur:0.0f}s  q:cam={s['camera_q']}/{s['camera_q_max']}",
            "keys: q stop  p pause  m marker  s scenario  d debug  h help",
        ]
        y = 24
        for line in overlay:
            cv2.putText(canvas, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 255, 0), 1, cv2.LINE_AA)
            y += 24
        try:
            cv2.imshow("follow_data_collector (preview)", canvas)
            k = cv2.waitKey(1) & 0xFF
            if k != 255:
                handle_key(chr(k).lower())
        except Exception:
            pass

    # ------------------------------------------------------------------
    def _shutdown(self, stop_evt, cam_thread, writer_thread, can_thread,
                  dashboard, key_reader, session_dir, frames_writer, raw_writer,
                  markers_writer, notes) -> None:
        stop_evt.set()
        with self.stats.lock:
            self.stats.state = "stopped"
        dashboard.stop()
        if self._web_server is not None:
            try:
                self._web_server.stop()
            except Exception:
                pass
        self.logger.info("Stopping threads ...")

        for t in (cam_thread, writer_thread, can_thread, key_reader):
            if t is not None:
                try:
                    t.join(timeout=5.0)
                except Exception:
                    pass

        # close camera windows
        if HAVE_CV2:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass

        # final stats
        s = self.stats.snapshot()
        end = time.time()
        duration = (end - s["start_time"] - s.get("pause_accum", 0.0)) \
            if s["start_time"] else 0.0
        meta = {
            "session_name": s["session_name"],
            "start_time": datetime.fromtimestamp(s["start_time"]).isoformat()
            if s["start_time"] else None,
            "end_time": datetime.fromtimestamp(end).isoformat(),
            "duration_sec": round(duration, 2),
            "total_frames": s["frames_saved"],
            "total_can_messages": s["can_messages"],
            "camera_source": s["camera_source"],
            "can_channel": s["can_channel"],
            "fps_target": s["fps_target"],
            "notes": notes,
            "safety_mode": "listen_only_can",
        }
        try:
            with (session_dir / "session_meta.json").open("w", encoding="utf-8") as f:
                json.dump(meta, f, indent=2, ensure_ascii=False)
        except Exception as e:
            self.logger.error("Failed to write session_meta.json: %s", e)

        for w in (frames_writer, raw_writer, markers_writer):
            try:
                w.close()
            except Exception:
                pass
        self.health.close()

        # final summary to terminal
        print("\n" + "=" * 60)
        print(" SESSION COMPLETE")
        print("=" * 60)
        print(f"  session       : {meta['session_name']}")
        print(f"  duration      : {meta['duration_sec']} s")
        print(f"  frames saved  : {meta['total_frames']}")
        print(f"  CAN messages  : {meta['total_can_messages']}")
        print(f"  dropped frames: {s['dropped_frames']}")
        print(f"  write errors  : {s['write_errors']}")
        print(f"  output        : {session_dir}")
        print(f"  issues (W/E/C): {self.health.counts['WARN']}/"
              f"{self.health.counts['ERROR']}/{self.health.counts['CRITICAL']}")
        print("=" * 60)
        self.logger.info("Session saved to %s", session_dir)


# module-level cache so the preview shows real frames captured by the thread
_PREVIEW_CACHE: Dict[str, Any] = {"frame": None}


def _dir_size_mb(path: Path) -> float:
    total = 0
    try:
        for p in path.rglob("*"):
            if p.is_file():
                total += p.stat().st_size
    except Exception:
        pass
    return total / (1024 * 1024)


# Cached camera thread: identical to CameraThread but also stashes the latest
# frame in _PREVIEW_CACHE so the preview window can overlay live video.
# main() swaps the global `CameraThread` name to this class before collecting.
class _CameraThreadCached(CameraThread):
    def run(self):  # noqa: D401
        # identical to parent but also caches frames for preview
        if not self._open():
            return
        max_fail = int(self.cfg["camera"].get("max_consecutive_read_failures", 30))
        fps_window: List[float] = []
        save_period = 1.0 / max(0.1, float(self.cfg["camera"]["fps"]))
        last_save = 0.0
        while not self.stop_evt.is_set():
            if self.pause_evt.is_set():
                time.sleep(0.05)
                continue
            try:
                ok, frame = self.cap.read()
            except Exception as e:
                ok, frame = False, None
                self.health.error("CAMERA", "exception during read", exc=e)
            now = time.time()
            if not ok or frame is None:
                with self.stats.lock:
                    self.stats.consecutive_read_failures += 1
                    fails = self.stats.consecutive_read_failures
                    self.stats.camera_opened = False
                self.health.warn("CAMERA", f"frame read failed ({fails}/{max_fail})")
                if fails >= max_fail:
                    self.health.error("CAMERA",
                                      "too many consecutive read failures; reconnecting")
                    try:
                        if self.cap:
                            self.cap.release()
                    except Exception:
                        pass
                    if not self._open():
                        self.health.critical("CAMERA",
                                             "camera disconnected and reconnect failed")
                        return
                    with self.stats.lock:
                        self.stats.consecutive_read_failures = 0
                else:
                    time.sleep(0.05)
                continue
            ts_ns = now_ns()
            _PREVIEW_CACHE["frame"] = frame  # cache for preview overlay
            with self.stats.lock:
                self.stats.consecutive_read_failures = 0
                self.stats.frames_captured += 1
                self.stats.last_frame_ts_ns = ts_ns
                self.stats.camera_opened = True
            fps_window.append(now)
            fps_window = [t for t in fps_window if now - t <= 1.0]
            with self.stats.lock:
                self.stats.actual_capture_fps = float(len(fps_window))
            if now - last_save >= save_period:
                last_save = now
                try:
                    self.out_q.put_nowait((ts_ns, frame))
                except queue.Full:
                    with self.stats.lock:
                        self.stats.dropped_frames += 1
                        self.stats.queue_overflow += 1
                    self.health.warn(
                        "STORAGE", "image writer queue full; dropping frame",
                        "Image writer queue is growing. Disk may be too slow or "
                        "FPS is too high. Lower camera.fps or use faster storage.")
        try:
            if self.cap:
                self.cap.release()
        except Exception:
            pass


# ============================================================================
# Interactive menu
# ============================================================================

class App:
    def __init__(self, cfg: dict, cfg_path: Optional[Path], logger, health, logs_dir):
        self.cfg = cfg
        self.cfg_path = cfg_path
        self.logger = logger
        self.health = health
        self.logs_dir = logs_dir

    def _print_config(self):
        c = self.cfg
        print("\n  Current configuration")
        print("  ---------------------")
        print(f"  camera source   : {c['camera']['source']}")
        print(f"  save FPS        : {c['camera']['fps']}")
        print(f"  resolution      : {c['camera']['width']}x{c['camera']['height']}")
        print(f"  show preview    : {c['camera']['show_preview']}")
        print(f"  CAN channel     : {c['can']['channel']}  "
              f"(enabled={c['can']['enabled']}, listen_only={c['can']['listen_only']})")
        print(f"  output dir      : {c['dataset']['root_dir']}")
        print(f"  session name    : {c['dataset']['session_name']}")
        print(f"  scenario        : {c['dataset']['default_scenario']}")
        print(f"  allow_can_send  : {c['safety']['allow_can_send']}  "
              f"(must be False in phase 1)")

    def _menu(self) -> str:
        print("\n" + "=" * 60)
        print("  follow_data_collector  —  vision follow-the-person")
        print("  PHASE 1: data collection only. No vehicle control.")
        print("=" * 60)
        self._print_config()
        print("\n  [1] Check camera")
        print("  [2] Check CAN")
        print("  [3] Start collection")
        print("  [4] Preview most recent session")
        print("  [5] Dataset statistics")
        print("  [6] Edit settings (camera/CAN/FPS/scenario)")
        print("  [7] Exit")
        return input("\n  Select an option: ").strip()

    # ---- menu actions -------------------------------------------------
    def check_camera(self):
        src = self.cfg["camera"]["source"]
        print(f"\n  Testing camera source = {src} ...")
        r = quick_camera_check(src)
        tag = {"OK": "[OK]", "WARN": "[WARN]", "FAIL": "[ERROR]"}[r.status]
        print(f"  {tag} {r.detail}")
        if r.status == "FAIL":
            print("  [FIX] Try source 0 or 1 (USB), or an RTSP URL like")
            print("        rtsp://192.168.1.251:554/stream . "
                  "Run scripts/check_camera.py for a scan.")

    def check_can(self):
        ch = self.cfg["can"]["channel"]
        print(f"\n  Checking CAN device {ch} ...")
        if not HAVE_CAN:
            print("  [WARN] python-can not installed. pip3 install python-can")
        exists, up = can_device_state(ch)
        if not exists:
            print(f"  [ERROR] {ch} not found.")
            print("  [FIX] ip -details link show can0 ; confirm USB-CAN driver loaded.")
        elif not up:
            print(f"  [WARN] {ch} exists but is DOWN.")
            print(f"  [FIX] sudo ip link set {ch} up type can bitrate "
                  f"{self.cfg['can']['bitrate']}")
        else:
            print(f"  [OK] {ch} is up.")
            print(f"  Tip: 'candump {ch}' should show live frames while driving.")

    def start(self):
        # preflight
        print("\n  Preflight check")
        print("  ---------------")
        results = run_preflight(self.cfg, self.health)
        any_fail = any(r.status == "FAIL" for r in results)
        critical_block = False
        for r in results:
            tag = {"OK": "[OK]", "WARN": "[WARN]", "FAIL": "[ERROR]"}[r.status]
            print(f"  {tag:7s} {r.name:24s} {r.detail}")
            if r.status == "FAIL" and r.name == "Output dir writable":
                critical_block = True

        if critical_block:
            print("\n  [CRITICAL] Output directory is not writable — cannot start.")
            print("  [FIX] Fix dataset.root_dir permissions or change it in config.yaml.")
            return
        if any_fail:
            ans = input("\n  Some checks FAILED. Continue anyway? [y/N]: ").strip().lower()
            if ans != "y":
                print("  Aborted.")
                return

        notes = input("\n  Session notes (optional, e.g. weather/site/driver): ").strip()
        web = input("  Start the local web UI too? (browser dashboard) [y/N]: ").strip().lower()
        collector = Collector(self.cfg, self.logger, self.health, self.logs_dir)
        collector.web_ui = (web == "y")
        collector.collect(notes=notes)

    def preview_recent(self):
        root = Path(self.cfg["dataset"]["root_dir"]).expanduser()
        sessions = sorted([p for p in root.glob("*") if p.is_dir()]) if root.exists() else []
        if not sessions:
            print("\n  No sessions found in", root)
            return
        latest = sessions[-1]
        print(f"\n  Previewing {latest.name}")
        print("  Launch the dedicated viewer for full controls:")
        print(f"    python3 tools/preview_dataset.py {latest}")
        # minimal inline preview
        if not HAVE_CV2:
            print("  [WARN] opencv not available for inline preview.")
            return
        frames_csv = latest / "frames.csv"
        if not frames_csv.exists():
            print("  [WARN] frames.csv missing.")
            return
        try:
            import csv as _csv
            with frames_csv.open() as f:
                rows = list(_csv.DictReader(f))
            print(f"  {len(rows)} frames. Showing first few (press any key, q to quit)...")
            for row in rows[:50]:
                img = latest / row["image_path"]
                frame = cv2.imread(str(img))
                if frame is None:
                    continue
                cv2.putText(frame, f"{row['frame_index']} {row['scenario']}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 255, 0), 2)
                cv2.imshow("preview", frame)
                if (cv2.waitKey(200) & 0xFF) == ord("q"):
                    break
            cv2.destroyAllWindows()
        except Exception as e:
            print(f"  [ERROR] preview failed: {e}")

    def stats(self):
        root = Path(self.cfg["dataset"]["root_dir"]).expanduser()
        if not root.exists():
            print("\n  No dataset directory yet:", root)
            return
        sessions = sorted([p for p in root.glob("*") if p.is_dir()])
        print(f"\n  Dataset root: {root}")
        print(f"  Sessions: {len(sessions)}")
        total_frames = 0
        total_can = 0
        for s in sessions:
            meta = s / "session_meta.json"
            nf = nc = 0
            if meta.exists():
                try:
                    d = json.loads(meta.read_text())
                    nf = d.get("total_frames", 0)
                    nc = d.get("total_can_messages", 0)
                except Exception:
                    pass
            total_frames += nf
            total_can += nc
            print(f"   - {s.name:40s} frames={nf:<7} can={nc}")
        print(f"\n  TOTAL frames={total_frames}  CAN messages={total_can}")

    def edit_settings(self):
        c = self.cfg
        print("\n  Edit settings (press Enter to keep current value)")
        new = input(f"  camera source [{c['camera']['source']}]: ").strip()
        if new:
            c["camera"]["source"] = int(new) if new.isdigit() else new
        new = input(f"  CAN channel [{c['can']['channel']}]: ").strip()
        if new:
            c["can"]["channel"] = new
        new = input(f"  save FPS [{c['camera']['fps']}]: ").strip()
        if new:
            try:
                c["camera"]["fps"] = float(new)
            except ValueError:
                print("  [WARN] invalid FPS, keeping current.")
        new = input(f"  session name [{c['dataset']['session_name']}]: ").strip()
        if new:
            c["dataset"]["session_name"] = new
        scenarios = c["dataset"].get("scenarios", [])
        print(f"  scenarios available: {', '.join(scenarios)}")
        new = input(f"  default scenario [{c['dataset']['default_scenario']}]: ").strip()
        if new:
            c["dataset"]["default_scenario"] = new
        print("  [OK] settings updated for this run (not written to config.yaml).")

    def run(self):
        while True:
            try:
                choice = self._menu()
            except (EOFError, KeyboardInterrupt):
                print("\n  Bye.")
                return
            if choice == "1":
                self.check_camera()
            elif choice == "2":
                self.check_can()
            elif choice == "3":
                self.start()
            elif choice == "4":
                self.preview_recent()
            elif choice == "5":
                self.stats()
            elif choice == "6":
                self.edit_settings()
            elif choice in ("7", "q", "exit", "quit"):
                print("  Bye.")
                return
            else:
                print("  Unknown option.")


# ============================================================================
# Entry point
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="follow_data_collector — vision follow-the-person data "
                    "collection (PHASE 1: listen-only, no vehicle control).")
    parser.add_argument("--config", type=str, default="config.yaml",
                        help="path to config.yaml")
    parser.add_argument("--no-menu", action="store_true",
                        help="skip the menu and start collecting immediately")
    args = parser.parse_args()

    cfg_path = Path(args.config)
    cfg, warnings = load_config(cfg_path)

    logs_dir = Path("logs")
    logger = setup_logging(cfg["logging"]["level"], logs_dir)
    errors_csv = None  # late-bound per session
    health = HealthMonitor(logger, errors_csv)

    for w in warnings:
        health.warn("CONFIG", w)

    # hard safety assertion at startup
    if cfg["safety"].get("allow_can_send", False):
        health.critical("CAN", "allow_can_send is True — refusing to run",
                        "Phase 1 is listen-only. Set safety.allow_can_send: false "
                        "in config.yaml.")
        print("[CRITICAL] safety.allow_can_send must be false. Exiting.")
        sys.exit(2)

    # make Collector use the cached camera thread for live preview
    global CameraThread
    CameraThread = _CameraThreadCached  # type: ignore

    logger.info("HAVE_CV2=%s HAVE_CAN=%s HAVE_RICH=%s",
                HAVE_CV2, HAVE_CAN, HAVE_RICH)
    if not HAVE_CV2:
        health.error("CAMERA", "opencv-python not installed",
                     "pip3 install opencv-python (or use Jetson system OpenCV)")
    if not HAVE_RICH:
        health.info("SYSTEM", "rich not installed; using plain status output",
                    "pip3 install rich for the live dashboard")

    app = App(cfg, cfg_path, logger, health, logs_dir)
    if args.no_menu:
        app.start()
    else:
        app.run()


if __name__ == "__main__":
    main()
