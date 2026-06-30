#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
follow_data_collector / web_ui.py
=================================

A tiny LOCAL web dashboard + control panel for the data collector.

* Pure Python standard library (http.server) — no Flask/extra dependencies.
* Serves:
    GET  /                -> the HTML dashboard page
    GET  /status          -> JSON snapshot of Stats + recent issues
    GET  /stream.mjpg     -> live MJPEG preview (from the shared preview cache)
    GET  /snapshot.jpg    -> single latest JPEG frame
    POST /control         -> {"cmd": "q|p|m|s|d|h"} forwarded to the collector

SAFETY
------
The control endpoint ONLY forwards collector keystrokes (stop / pause / marker /
scenario / debug). It has NO path to the CAN bus and can never send a CAN frame
or move the vehicle. It is exactly the same command set as the keyboard.

This server runs in a background thread started by Collector.collect() when the
web UI is enabled (config web.enabled or chosen from the menu).
"""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from typing import Any, Callable, Dict, Optional


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    """Python 3.6 backport of http.server.ThreadingHTTPServer (added in 3.7)."""
    daemon_threads = True

try:
    import cv2
    HAVE_CV2 = True
except Exception:
    cv2 = None  # type: ignore
    HAVE_CV2 = False


ALLOWED_COMMANDS = {"q", "p", "m", "s", "d", "h"}


INDEX_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>follow_data_collector — control panel</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  body { margin:0; font-family: system-ui, -apple-system, "Segoe UI", Roboto, sans-serif;
         background:#0d1117; color:#e6edf3; }
  header { padding:14px 20px; background:#161b22; border-bottom:1px solid #30363d;
           display:flex; align-items:center; gap:16px; flex-wrap:wrap; }
  header h1 { font-size:16px; margin:0; font-weight:600; }
  .safety { font-size:12px; color:#f0c674; border:1px solid #6e5494; padding:3px 8px;
            border-radius:6px; }
  .state { font-weight:700; padding:3px 10px; border-radius:6px; font-size:13px; }
  .recording { background:#1a7f37; } .paused { background:#9e6a03; }
  .stopped { background:#b62324; } .idle { background:#30363d; }
  main { display:grid; grid-template-columns: 1.3fr 1fr; gap:16px; padding:16px; }
  @media (max-width: 900px){ main { grid-template-columns:1fr; } }
  .card { background:#161b22; border:1px solid #30363d; border-radius:10px; padding:14px; }
  .card h2 { font-size:13px; margin:0 0 10px; color:#7d8590; text-transform:uppercase;
             letter-spacing:.5px; }
  img#preview { width:100%; border-radius:8px; background:#000; display:block; }
  table { width:100%; border-collapse:collapse; font-size:13px; }
  td { padding:3px 6px; border-bottom:1px solid #21262d; }
  td.k { color:#7d8590; white-space:nowrap; width:45%; }
  td.v { font-variant-numeric: tabular-nums; }
  .controls { display:flex; gap:8px; flex-wrap:wrap; }
  button { cursor:pointer; border:1px solid #30363d; background:#21262d; color:#e6edf3;
           padding:9px 14px; border-radius:8px; font-size:13px; font-weight:600; }
  button:hover { background:#30363d; }
  button.stop { background:#b62324; border-color:#b62324; }
  button.pause { background:#9e6a03; border-color:#9e6a03; }
  #issues { list-style:none; margin:0; padding:0; font-size:12.5px; max-height:230px;
            overflow:auto; }
  #issues li { padding:5px 6px; border-bottom:1px solid #21262d; }
  .lvl { font-weight:700; margin-right:6px; }
  .INFO{color:#7d8590;} .WARN{color:#e3b341;} .ERROR{color:#f85149;}
  .CRITICAL{color:#ff7b72;} .sug{ color:#7d8590; }
  .grid2 { display:grid; grid-template-columns:1fr 1fr; gap:14px; }
</style>
</head>
<body>
<header>
  <h1>follow_data_collector</h1>
  <span class="safety">SAFETY: listen-only CAN · no vehicle control</span>
  <span id="state" class="state idle">IDLE</span>
  <span id="dur"></span>
</header>
<main>
  <div>
    <div class="card">
      <h2>Live preview</h2>
      <img id="preview" src="/stream.mjpg" alt="camera preview"/>
    </div>
    <div class="card" style="margin-top:16px;">
      <h2>Controls (collector only — never CAN)</h2>
      <div class="controls">
        <button class="stop"  onclick="cmd('q')">Stop (q)</button>
        <button class="pause" onclick="cmd('p')">Pause/Resume (p)</button>
        <button onclick="cmd('m')">Add marker (m)</button>
        <button onclick="cmd('s')">Next scenario (s)</button>
        <button onclick="cmd('d')">Toggle debug (d)</button>
        <button onclick="cmd('h')">Help (h)</button>
      </div>
    </div>
  </div>

  <div>
    <div class="grid2">
      <div class="card"><h2>Session</h2><table id="t-session"></table></div>
      <div class="card"><h2>Camera</h2><table id="t-camera"></table></div>
      <div class="card"><h2>CAN</h2><table id="t-can"></table></div>
      <div class="card"><h2>Storage</h2><table id="t-store"></table></div>
    </div>
    <div class="card" style="margin-top:14px;">
      <h2>Issues &amp; warnings</h2>
      <ul id="issues"></ul>
    </div>
  </div>
</main>

<script>
function rows(id, pairs){
  const t = document.getElementById(id);
  t.innerHTML = pairs.map(p =>
    `<tr><td class="k">${p[0]}</td><td class="v">${p[1]}</td></tr>`).join('');
}
async function cmd(c){
  try{
    await fetch('/control', {method:'POST', headers:{'Content-Type':'application/json'},
                             body: JSON.stringify({cmd:c})});
  }catch(e){ console.warn(e); }
}
function yn(b){ return b ? 'yes' : 'no'; }
async function poll(){
  try{
    const r = await fetch('/status'); const s = await r.json();
    const st = document.getElementById('state');
    st.textContent = (s.state||'idle').toUpperCase();
    st.className = 'state ' + (s.state||'idle');
    document.getElementById('dur').textContent = (s.duration||0).toFixed(1) + ' s';
    rows('t-session', [
      ['session', s.session_name||''],
      ['scenario', s.scenario||''],
      ['marker id', s.marker_id||0],
      ['output', (s.session_dir||'').split('/').slice(-1)[0]],
    ]);
    rows('t-camera', [
      ['source', s.camera_source], ['opened', yn(s.camera_opened)],
      ['resolution', s.frame_w + 'x' + s.frame_h],
      ['target fps', (s.fps_target||0).toFixed(1)],
      ['capture fps', (s.actual_capture_fps||0).toFixed(1)],
      ['save fps', (s.actual_save_fps||0).toFixed(1)],
      ['saved', s.frames_saved], ['dropped', s.dropped_frames],
    ]);
    rows('t-can', [
      ['enabled', yn(s.can_enabled)], ['channel', s.can_channel],
      ['opened', yn(s.can_opened)], ['messages', s.can_messages],
      ['msg/s', (s.can_rate||0).toFixed(1)], ['last id', s.last_can_id||'n/a'],
      ['last data', s.last_can_data_hex||''], ['errors', s.can_error_count],
    ]);
    rows('t-store', [
      ['disk free', (s.disk_free_gb||0).toFixed(1)+' GB'],
      ['session size', (s.session_size_mb||0).toFixed(1)+' MB'],
      ['write errors', s.write_errors],
      ['cam queue', s.camera_q + '/' + s.camera_q_max],
      ['writer queue', s.writer_q + '/' + s.writer_q_max],
      ['q overflow', s.queue_overflow],
    ]);
    const ul = document.getElementById('issues');
    ul.innerHTML = (s.issues||[]).slice().reverse().map(it =>
      `<li><span class="lvl ${it.level}">${it.level}</span>
       [${it.component}] ${it.message}
       ${it.suggestion ? '<div class="sug">→ '+it.suggestion+'</div>' : ''}</li>`
    ).join('');
  }catch(e){ /* server may be shutting down */ }
}
setInterval(poll, 1000); poll();
</script>
</body>
</html>
"""


class WebDashboardServer:
    """Runs a ThreadingHTTPServer in a background thread."""

    def __init__(self, stats, health, preview_cache: Dict[str, Any],
                 command_cb: Callable[[str], None], detailed_getter,
                 host: str = "0.0.0.0", port: int = 8080):
        self.stats = stats
        self.health = health
        self.preview_cache = preview_cache
        self.command_cb = command_cb
        self.detailed_getter = detailed_getter
        self.host = host
        self.port = port
        self._httpd: Optional[ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None

    # -- build the JSON status payload ----------------------------------
    def status_payload(self) -> dict:
        s = self.stats.snapshot()
        if s.get("start_time"):
            if s.get("state") == "paused":
                s["duration"] = s.get("pause_accum", 0.0)
            else:
                s["duration"] = time.time() - s["start_time"] - s.get("pause_accum", 0.0)
        else:
            s["duration"] = 0.0
        s.pop("lock", None)
        issues = []
        for it in self.health.recent():
            issues.append({"level": it.level, "component": it.component,
                           "message": it.message, "suggestion": it.suggestion})
        s["issues"] = issues
        return s

    def latest_jpeg(self) -> Optional[bytes]:
        frame = self.preview_cache.get("frame")
        if frame is None or not HAVE_CV2:
            return None
        try:
            ok, buf = cv2.imencode(".jpg", frame,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if ok:
                return buf.tobytes()
        except Exception:
            return None
        return None

    # -- lifecycle ------------------------------------------------------
    def start(self):
        handler = _make_handler(self)
        self._httpd = ThreadingHTTPServer((self.host, self.port), handler)
        self._httpd.daemon_threads = True
        self._thread = threading.Thread(target=self._httpd.serve_forever,
                                        name="WebUI", daemon=True)
        self._thread.start()

    def stop(self):
        if self._httpd is not None:
            try:
                self._httpd.shutdown()
                self._httpd.server_close()
            except Exception:
                pass
            self._httpd = None


def _make_handler(server: WebDashboardServer):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):  # silence default stderr logging
            pass

        def _send(self, code: int, content: bytes, ctype: str):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(content)

        def do_GET(self):
            if self.path == "/" or self.path.startswith("/index"):
                self._send(200, INDEX_HTML.encode("utf-8"), "text/html; charset=utf-8")
            elif self.path.startswith("/status"):
                payload = json.dumps(server.status_payload()).encode("utf-8")
                self._send(200, payload, "application/json")
            elif self.path.startswith("/snapshot"):
                jpg = server.latest_jpeg()
                if jpg is None:
                    self._send(503, b"no frame", "text/plain")
                else:
                    self._send(200, jpg, "image/jpeg")
            elif self.path.startswith("/stream"):
                self._stream_mjpeg()
            else:
                self._send(404, b"not found", "text/plain")

        def do_POST(self):
            if not self.path.startswith("/control"):
                self._send(404, b"not found", "text/plain")
                return
            try:
                n = int(self.headers.get("Content-Length", 0))
                body = self.rfile.read(n) if n else b"{}"
                data = json.loads(body or b"{}")
                cmd = str(data.get("cmd", "")).lower().strip()
            except Exception:
                self._send(400, b'{"ok":false,"error":"bad request"}',
                           "application/json")
                return
            if cmd in ALLOWED_COMMANDS:
                try:
                    server.command_cb(cmd)  # only collector keystrokes — never CAN
                    self._send(200, b'{"ok":true}', "application/json")
                except Exception as e:
                    self._send(500,
                               json.dumps({"ok": False, "error": str(e)}).encode(),
                               "application/json")
            else:
                self._send(400, b'{"ok":false,"error":"command not allowed"}',
                           "application/json")

        def _stream_mjpeg(self):
            self.send_response(200)
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                while True:
                    jpg = server.latest_jpeg()
                    if jpg is not None:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(
                            f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                    time.sleep(0.1)  # ~10 fps stream
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception:
                pass

    return Handler
