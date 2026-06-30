# follow_data_collector

A data-collection tool for a **"vision follow-the-person" self-driving** training
platform (intelligent connected vehicle / 智能网联汽车实训平台).

**Phase 1 is data collection only.** The car is driven **manually with a remote**.
This program records synchronized **camera frames + raw CAN bus traffic** for later
training (`image → steering/speed`, or YOLO person-detector + PID). It is
**receive-only on the CAN bus and never sends a CAN frame** — every CAN write path
is guarded by `safety.allow_can_send`, which must stay `false`.

---

## Project structure

```
follow_data_collector/
├── collect.py              # interactive collector (camera + CAN + dashboard)
├── web_ui.py               # optional local browser dashboard & control panel
├── config.yaml             # configuration
├── requirements.txt        # Python dependencies
├── README.md
├── scripts/
│   ├── start_collector.sh  # ONE-CLICK launcher (checks + runs collect.py)
│   ├── setup_can.sh        # bring up can0
│   └── check_camera.py     # find/test a working camera source
├── tools/
│   ├── inspect_can.py      # analyse raw_can.csv -> guess speed/steering IDs
│   ├── sync_dataset.py     # align frames.csv <-> raw_can.csv
│   └── preview_dataset.py  # play back a recorded session
├── logs/                   # startup_*.log, session_*.log (auto-created)
└── dataset/                # recorded sessions (auto-created)
```

---

## Quick start (one-click)

```bash
bash scripts/start_collector.sh
```

The launcher runs all checks, then starts the interactive collector. You will see:

- **Environment check** — OS, Python, pip, dependencies (offers to `pip3 install`).
- **CAN check** — whether `can0` exists / is UP (offers to bring it up).
- **Camera check** — whether the configured source opens and produces a frame.
- **Storage check** — output directory writable, free disk space.
- **The live collection dashboard** — session/camera/CAN/storage panels.
- **Errors and suggested fix commands** — every problem is shown with a `[FIX]` line.

Tags used everywhere: `[OK]` normal · `[WARN]` continue with caution ·
`[ERROR]` needs attention · `[FIX]` a command you can run. All launcher output is
saved to `logs/startup_YYYYmmdd_HHMMSS.log`.

---

## Manual setup

### 1. Install dependencies

```bash
pip3 install -r requirements.txt
```

On **NVIDIA Jetson**, OpenCV is usually preinstalled with CUDA/GStreamer. Test it
first and skip reinstalling if it works:

```bash
python3 -c "import cv2; print(cv2.__version__)"
```

`rich` and `cantools` are optional. Without `rich` the dashboard falls back to plain
text. Without `cantools` you can still record raw CAN (decoding is added later via a
DBC file).

### 2. Bring up CAN

```bash
bash scripts/setup_can.sh           # can0 @ 500000
# or manually:
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0
candump can0                        # should print frames while the car moves
```

### 3. Check the camera

```bash
python3 scripts/check_camera.py                 # scan USB indices 0..5
python3 scripts/check_camera.py 0               # test one source
python3 scripts/check_camera.py rtsp://192.168.1.251:554/stream
```

Start with a **USB camera** (`source: 0`) to validate the pipeline, then move to the
**network binocular camera**. If you don't know its RTSP URL, just try different
sources in the menu / `check_camera.py` until one works.

### 4. Start collecting

```bash
python3 collect.py --config config.yaml
```

Menu:

```
[1] Check camera
[2] Check CAN
[3] Start collection
[4] Preview most recent session
[5] Dataset statistics
[6] Edit settings (camera/CAN/FPS/scenario)
[7] Exit
```

During collection:

| key | action |
|-----|--------|
| `q` | stop |
| `p` | pause / resume |
| `m` | add a marker event |
| `s` | switch scenario label (cycles the presets) |
| `d` | toggle detailed debug view (queues etc.) |
| `h` | help |

`Ctrl+C` exits safely: threads stop, CSVs flush, `session_meta.json` is written.

---

## Local web UI (browser control panel)

A lightweight, **dependency-free** (Python stdlib) browser dashboard is included. It
shows the live camera stream, all status panels, the issue list, and control buttons.

Enable it in `config.yaml`:

```yaml
web:
  enabled: true
  host: 0.0.0.0
  port: 8080
```

…or answer **y** to "Start the local web UI too?" when you pick `[3] Start collection`.
Then open from any laptop on the same network:

```
http://<工控机IP>:8080      e.g. http://<工控机IP>:8080
```

The web buttons (stop / pause / marker / next scenario / debug) **only control the
collector** — exactly the same as the keyboard. They have **no path to the CAN bus**
and cannot move the vehicle.

---

## Data format

Each run creates `dataset/<timestamp>_<session>/`:

```
2026-06-25_153000_follow_001/
├── images/
│   ├── 00000001_1710000000000000000.jpg
│   └── 00000002_1710000000100000000.jpg
├── frames.csv
├── raw_can.csv
├── markers.csv
├── errors.csv
├── collector.log
├── config_used.yaml
└── session_meta.json
```

**frames.csv** — `timestamp_ns, frame_index, image_path, scenario, marker_id,
camera_source, width, height`

**raw_can.csv** — `timestamp_ns, can_timestamp, arbitration_id, is_extended_id,
dlc, data_hex`

**markers.csv** — `timestamp_ns, marker_id, marker_text, scenario`

**errors.csv** — `timestamp_ns, level, component, message, suggestion,
exception_type, exception_detail`

**session_meta.json** — `session_name, start_time, end_time, duration_sec,
total_frames, total_can_messages, camera_source, can_channel, fps_target, notes,
safety_mode: "listen_only_can"`

### Post-processing

```bash
# align frames with CAN -> synced_samples.csv
python3 tools/sync_dataset.py dataset/2026-06-25_153000_follow_001
python3 tools/sync_dataset.py <session> --window-ms 100
python3 tools/sync_dataset.py <session> --dbc vehicle.dbc   # optional decode

# reverse-engineer CAN -> guess which ID is speed/steering/throttle/brake
python3 tools/inspect_can.py dataset/2026-06-25_153000_follow_001
python3 tools/inspect_can.py <session> --id 0x18

# play back a session
python3 tools/preview_dataset.py dataset/2026-06-25_153000_follow_001
```

---

## Collection advice

Drive slowly in an open, flat area and record **all of these scenarios** (switch the
label with `s`, and drop a marker with `m` at interesting moments):

- `straight_follow` — following a person in a straight line
- `left_turn`, `right_turn` — the person turns
- `person_stop` — the person stops, the car should stop
- `person_lost` — the person leaves the frame
- `multi_person` — several people in view (distractors)
- `no_person` — empty scene (negative examples)

Tips: keep the sun behind the camera, vary distance to the person, and record several
short sessions rather than one huge one. 5–10 save FPS is plenty for follow training.

---

## Safety notes

- **Phase 1 records only. The program never controls the vehicle and never sends CAN.**
- A human **must** hold the remote at all times, ready to **emergency-stop**.
- Collect at **low speed** in an **open area** with no bystanders nearby.
- Validate with a **USB camera first**, then the network binocular camera.
- `safety.allow_can_send` must remain `false`. The collector refuses to start if it is
  `true`, and the CAN thread has no send path at all.

---

## Troubleshooting

**1. `can0 not found`**
USB-CAN adapter not detected. Check `dmesg | grep -i can` and `ip link`. Confirm the
driver/module is loaded. `[FIX]` re-plug the adapter, then `ip -details link show can0`.

**2. `can0 permission denied`**
You usually need root to bring the interface up.
`[FIX] sudo ip link set can0 up type can bitrate 500000`. Receiving frames generally
does not need extra group membership once the interface is UP.

**3. Camera cannot open**
Wrong source index or busy device. `[FIX]` run `python3 scripts/check_camera.py` to
scan indices; close other apps using the camera; try `0` then `1`.

**4. RTSP stream failed**
Wrong URL, wrong IP, or the camera needs GStreamer.
`[FIX]` confirm reachability: `ping 192.168.1.251`. Try
`rtsp://192.168.1.251:554/stream` or an `http://…/?action=stream` MJPEG URL. On Jetson
a GStreamer pipeline string also works as `camera.source`.

**5. No CAN messages**
Interface is UP but silent. `[FIX]` confirm the vehicle is powered and being driven,
check wiring/termination, verify the **bitrate** matches the bus (default 500000), and
test with `candump can0`.

**6. FPS too low**
Disk too slow, resolution too high, or CPU-bound encode. `[FIX]` lower `camera.fps` or
`camera.width/height`, reduce `jpeg_quality`, or write to faster storage (NVMe/SSD).
Watch the **writer queue** in the detailed view (`d`).

**7. Disk space low**
`[FIX]` free space or change `dataset.root_dir` in `config.yaml` to a larger disk. The
dashboard warns below `runtime.disk_low_gb` (default 5 GB).

**8. Image write failed**
Permission or full disk. `[FIX]` check the output directory is writable and has space;
errors are logged to `errors.csv` and collection continues with the next frame.

**9. `config.yaml` parse error**
Invalid YAML. `[FIX]` check indentation/quotes; delete the file and re-run the launcher
to regenerate a default, or copy the example from this README.

**10. OpenCV window not showing**
Headless machine / no display. `[FIX]` set `camera.show_preview: false` and use the
**web UI** (`web.enabled: true`) to watch the stream in a browser, or run over SSH with
X-forwarding. The collector still works fully without a preview window.
```
