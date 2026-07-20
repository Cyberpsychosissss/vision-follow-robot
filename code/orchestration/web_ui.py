#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# web_ui.py (v2) —— 宿主机 :8080 采集 + 跟随状态面板 (纯 stdlib, py3.6)
#
# 这台车 Apollo roscpp 的 C++ advertise() 会崩, 相机数据走「SDK→写文件」(见 obstacles_to_target.py)。
# 所以本面板全部文件式:
#   感知输入: GRAB_OUT/ (left_latest.jpg|ppm, disparity_latest.jpg|pgm, obstacles_latest.json, camera_status.json)
#             —— 由 fr07 的 zkhy_frame_grabber 或我们自己的 grabber 产出。
#   跟随状态: RUNTIME/follow_status.json, target.json (follow_controller 写)。
#   录制: 文件式后台线程, 把每帧 左目/视差/障碍物 存进 dataset/run_NNN/, + candump CAN。
import os, sys, math, time, json, signal, shutil, subprocess, threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from urllib.parse import urlparse, parse_qs

HOST_ROOT = '/home/nvidia/work/AutoApollo/apollo/follow_data'
HOST_DS   = HOST_ROOT + '/dataset'
RUNTIME   = HOST_ROOT + '/runtime'
DEMO_DIR  = HOST_ROOT + '/demos'                # demo 视频(YOLO框+参数HUD 烧进画面)
BIN       = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = RUNTIME + '/follow_config.json'   # 保持距离/最高速度(web 写, follow_controller 热读)
# 默认看我们自己 grabber 的输出(含真实米深度), 不再是 fr07 的
GRAB_OUT  = os.environ.get('GRAB_OUT', RUNTIME + '/grab')
PORT      = int(os.environ.get('FOLLOW_WEB_PORT', '8080'))
REC_FPS   = 5.0
DEMO_FPS  = 10.0    # demo 视频写入帧率(按墙钟节拍写最新帧, 播放即真实速度)
SPEED_CAP = 2.2     # m/s  面板可设的最高速度天花板(=FR-07Pro硬件规格8km/h, car_control ABS_MAX_SPEED)
CONTAINER = os.environ.get('APOLLO_CONTAINER', 'apollo_dev_nvidia')
# ReID 选择器跑在另一个容器(有 onnxruntime; apollo 容器是 py2.7/3.4 装不了现代包)
SEL_CONTAINER = os.environ.get('SELECTOR_CONTAINER', 'follow_yolo2026')

# 容器内感知进程启动命令:
#   grabber      → grab/ (左目图 + 真实米深度 + 障碍物)
#   yolo_follow  → detections.json (所有 person 候选框; --out 改道, 不再直写 target.json)
#   target_selector → target.json (ReID 从候选里挑「主人」; 未锁定时透传最大框 = 旧行为)
# 帧率: grabber --write-fps 15(=不限流, 相机实测 ~12.5fps 全吃满; 旧值9白扔近30%帧), yolo 15hz。
GRAB_CMD = ("docker exec -d %s bash -c 'cd /apollo/follow_data/zkhy_grab && "
            "LD_LIBRARY_PATH=/apollo/follow_data/lib:/apollo/modules/drivers/zkhy/src/Bin "
            "./zkhy_grabber --out-dir /apollo/follow_data/runtime/grab --duration 0 --write-fps 15 "
            "> /tmp/grab.log 2>&1'") % CONTAINER
YOLO_CMD = ("docker exec -d %s bash -c 'cd /apollo/follow_data/trtx/build && "
            "LD_LIBRARY_PATH=/apollo/follow_data/trtx/build:/usr/lib/aarch64-linux-gnu/tegra:/usr/local/cuda-10.0/lib64 "
            "./yolo_follow --engine yolov5s.engine --grab-dir /apollo/follow_data/runtime/grab "
            "--runtime /apollo/follow_data/runtime --out detections.json --hz 15 "
            "> /tmp/yolo_follow.log 2>&1'") % CONTAINER
SEL_CMD = ("docker exec -d %s bash -c 'cd /apollo/follow_data/bin && "
           "OPENBLAS_CORETYPE=ARMV8 PYTHONIOENCODING=utf-8 "
           "FOLLOW_RUNTIME=/apollo/follow_data/runtime "
           "OSNET_ONNX=/apollo/follow_data/models/osnet_x0_25_msmt17.onnx "
           "python3 -u target_selector.py > /tmp/selector.log 2>&1'") % SEL_CONTAINER

S = {'recording': False, 'run': None, 'host_run': None, 'start_ts': 0,
     'can_proc': None, 'rec_thread': None, 'rec_stop': None,
     'frames': 0, 'obs_rows': 0, 'depth': 0,
     'follow_proc': None, 'follow_armed': False,
     'demo': False, 'demo_stop': None, 'demo_thread': None, 'demo_meta': None}
lock = threading.Lock()


def _sh(cmd, timeout=20):
    try:
        p = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, timeout=timeout)
        return p.returncode, p.stdout.decode('utf-8', 'ignore')
    except Exception as e:
        return -1, str(e)


def selector_running():
    return _sh("docker exec %s pgrep -f [t]arget_selector" % SEL_CONTAINER)[0] == 0


def perception_start():
    """容器内拉起 grabber + yolo_follow + target_selector(各自若没在跑才起)。"""
    if _sh("docker exec %s pgrep -x zkhy_grabber" % CONTAINER)[0] != 0:
        _sh(GRAB_CMD)
    if _sh("docker exec %s pgrep -x yolo_follow" % CONTAINER)[0] != 0:
        _sh(YOLO_CMD)
    sel_msg = ''
    if not selector_running():
        rc, out = _sh(SEL_CMD)
        if rc != 0:
            sel_msg = ' ⚠ 选择器起不来(容器 %s 在吗?): %s' % (SEL_CONTAINER, out.strip()[:80])
    return True, '感知已启动(grabber+yolo+选择器)' + sel_msg


def perception_stop():
    _sh("docker exec %s pkill -f [t]arget_selector" % SEL_CONTAINER)
    _sh("docker exec %s pkill -x yolo_follow" % CONTAINER)
    _sh("docker exec %s pkill -x zkhy_grabber" % CONTAINER)
    return True, '感知已停止'


def follow_start(arm, steer_only=False):
    """宿主机拉起 follow_controller。arm=真发帧; steer_only=只转向不前进(速度锁0)。"""
    with lock:
        p = S.get('follow_proc')
        if p and p.poll() is None:
            return False, '跟随控制器已在运行, 先停止'
        cmd = [sys.executable, '-u', os.path.join(BIN, 'follow_controller.py')]   # -u: 日志不缓冲, 短跑也能看到
        if arm:
            cmd.append('--arm')
        if steer_only:
            cmd.append('--steer-only')
        try:
            lp = open('/tmp/follow_ctl.log', 'wb')
            np = subprocess.Popen(cmd, stdout=lp, stderr=subprocess.STDOUT)
        except Exception as e:
            return False, '启动失败: %s' % e
        S['follow_proc'] = np
        S['follow_armed'] = bool(arm)
        S['follow_steer_only'] = bool(steer_only)
    if not arm:
        return True, 'dry-run 不发帧'
    return True, ('ARMED 只转向(不前进, 车不会走)' if steer_only else 'ARMED 真发帧(车轮架空!)')


def follow_stop():
    with lock:
        p = S.get('follow_proc')
        if p and p.poll() is None:
            try:
                p.terminate(); p.wait(timeout=3)
            except Exception:
                try: p.kill()
                except Exception: pass
        S['follow_proc'] = None
        S['follow_armed'] = False
    return True, '跟随已停(控制器退出会下发停车)'


def _write_config(updates):
    """合并写 follow_config.json(别整文件覆盖, 否则设距离会把 max_speed 抹掉)。"""
    cfg = _read_json(CONFIG_FILE) or {}
    cfg.update(updates)
    tmp = CONFIG_FILE + '.tmp'
    json.dump(cfg, open(tmp, 'w'))
    os.replace(tmp, CONFIG_FILE)


def set_dist(path):
    """从 /set_dist?min=&max= 写 follow_config.json, follow_controller 热读生效。"""
    try:
        q = parse_qs(urlparse(path).query)
        mn = float(q.get('min', ['0'])[0]); mx = float(q.get('max', ['0'])[0])
        if not (0.3 < mn < mx < 20):
            return False, '非法: 需 0.3 < 近 < 远 < 20'
        _write_config({'desired_min': mn, 'desired_max': mx})
        return True, '保持距离设为 %.1f~%.1f m (热生效)' % (mn, mx)
    except Exception as e:
        return False, '设置失败: %s' % e


def set_lock(on):
    """锁定/解锁「主人」: 写 follow_config.json, target_selector 边沿触发(靠 lock_ts 变化)。
    锁定 = 把当前画面里最大的人注册成主人, 之后只跟他; 解锁 = 回到「跟最大框」。"""
    if on:
        tgt = _read_json(os.path.join(RUNTIME, 'target.json'), max_age=2)
        if not tgt or not tgt.get('valid'):
            return False, '画面里没检到人, 无法锁定(先站到相机前)'
        _write_config({'lock': True, 'lock_ts': time.time()})
        return True, '🔒 已锁定当前目标为主人(路人不再抢跟随)'
    _write_config({'lock': False})
    return True, '已解锁: 回到跟最显著的人'


def set_speed(path):
    """从 /set_speed?max= 写最高速度, follow_controller 热读生效(天花板 2.2=硬件规格 8km/h)。"""
    try:
        q = parse_qs(urlparse(path).query)
        v = float(q.get('max', ['0'])[0])
        if not (0.05 <= v <= SPEED_CAP):
            return False, '非法: 需 0.05 ≤ 速度 ≤ %.1f m/s' % SPEED_CAP
        _write_config({'max_speed': v})
        warn = ' ⚠ 高速, 确认场地开阔' if v > 0.6 else ''
        return True, '最高速度设为 %.2f m/s (热生效)%s' % (v, warn)
    except Exception as e:
        return False, '设置失败: %s' % e


def next_run():
    n = 1
    if os.path.isdir(HOST_DS):
        for d in os.listdir(HOST_DS):
            if d.startswith('run_') and d[4:].isdigit():
                n = max(n, int(d[4:]) + 1)
    return 'run_%03d' % n


def _read_json(path, max_age=None):
    try:
        if max_age is not None and (time.time() - os.path.getmtime(path)) > max_age:
            return None
        with open(path) as f:
            return json.load(f)
    except Exception:
        return None


def _recorder_loop(host_run, stop_ev):
    """文件式录制: 周期性把 grabber 最新输出存进 run 目录。墙钟时间戳对齐。"""
    imgdir = host_run + '/images'; depthdir = host_run + '/depth'
    for d in (imgdir, depthdir):
        try: os.makedirs(d)
        except OSError: pass
    fcsv = open(host_run + '/frames.csv', 'a'); fcsv.write('seq,filename,ts_wall,src_mtime\n'); fcsv.flush()
    ocsv = open(host_run + '/obstacles.csv', 'a')
    ocsv.write('ts_wall,type,type_id,distance_m,center_x_m,bx,by,bw,bh\n'); ocsv.flush()
    dcsv = open(host_run + '/depth.csv', 'a'); dcsv.write('seq,filename,ts_wall\n'); dcsv.flush()
    # YOLO 检测输出(off_x↔转向 r=-0.53, Phase2 端到端的关键标签), 边采边存省得离线重跑
    tcsv = open(host_run + '/target.csv', 'a')
    tcsv.write('ts_wall,valid,dist_m,lateral_m,off_x,box_h_norm,conf,bx,by,bw,bh\n'); tcsv.flush()
    period = 1.0 / REC_FPS
    last_img_m = last_dsp_m = last_obs_m = last_tgt_m = None
    while not stop_ev.is_set():
        tw = time.time()
        # 左目: 优先 jpg, 否则 ppm
        for src in ('left_latest.jpg', 'left_latest.ppm', 'latest.ppm'):
            p = os.path.join(GRAB_OUT, src)
            if os.path.exists(p):
                try:
                    m = os.path.getmtime(p)
                    if m != last_img_m:
                        last_img_m = m; S['frames'] += 1; seq = S['frames']
                        ext = '.jpg' if src.endswith('jpg') else '.ppm'
                        fn = '%06d%s' % (seq, ext)
                        shutil.copyfile(p, os.path.join(imgdir, fn))
                        fcsv.write('%d,%s,%.6f,%.6f\n' % (seq, fn, tw, m)); fcsv.flush()
                except Exception: pass
                break
        # 视差/深度: jpg 或 pgm
        for src in ('disparity_latest.jpg', 'disparity_latest.pgm', 'depth_latest.png'):
            p = os.path.join(GRAB_OUT, src)
            if os.path.exists(p):
                try:
                    m = os.path.getmtime(p)
                    if m != last_dsp_m:
                        last_dsp_m = m; S['depth'] += 1; seq = S['depth']
                        ext = os.path.splitext(src)[1]
                        fn = '%06d%s' % (seq, ext)
                        shutil.copyfile(p, os.path.join(depthdir, fn))
                        dcsv.write('%d,%s,%.6f\n' % (seq, fn, tw)); dcsv.flush()
                except Exception: pass
                break
        # 障碍物: 解析 obstacles_latest.json 追加 CSV
        oj = os.path.join(GRAB_OUT, 'obstacles_latest.json')
        try:
            m = os.path.getmtime(oj)
            if m != last_obs_m:
                last_obs_m = m
                data = _read_json(oj)
                for o in (data or {}).get('obstacles', []):
                    bb = o.get('bbox', [0, 0, 0, 0]) + [0, 0, 0, 0]
                    ocsv.write('%.6f,%s,%s,%.3f,%.3f,%d,%d,%d,%d\n' % (
                        tw, o.get('type', '?'), o.get('type_id', -1),
                        float(o.get('distance_m', 0)), float(o.get('center_x_m', 0)),
                        bb[0], bb[1], bb[2], bb[3]))
                    S['obs_rows'] += 1
                ocsv.flush()
        except Exception: pass
        # YOLO target.json: 每次更新追一行(含 bbox + 距离/横向/off_x/框高/置信度)
        tj = os.path.join(RUNTIME, 'target.json')
        try:
            m = os.path.getmtime(tj)
            if m != last_tgt_m:
                last_tgt_m = m
                tg = _read_json(tj)
                if tg:
                    bb = (tg.get('bbox') or [0, 0, 0, 0]) + [0, 0, 0, 0]
                    def _n(x): return '' if x is None else x
                    tcsv.write('%.6f,%d,%s,%s,%s,%s,%s,%d,%d,%d,%d\n' % (
                        tw, 1 if tg.get('valid') else 0,
                        _n(tg.get('dist_m')), _n(tg.get('lateral_m')), _n(tg.get('off_x')),
                        _n(tg.get('box_h_norm')), _n(tg.get('conf')),
                        bb[0], bb[1], bb[2], bb[3]))
                    tcsv.flush()
        except Exception: pass
        stop_ev.wait(period)
    for f in (fcsv, ocsv, dcsv, tcsv):
        try: f.flush(); f.close()
        except Exception: pass


def start_recording():
    with lock:
        if S['recording']:
            return False, 'already recording'
        run = next_run(); host_run = HOST_DS + '/' + run
        try: os.makedirs(host_run)
        except OSError: pass
        ts = int(time.time())
        json.dump({'run': run, 'start_ts': ts, 'start_iso': time.strftime('%Y-%m-%dT%H:%M:%S'),
                   'grab_out': GRAB_OUT, 'mode': 'file-based'}, open(host_run + '/meta.json', 'w'), indent=2)
        cp = None
        try:
            cp = subprocess.Popen(['candump', '-L', 'can0'], stdout=open(host_run + '/can.log', 'wb'),
                                  stderr=subprocess.DEVNULL)
        except Exception:
            pass
        stop_ev = threading.Event()
        S.update(frames=0, obs_rows=0, depth=0)
        th = threading.Thread(target=_recorder_loop, args=(host_run, stop_ev)); th.daemon = True; th.start()
        S.update(recording=True, run=run, host_run=host_run, start_ts=ts,
                 can_proc=cp, rec_thread=th, rec_stop=stop_ev)
        return True, run


def stop_recording():
    with lock:
        if not S['recording']:
            return False, 'not recording'
        host_run = S['host_run']; run = S['run']
        if S['rec_stop']: S['rec_stop'].set()
        if S['rec_thread']: S['rec_thread'].join(timeout=3)
        try: S['can_proc'].terminate()
        except Exception: pass
        end = int(time.time()); dur = end - S['start_ts']
        summary = {'run': run, 'frames': S['frames'], 'obstacle_rows': S['obs_rows'],
                   'depth_frames': S['depth'], 'duration_s': dur}
        open(host_run + '/summary.txt', 'w').write(json.dumps(summary, indent=2))
        S.update(recording=False, run=None, host_run=None, can_proc=None, rec_thread=None, rec_stop=None)
        return True, summary


# ---------------- Demo 视频录制(深色工程风: 左数据栏 + 右上相机 + 右下视差图) ----------------
DEMO_ASSETS = os.path.join(BIN, 'demo_assets')   # 预渲染中文标签 PNG(cv2.putText 不支持中文)
SIDEBAR_W = 400
VIEW_W, VIEW_H = 800, 450                        # 右侧两个视图各 800x450, 画布 1200x900

# 深色简约工程风配色(BGR)
C_BG     = (23, 17, 13)      # 画布底 #0d1117
C_LINE   = (65, 50, 38)      # 分隔线 #263241
C_BAR_BG = (59, 41, 30)      # 刻度条底 #1e293b
C_CHIPBG = (51, 39, 28)      # 深色 chip 底 #1c2733
C_TEXT   = (243, 237, 230)   # 主文字 #e6edf3
C_SUB    = (165, 152, 139)   # 次文字 #8b98a5
C_CYAN   = (238, 211, 34)    # #22d3ee 速度
C_GREEN  = (94, 197, 34)     # #22c55e
C_AMBER  = (11, 158, 245)    # #f59e0b 转向
C_RED    = (68, 68, 239)     # #ef4444
C_PURPLE = (246, 92, 139)    # #8b5cf6
C_SKY    = (248, 189, 56)    # #38bdf8
C_GRAY   = (99, 85, 75)      # #4b5563
C_BAND   = (45, 83, 20)      # 保持距离带 #14532d
C_LATCOL = (250, 139, 167)   # 偏移 #a78bfa

# 状态 → (chip 底色, 中文标签素材名)
_STATE_STYLE = {'FOLLOW': (C_GREEN, 'st_follow'), 'HOLD': (C_AMBER, 'st_hold'),
                'STOP_NEAR': (C_RED, 'st_stopnear'), 'SEARCH': (C_PURPLE, 'st_search'),
                'COAST': (C_SKY, 'st_coast'), 'STEER_ONLY': (C_AMBER, 'st_steeronly'),
                'OFF': (C_GRAY, 'st_off')}
# 模式 → 中文素材名(彩色字, 深色底)
_MODE_STYLE = {'armed': 'md_armed', 'steer': 'md_steer', 'dry': 'md_dry', 'off': 'md_off'}

_STATE_BGR = {'FOLLOW': (88, 209, 48), 'HOLD': (10, 214, 255), 'STOP_NEAR': (58, 69, 255),
              'SEARCH': (230, 92, 94), 'COAST': (200, 200, 200), 'STEER_ONLY': (10, 214, 255),
              'OFF': (140, 140, 140)}

_A = {}


def _asset(cv2, name):
    """读预渲染标签 PNG(BGRA), 缓存; 没有返回 None。"""
    if name not in _A:
        p = os.path.join(DEMO_ASSETS, name + '.png')
        _A[name] = cv2.imread(p, -1) if os.path.exists(p) else None
    return _A[name]


def _blit(dst, src, x, y):
    """把 BGRA 小图 alpha 混合进 BGR 画布(左上角 x,y)。"""
    if src is None:
        return
    H, W = dst.shape[:2]
    h, w = src.shape[:2]
    if x >= W or y >= H or x < 0 or y < 0:
        return
    w = min(w, W - x); h = min(h, H - y)
    roi = dst[y:y + h, x:x + w]
    s = src[:h, :w]
    if s.shape[2] == 4:
        a = s[:, :, 3:4].astype('float32') / 255.0
        roi[:] = (s[:, :, :3].astype('float32') * a + roi.astype('float32') * (1.0 - a)).astype('uint8')
    else:
        roi[:] = s[:, :, :3]


def _chip(cv2, img, x1, y1, x2, y2, color, asset):
    """填色 chip + 居中标签。"""
    cv2.rectangle(img, (x1, y1), (x2, y2), color, -1)
    if asset is not None:
        _blit(img, asset, x1 + (x2 - x1 - asset.shape[1]) // 2, y1 + (y2 - y1 - asset.shape[0]) // 2)


def _corner_box(cv2, img, x, y, w, h, color, t=3, L=26):
    """科技风角标式目标框: 四角短线 + 1px 细框。"""
    for cx, cy, dx, dy in ((x, y, 1, 1), (x + w, y, -1, 1), (x, y + h, 1, -1), (x + w, y + h, -1, -1)):
        cv2.line(img, (cx, cy), (cx + dx * L, cy), color, t)
        cv2.line(img, (cx, cy), (cx, cy + dy * L), color, t)
    cv2.rectangle(img, (x, y), (x + w, y + h), color, 1)


def _fv(v, fmt='%.2f'):
    return (fmt % v) if v is not None else '--'


def _demo_compose(cv2, cam, disp, tgt, fol, cam_st, bms):
    """深色工程风画布 1200x900: 左 400px 数据栏 + 右上相机(YOLO框) + 右下视差图(伪彩)。"""
    import numpy as np
    FONT = cv2.FONT_HERSHEY_DUPLEX
    canvas = np.zeros((VIEW_H * 2, SIDEBAR_W + VIEW_W, 3), dtype='uint8')
    canvas[:] = C_BG

    # ================= 右上: 相机画面 + 候选灰框 + 选中绿角标框 =================
    cv_ = cv2.resize(cam, (VIEW_W, VIEW_H))
    bb = tgt.get('bbox'); iw = tgt.get('img_w'); ih = tgt.get('img_h')
    locked = bool(tgt.get('locked'))
    if iw and ih:
        sx, sy = float(VIEW_W) / iw, float(VIEW_H) / ih
        # 其他候选(选择器输出): 细灰框 + 外观相似度%。丢失时也画 → demo 能看到「路人在场但不跟」
        sel = tuple(bb) if (bb and tgt.get('valid')) else None
        for c in (tgt.get('candidates') or []):
            cb = c.get('bbox')
            if not cb or (sel and tuple(cb) == sel):
                continue
            cx, cy = int(cb[0] * sx), int(cb[1] * sy)
            cw2, ch2 = int(cb[2] * sx), int(cb[3] * sy)
            cv2.rectangle(cv_, (cx, cy), (cx + cw2, cy + ch2), C_SUB, 1)
            if c.get('app') is not None:
                cv2.putText(cv_, '%d%%' % round(c['app'] * 100), (cx + 3, max(14, cy - 5)),
                            FONT, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
        if bb and tgt.get('valid'):
            x, y = int(bb[0] * sx), int(bb[1] * sy)
            w, h = int(bb[2] * sx), int(bb[3] * sy)
            _corner_box(cv2, cv_, x, y, w, h, C_GREEN, t=2, L=20)
            txt = _fv(tgt.get('dist_m'), '%.2fm')
            # 锁定时显示外观相似度(lock_conf=原始余弦), 未锁定显示 YOLO 置信度
            if locked and tgt.get('lock_conf') is not None:
                txt += '  %d%%' % round(tgt['lock_conf'] * 100)
            elif tgt.get('conf') is not None:
                txt += '  %d%%' % round(tgt['conf'] * 100)
            tag = _asset(cv2, 'tag_owner' if locked else 'tag_person')
            (tw_, th_), _b = cv2.getTextSize(txt, FONT, 0.5, 1)
            cw = (tag.shape[1] + 8 if tag is not None else 0) + tw_ + 14
            cy1 = max(0, y - 28)
            cv2.rectangle(cv_, (x, cy1), (x + cw, cy1 + 24), C_CHIPBG, -1)
            tx = x + 7
            if tag is not None:
                _blit(cv_, tag, tx, cy1 + (24 - tag.shape[0]) // 2)
                tx += tag.shape[1] + 8
            cv2.putText(cv_, txt, (tx, cy1 + 17), FONT, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    canvas[0:VIEW_H, SIDEBAR_W:] = cv_

    # 锁定状态 chip(相机区右上角): 锁定跟踪/重捕获中/目标丢失; 未锁定不画(与旧版画面一致)
    if locked:
        track = tgt.get('track') or 'TRACKING'
        lk_col = {'TRACKING': C_GREEN, 'REACQ': C_AMBER, 'LOST': C_RED}.get(track, C_GRAY)
        lk_a = _asset(cv2, {'TRACKING': 'chip_lk_track', 'REACQ': 'chip_lk_reacq',
                            'LOST': 'chip_lk_lost'}.get(track, 'chip_lk_track'))
        lk_w = (lk_a.shape[1] + 16) if lk_a is not None else 108
        lx1 = SIDEBAR_W + VIEW_W - lk_w - 8
        cv2.rectangle(canvas, (lx1, 8), (lx1 + lk_w, 34), lk_col, -1)
        if lk_a is not None:
            _blit(canvas, lk_a, lx1 + 8, 8 + (26 - lk_a.shape[0]) // 2)
        else:  # 素材缺失(车上 demo_assets 没更新)也别留空 chip
            cv2.putText(canvas, track, (lx1 + 8, 26), FONT, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    # ================= 右下: 视差图(JET 伪彩) =================
    if disp is not None:
        d_ = cv2.resize(disp, (VIEW_W, VIEW_H))
        canvas[VIEW_H:, SIDEBAR_W:] = cv2.applyColorMap(d_, cv2.COLORMAP_JET)
    else:
        cv2.putText(canvas, '--', (SIDEBAR_W + VIEW_W // 2 - 15, VIEW_H + VIEW_H // 2),
                    FONT, 1.0, C_SUB, 2, cv2.LINE_8)

    # 视图标题 chip + 分隔线
    for cap, cy0 in (('v_cam', 8), ('v_disp', VIEW_H + 8)):
        a = _asset(cv2, cap)
        if a is not None:
            cv2.rectangle(canvas, (SIDEBAR_W + 8, cy0), (SIDEBAR_W + 8 + a.shape[1] + 16, cy0 + 26), C_CHIPBG, -1)
            _blit(canvas, a, SIDEBAR_W + 16, cy0 + (26 - a.shape[0]) // 2)
    cv2.line(canvas, (SIDEBAR_W, 0), (SIDEBAR_W, VIEW_H * 2), C_LINE, 2)
    cv2.line(canvas, (SIDEBAR_W, VIEW_H), (SIDEBAR_W + VIEW_W, VIEW_H), C_LINE, 2)

    # ================= 左侧数据栏 =================
    def hline(y):
        cv2.line(canvas, (20, y), (380, y), C_LINE, 1)

    _blit(canvas, _asset(cv2, 'title'), 20, 24)
    cv2.putText(canvas, time.strftime('%Y-%m-%d  %H:%M:%S'), (20, 80), FONT, 0.5, C_SUB, 1, cv2.LINE_AA)

    state = fol.get('state', 'OFF') if fol else 'OFF'
    scol, sasset = _STATE_STYLE.get(state, _STATE_STYLE['OFF'])
    _chip(cv2, canvas, 20, 96, 380, 144, scol, _asset(cv2, sasset))
    mk = (('steer' if fol.get('steer_only') else 'armed') if fol.get('armed') else 'dry') if fol else 'off'
    _chip(cv2, canvas, 20, 152, 380, 182, C_CHIPBG, _asset(cv2, _MODE_STYLE[mk]))
    hline(200)

    # ---- 目标距离: 大数字 + 0~6m 带状标尺(绿带=保持区间, 亮线=当前) ----
    des = (fol.get('desired') if fol else None) or []
    dist = fol.get('dist_m') if fol else None
    _blit(canvas, _asset(cv2, 'l_dist'), 20, 214)
    # 粗细=2 的数值文字必须用 LINE_8: 车上 cv2 3.2(aarch64) 的 LINE_AA 粗笔画光栅化
    # 会溢出, 从字形中段画一条贯穿到画布右缘的白色横线(实测真实值域 26% 触发, LINE_8 为 0)
    cv2.putText(canvas, _fv(dist), (20, 262), FONT, 1.2, C_TEXT, 2, cv2.LINE_8)
    _blit(canvas, _asset(cv2, 'l_lat'), 235, 214)
    cv2.putText(canvas, _fv(fol.get('lateral_m') if fol else None, '%+.2f'), (235, 258), FONT, 0.85, C_TEXT, 2, cv2.LINE_8)

    def dx(v):
        return int(20 + max(0.0, min(1.0, v / 6.0)) * 360)
    cv2.rectangle(canvas, (20, 284), (380, 300), C_BAR_BG, -1)
    if len(des) >= 2:
        cv2.rectangle(canvas, (dx(des[0]), 284), (dx(des[1]), 300), C_BAND, -1)
    for tv in (0, 2, 4, 6):
        cv2.line(canvas, (dx(tv), 300), (dx(tv), 305), C_SUB, 1)
        cv2.putText(canvas, str(tv), (dx(tv) - 4, 320), FONT, 0.42, C_SUB, 1, cv2.LINE_AA)
    if dist is not None:
        mx = dx(dist)
        cv2.rectangle(canvas, (mx - 1, 280), (mx + 1, 304), C_CYAN, -1)
    hline(334)

    # ---- 下发速度: 数字 + 刻度条(0~vmax, 四分刻度) ----
    vmax = (fol.get('max_speed') if fol else None) or 0.4
    spd = fol.get('cmd_speed') if fol else None
    _blit(canvas, _asset(cv2, 'l_speed'), 20, 348)
    cv2.putText(canvas, _fv(spd), (20, 394), FONT, 0.9, C_TEXT, 2, cv2.LINE_8)
    cv2.putText(canvas, 'max %.2f' % vmax, (285, 392), FONT, 0.5, C_SUB, 1, cv2.LINE_AA)
    cv2.rectangle(canvas, (20, 406), (380, 422), C_BAR_BG, -1)
    if spd is not None and vmax > 0:
        r = max(0.0, min(1.0, spd / vmax))
        if r > 0:
            cv2.rectangle(canvas, (20, 406), (20 + int(360 * r), 422), C_CYAN, -1)
    for i in (1, 2, 3):
        tx_ = 20 + int(360 * i / 4.0)
        cv2.line(canvas, (tx_, 406), (tx_, 422), C_BG, 1)
    hline(440)

    # ---- 下发转向: 弧形表盘(±25°, 正=右) ----
    st_ = fol.get('cmd_steer') if fol else None
    _blit(canvas, _asset(cv2, 'l_steer'), 20, 452)
    ccx, ccy, R = 200, 592, 86
    cv2.ellipse(canvas, (ccx, ccy), (R, R), 0, 210, 330, C_BAR_BG, 12)
    for tdeg in (210, 240, 270, 300, 330):
        rad = math.radians(tdeg)
        p1 = (int(ccx + (R + 8) * math.cos(rad)), int(ccy + (R + 8) * math.sin(rad)))
        p2 = (int(ccx + (R + 14) * math.cos(rad)), int(ccy + (R + 14) * math.sin(rad)))
        cv2.line(canvas, p1, p2, C_SUB, 1)
    if st_ is not None:
        rr = max(-1.0, min(1.0, st_ / 25.0))
        ang = 270 + rr * 60.0
        if ang >= 270:
            cv2.ellipse(canvas, (ccx, ccy), (R, R), 0, 270, ang, C_AMBER, 12)
        else:
            cv2.ellipse(canvas, (ccx, ccy), (R, R), 0, ang, 270, C_AMBER, 12)
        rad = math.radians(ang)
        cv2.line(canvas, (ccx, ccy),
                 (int(ccx + (R - 20) * math.cos(rad)), int(ccy + (R - 20) * math.sin(rad))), C_TEXT, 2)
    cv2.circle(canvas, (ccx, ccy), 5, C_TEXT, -1)
    _blit(canvas, _asset(cv2, 'l_left'), 66, 596)
    _blit(canvas, _asset(cv2, 'l_right'), 316, 596)
    cv2.putText(canvas, _fv(st_, '%+.1f'), (162, 632), FONT, 0.85, C_TEXT, 2, cv2.LINE_8)
    hline(654)

    # ---- 设定 / 感知 ----
    _blit(canvas, _asset(cv2, 'l_keep'), 20, 666)
    cv2.putText(canvas, ('%.1f - %.1f' % (des[0], des[1])) if len(des) >= 2 else '--',
                (20, 696), FONT, 0.6, C_TEXT, 1, cv2.LINE_AA)
    _blit(canvas, _asset(cv2, 'l_vmax'), 205, 666)
    cv2.putText(canvas, '%.2f' % vmax, (205, 696), FONT, 0.6, C_TEXT, 1, cv2.LINE_AA)
    try:
        cfps = float((cam_st or {}).get('left_fps', 0))
    except Exception:
        cfps = 0.0
    try:
        lux = '%d' % round(float(cv2.cvtColor(cam, cv2.COLOR_BGR2GRAY).mean()))
    except Exception:
        lux = '--'
    _blit(canvas, _asset(cv2, 'l_cam'), 20, 712)
    cv2.putText(canvas, '%.1f' % cfps, (20, 742), FONT, 0.6, C_TEXT, 1, cv2.LINE_AA)
    _blit(canvas, _asset(cv2, 'l_conf'), 150, 712)
    cv2.putText(canvas, ('%d' % round(tgt['conf'] * 100)) if tgt.get('conf') is not None else '--',
                (150, 742), FONT, 0.6, C_TEXT, 1, cv2.LINE_AA)
    _blit(canvas, _asset(cv2, 'l_lux'), 255, 712)
    cv2.putText(canvas, lux, (255, 742), FONT, 0.6, C_TEXT, 1, cv2.LINE_AA)
    hline(758)

    # ---- 电池 ----
    _blit(canvas, _asset(cv2, 'l_batt'), 20, 772)
    if bms.get('charging') is not None:
        _chip(cv2, canvas, 290, 766, 380, 792, (246, 130, 59) if bms.get('charging') else C_GRAY,
              _asset(cv2, 'chip_chg' if bms.get('charging') else 'chip_dis'))
    soc = bms.get('soc_pct')
    cv2.putText(canvas, ('%d%%' % soc) if soc is not None else '--', (20, 818), FONT, 0.95, C_TEXT, 2, cv2.LINE_8)
    cv2.rectangle(canvas, (110, 804), (380, 818), C_BAR_BG, -1)
    if soc is not None:
        bc = (246, 130, 59) if bms.get('charging') else (C_RED if soc < 20 else C_GREEN)
        cv2.rectangle(canvas, (110, 804), (110 + int(270 * soc / 100.0), 818), bc, -1)
    _blit(canvas, _asset(cv2, 'l_volt'), 20, 834)
    cv2.putText(canvas, _fv(bms.get('voltage_v'), '%.1f'), (20, 864), FONT, 0.6, C_TEXT, 1, cv2.LINE_AA)
    _blit(canvas, _asset(cv2, 'l_amp'), 150, 834)
    cv2.putText(canvas, _fv(bms.get('current_a'), '%+.1f'), (150, 864), FONT, 0.6, C_TEXT, 1, cv2.LINE_AA)
    _blit(canvas, _asset(cv2, 'l_ah'), 255, 834)
    cv2.putText(canvas, _fv(bms.get('remaining_ah'), '%.1f'), (255, 864), FONT, 0.6, C_TEXT, 1, cv2.LINE_AA)
    return canvas


def _demo_overlay(cv2, f, tgt, fol, cam):
    """在帧上画 YOLO 目标框 + 左上角实时参数 HUD。cv2.putText 不支持中文, HUD 用英文。"""
    H, W = f.shape[:2]
    # YOLO 目标框: target.json 的 bbox 是感知分辨率像素, 按比例缩放到本帧
    bb = tgt.get('bbox'); iw = tgt.get('img_w'); ih = tgt.get('img_h')
    locked = bool(tgt.get('locked'))
    if iw and ih:
        sx, sy = float(W) / iw, float(H) / ih
        sel = tuple(bb) if (bb and tgt.get('valid')) else None
        for c in (tgt.get('candidates') or []):   # 其他候选: 灰框+相似度(丢失时也画)
            cb = c.get('bbox')
            if not cb or (sel and tuple(cb) == sel):
                continue
            cx, cy = int(cb[0] * sx), int(cb[1] * sy)
            cv2.rectangle(f, (cx, cy), (cx + int(cb[2] * sx), cy + int(cb[3] * sy)), (99, 85, 75), 1)
            if c.get('app') is not None:
                cv2.putText(f, '%d%%' % round(c['app'] * 100), (cx + 2, max(14, cy - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
    if bb and iw and ih and tgt.get('valid'):
        sx, sy = float(W) / iw, float(H) / ih
        x, y = int(bb[0] * sx), int(bb[1] * sy)
        w, h = int(bb[2] * sx), int(bb[3] * sy)
        cv2.rectangle(f, (x, y), (x + w, y + h), (88, 209, 48), 2)
        lbl = 'OWNER' if locked else 'PERSON'
        if tgt.get('dist_m') is not None:
            lbl += ' %.2fm' % tgt['dist_m']
        if locked and tgt.get('lock_conf') is not None:
            lbl += ' %d%%' % round(tgt['lock_conf'] * 100)
        elif tgt.get('conf') is not None:
            lbl += ' %d%%' % round(tgt['conf'] * 100)
        (tw_, th_), _ = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        ly = max(th_ + 4, y - 6)
        cv2.rectangle(f, (x, ly - th_ - 4), (x + tw_ + 6, ly + 4), (88, 209, 48), -1)
        cv2.putText(f, lbl, (x + 3, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2)
    # 左上角 HUD(半透明黑底)
    state = fol.get('state', 'OFF') if fol else 'OFF'
    if fol:
        mode = ('ARM-STEER' if fol.get('steer_only') else 'ARMED') if fol.get('armed') else 'DRY-RUN'
    else:
        mode = 'CTRL OFF'
    des = fol.get('desired') or []
    try:
        cam_fps = float((cam or {}).get('left_fps', 0))
    except Exception:
        cam_fps = 0.0
    lines = [
        (time.strftime('%H:%M:%S') + '  ' + mode, (255, 255, 255)),
        ('STATE  ' + state, _STATE_BGR.get(state, (255, 255, 255))),
        ('DIST   %s m    LAT %s m' % (
            ('%.2f' % fol['dist_m']) if fol.get('dist_m') is not None else '--',
            ('%+.2f' % fol['lateral_m']) if fol.get('lateral_m') is not None else '--'), (255, 255, 255)),
        ('CMD    v %s m/s   steer %s deg' % (
            ('%.2f' % fol['cmd_speed']) if fol.get('cmd_speed') is not None else '--',
            ('%+.1f' % fol['cmd_steer']) if fol.get('cmd_steer') is not None else '--'), (255, 255, 255)),
        ('LIMIT  vmax %s m/s   keep %s m' % (
            ('%.2f' % fol['max_speed']) if fol.get('max_speed') is not None else '--',
            ('%.1f-%.1f' % (des[0], des[1])) if len(des) >= 2 else '--'), (200, 200, 200)),
        ('CAM    %.1f fps   CONF %s' % (
            cam_fps,
            ('%d%%' % round(tgt['conf'] * 100)) if tgt.get('conf') is not None else '--'), (200, 200, 200)),
        (('LOCK   %s%s' % (tgt.get('track') or 'ON',
                           (' %d%%' % round(tgt['lock_conf'] * 100)) if tgt.get('lock_conf') is not None else ''))
         if locked else 'LOCK   OFF (largest person)',
         {'TRACKING': (88, 209, 48), 'REACQ': (10, 214, 255), 'LOST': (58, 69, 255)}.get(
             tgt.get('track'), (200, 200, 200)) if locked else (140, 140, 140)),
    ]
    pad, lh = 10, 24
    bw, bh = 380, pad * 2 + lh * len(lines)
    ov = f.copy()
    cv2.rectangle(ov, (0, 0), (bw, bh), (0, 0, 0), -1)
    cv2.addWeighted(ov, 0.55, f, 0.45, 0, f)
    y = pad + 16
    for txt, col in lines:
        cv2.putText(f, txt, (pad, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 1, cv2.LINE_AA)
        y += lh
    return f


_DEMO_CSV_HEADER = ('frame,ts,state,armed,steer_only,dist_m,lateral_m,cmd_speed,cmd_steer,'
                    'max_speed,desired_min,desired_max,gate,tgt_valid,raw_dist_m,raw_lateral_m,'
                    'off_x,box_h_norm,conf,n_persons,bx,by,bw,bh,'
                    'cam_fps,disp_fps,brightness,volt_v,curr_a,remain_ah,soc_pct,charging,'
                    'locked,track,lock_conf,n_cands\n')   # ReID 列追加在尾部, 老分析脚本列序不变


def _demo_write_meta(path_base, meta):
    """收尾写 demo_xxx.meta.json —— Mac 侧 tools/make_multiview.py 的输入契约。

    有了它, 合成脚本不必硬编码画布布局: 以后谁改了 SIDEBAR_W/VIEW_W/VIEW_H 或 DEMO_FPS,
    合成端跟着 meta 走就不会对不上。layout 里的四元组直接就是 ffmpeg crop 的 [w,h,x,y]。
    first/last_frame_ts 与同名 CSV 的 ts 列同源(同一个 time.time() 取值), 手机视频按
    tools/sync_phone.py 解出的 offset 平移后即可与本视频逐帧对齐。
    """
    try:
        d = {
            'schema': 1,
            'name': meta.get('name'), 'video': meta.get('file'), 'csv': meta.get('csv'),
            'fps': DEMO_FPS, 'frames': meta.get('frames', 0),
            'start_ts': meta.get('start'), 'end_ts': time.time(),
            'first_frame_ts': meta.get('first_ts'), 'last_frame_ts': meta.get('last_ts'),
            'clock': 'unix epoch, 车上系统钟(NTP 同步), 与 /slate 场记板同源',
            'layout': {                       # ffmpeg crop 参数: [w, h, x, y]
                'canvas': [SIDEBAR_W + VIEW_W, VIEW_H * 2],
                'sidebar': [SIDEBAR_W, VIEW_H * 2, 0, 0],
                'camera': [VIEW_W, VIEW_H, SIDEBAR_W, 0],
                'disparity': [VIEW_W, VIEW_H, SIDEBAR_W, VIEW_H],
            },
        }
        tmp = path_base + '.meta.json.tmp'
        with open(tmp, 'w') as fh:
            json.dump(d, fh, indent=1, ensure_ascii=False)
        os.replace(tmp, path_base + '.meta.json')
        meta['meta_file'] = os.path.basename(path_base) + '.meta.json'
    except Exception:
        pass          # 写不出 meta 不该拖累录像本身


def _demo_loop(path_base, stop_ev, meta):
    """按墙钟 DEMO_FPS 节拍写最新左目帧+叠加层 → 播放即真实速度。独立于跟随/数据采集。
    同名 .csv 每视频帧记一行全量数据(第N帧=第N行, 与画面严格对齐, 供离线分析)。"""
    import cv2
    writer = None; size = None
    log_f = None
    last_m = None; frame = None
    last_dm = None; disp = None
    last_fresh = time.time()      # 上次拿到新左目帧的墙钟(冻帧检测)
    try:
        while not stop_ev.is_set():
            t0 = time.time()
            for src in ('left_latest.ppm', 'left_latest.jpg', 'latest.ppm'):
                p = os.path.join(GRAB_OUT, src)
                if os.path.exists(p):
                    try:
                        m = os.path.getmtime(p)
                        if m != last_m:
                            img = cv2.imread(p, cv2.IMREAD_COLOR)
                            if img is not None and img.size:   # 写一半读坏 → 跳过, 沿用上一帧
                                frame = img; last_m = m; last_fresh = t0
                    except Exception:
                        pass
                    break
            for src in ('disparity_latest.pgm', 'disparity_latest.jpg'):
                p = os.path.join(GRAB_OUT, src)
                if os.path.exists(p):
                    try:
                        m = os.path.getmtime(p)
                        if m != last_dm:
                            img = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
                            if img is not None and img.size:
                                disp = img; last_dm = m
                    except Exception:
                        pass
                    break
            if frame is None:
                stop_ev.wait(0.2); continue          # 感知没起, 等首帧
            tgt = _read_json(os.path.join(RUNTIME, 'target.json'), max_age=1.5) or {}
            fol = _read_json(os.path.join(RUNTIME, 'follow_status.json'), max_age=2.5) or {}
            cam = _read_json(os.path.join(GRAB_OUT, 'camera_status.json'), max_age=5) or {}
            bms = _read_json(os.path.join(RUNTIME, 'bms.json'), max_age=8) or {}
            try:
                if _asset(cv2, 'title') is not None:  # 有中文素材 → 深色三区排版
                    f = _demo_compose(cv2, frame, disp, tgt, fol, cam, bms)
                else:                                 # 缺素材 → 降级英文 HUD 叠加
                    f = frame.copy()
                    _demo_overlay(cv2, f, tgt, fol, cam)
            except Exception:
                f = frame.copy()                      # 合成出错也别断录
            stale_s = t0 - last_fresh
            if stale_s > 2.0:
                # 相机停帧: 旧版会无限重复最后一帧, 视频看着"正常"其实在录死画面 → 变暗+水印挑明
                f = cv2.convertScaleAbs(f, alpha=0.45)
                cv2.putText(f, 'CAMERA STALE %ds' % int(stale_s), (f.shape[1] // 2 - 160, 60),
                            cv2.FONT_HERSHEY_DUPLEX, 1.1, (68, 68, 239), 2, cv2.LINE_8)
            if writer is None:
                h, w = f.shape[:2]; size = (w, h)
                for cc, ext in (('mp4v', '.mp4'), ('XVID', '.avi'), ('MJPG', '.avi')):
                    path = path_base + ext
                    wr = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*cc), DEMO_FPS, size)
                    if wr.isOpened():
                        writer = wr; meta['path'] = path; meta['file'] = os.path.basename(path)
                        break
                    try: wr.release()
                    except Exception: pass
                    try: os.remove(path)
                    except OSError: pass
                if writer is None:
                    meta['error'] = 'VideoWriter 打不开(无可用编码器)'
                    return
                try:                                  # 同名数据日志(行缓冲, 掉电最多丢一行)
                    log_f = open(path_base + '.csv', 'w', buffering=1)
                    log_f.write(_DEMO_CSV_HEADER)
                    meta['csv'] = os.path.basename(path_base) + '.csv'
                except Exception:
                    log_f = None
            if (f.shape[1], f.shape[0]) != size:
                f = cv2.resize(f, size)
            writer.write(f)
            meta['frames'] = meta.get('frames', 0) + 1
            # 本帧墙钟: CSV 的 ts 列与 meta 的 first/last_frame_ts 必须同源同值,
            # 否则多视角对齐会差半帧 —— 所以只取一次 time.time()
            row_ts = time.time()
            if meta['frames'] == 1:
                meta['first_ts'] = row_ts
            meta['last_ts'] = row_ts
            if log_f is not None:
                try:
                    try:
                        lux = int(round(float(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).mean())))
                    except Exception:
                        lux = None
                    des = (fol.get('desired') or [None, None]) if fol else [None, None]
                    bb = (tgt.get('bbox') or [None, None, None, None])
                    row = [meta['frames'], '%.3f' % row_ts,
                           (fol.get('state') if fol else 'OFF'),
                           int(bool(fol.get('armed'))), int(bool(fol.get('steer_only'))),
                           fol.get('dist_m'), fol.get('lateral_m'),
                           fol.get('cmd_speed'), fol.get('cmd_steer'), fol.get('max_speed'),
                           des[0], des[1], fol.get('gate'),
                           int(bool(tgt.get('valid'))), tgt.get('dist_m'), tgt.get('lateral_m'),
                           tgt.get('off_x'), tgt.get('box_h_norm'), tgt.get('conf'), tgt.get('n_persons'),
                           bb[0], bb[1], bb[2], bb[3],
                           cam.get('left_fps'), cam.get('disparity_fps'), lux,
                           bms.get('voltage_v'), bms.get('current_a'), bms.get('remaining_ah'),
                           bms.get('soc_pct'),
                           ('' if bms.get('charging') is None else int(bool(bms.get('charging')))),
                           int(bool(tgt.get('locked'))), tgt.get('track') or '',
                           tgt.get('lock_conf'), len(tgt.get('candidates') or [])]
                    log_f.write(','.join('' if v is None else str(v) for v in row) + '\n')
                except Exception:
                    pass
            stop_ev.wait(max(0.0, 1.0 / DEMO_FPS - (time.time() - t0)))
    finally:
        if writer is not None:
            try: writer.release()                    # 必须 release, 否则 mp4 缺 moov 打不开
            except Exception: pass
        if log_f is not None:
            try: log_f.close()
            except Exception: pass
        if meta.get('frames'):                       # 有帧才写 sidecar(空录不留垃圾文件)
            _demo_write_meta(path_base, meta)


def demo_start():
    """开录 demo 视频。需感知在跑(有 left_latest 帧); 跟随控制器没跑也能录(HUD 显示 CTRL OFF)。"""
    with lock:
        if S['demo']:
            return False, 'Demo 已在录制'
        try:
            import cv2  # noqa  宿主机 python3-opencv 3.2
        except Exception as e:
            return False, '宿主机 cv2 不可用: %s' % e
        try: os.makedirs(DEMO_DIR)
        except OSError: pass
        name = time.strftime('demo_%Y%m%d_%H%M%S')
        meta = {'name': name, 'frames': 0, 'start': time.time(), 'path': None, 'file': None, 'error': None}
        ev = threading.Event()
        th = threading.Thread(target=_demo_loop, args=(os.path.join(DEMO_DIR, name), ev, meta))
        th.daemon = True; th.start()
        S.update(demo=True, demo_stop=ev, demo_thread=th, demo_meta=meta)
        return True, '⏺ Demo 录制中(YOLO框+参数已烧进视频): %s' % name


def demo_stop():
    with lock:
        if not S['demo']:
            return False, '没有在录 Demo'
        ev = S['demo_stop']; th = S['demo_thread']; meta = S['demo_meta'] or {}
        if ev: ev.set()
        still_writing = False
        if th:
            th.join(timeout=8)
            still_writing = th.is_alive()   # 没退完 = writer 还没 release, mp4 缺 moov 暂打不开
        S.update(demo=False, demo_stop=None, demo_thread=None)
    if meta.get('error'):
        return False, 'Demo 失败: %s' % meta['error']
    if not meta.get('frames'):
        return False, 'Demo 无帧(感知没起? 先「▶ 启动感知」出画面再录)'
    dur = int(time.time() - meta.get('start', time.time()))
    tail = ' ⚠ 文件还在收尾, 等几秒再下载' if still_writing else ', 下方可下载'
    return True, '✔ Demo 已保存: %s (%d帧/%ds)%s' % (meta.get('file', '?'), meta.get('frames', 0), dur, tail)


_LUX = {'m': None, 'v': None}


def _brightness():
    """左目画面平均灰度 0~255(光照强度代理)。按帧 mtime 缓存; 无 cv2/无帧返回 None。"""
    for src in ('left_latest.ppm', 'left_latest.jpg', 'latest.ppm'):
        p = os.path.join(GRAB_OUT, src)
        if not os.path.exists(p):
            continue
        try:
            m = os.path.getmtime(p)
            if m == _LUX['m']:
                return _LUX['v']
            import cv2
            g = cv2.imread(p, 0)
            if g is None or not g.size:
                return _LUX['v']
            _LUX['m'] = m
            _LUX['v'] = int(round(float(g.mean())))
            return _LUX['v']
        except Exception:
            return _LUX['v']
    return None


def status():
    s = {'recording': S['recording'], 'run': S['run']}
    if S['recording']:
        s['frames'] = S['frames']; s['obstacle_rows'] = S['obs_rows']; s['depth_frames'] = S['depth']
        s['elapsed'] = int(time.time()) - S['start_ts']
    # 感知/相机状态
    cam = _read_json(os.path.join(GRAB_OUT, 'camera_status.json'), max_age=5)
    s['camera'] = {'online': cam is not None,
                   'left_fps': round(cam.get('left_fps', 0), 1) if cam else 0,
                   'disparity_fps': round(cam.get('disparity_fps', 0), 1) if cam else 0,
                   'obstacle_fps': round(cam.get('obstacle_fps', 0), 1) if cam else 0,
                   'brightness': (_brightness() if cam is not None else None)}
    obs = _read_json(os.path.join(GRAB_OUT, 'obstacles_latest.json'), max_age=5)
    s['obstacles'] = (obs or {}).get('obstacles', []) if obs else []
    # 跟随状态
    fol = _read_json(os.path.join(RUNTIME, 'follow_status.json'), max_age=3)
    s['follow'] = fol if fol else {'state': 'OFF', 'running': False}
    if fol: s['follow']['running'] = True
    s['bms'] = _read_json(os.path.join(RUNTIME, 'bms.json'), max_age=6) or {}
    # 控制进程状态(从文件新鲜度推断, 不每秒 docker exec)
    tgt = _read_json(os.path.join(RUNTIME, 'target.json'), max_age=3)
    det = _read_json(os.path.join(RUNTIME, 'detections.json'), max_age=3)
    fp = S.get('follow_proc')
    # yolo 灯看 detections.json(新栈)或 target.json(旧栈, yolo 直写);
    # 选择器灯看 target.json 是否带 yolo_reid 来源(比每秒 docker exec 便宜)
    s['ctl'] = {'grabber': bool(s['camera']['online']),
                'yolo': (det is not None) or (tgt is not None),
                'selector': bool(tgt and tgt.get('source') == 'yolo_reid'),
                'follow_running': bool(fp and fp.poll() is None),
                'armed': bool(S.get('follow_armed'))}
    s['target'] = tgt or {}
    # demo 录制状态 + 最近文件(供面板下载)
    dm = S.get('demo_meta') or {}
    s['demo'] = {'recording': S['demo'], 'file': dm.get('file'), 'frames': dm.get('frames', 0),
                 'elapsed': (int(time.time() - dm['start']) if S['demo'] and dm.get('start') else 0),
                 'error': dm.get('error')}
    try:
        # 只列视频: 同目录还有同名 .csv 与 .meta.json, 不过滤会把这 5 个下载位挤掉
        names = sorted([n for n in os.listdir(DEMO_DIR)
                        if n.startswith('demo_') and n.endswith(('.mp4', '.avi'))], reverse=True)[:5]
        s['demos'] = [{'name': n, 'mb': round(os.path.getsize(os.path.join(DEMO_DIR, n)) / 1e6, 1)}
                      for n in names]
    except Exception:
        s['demos'] = []
    s['config'] = _read_json(CONFIG_FILE) or {}
    try:
        st = os.statvfs(HOST_DS); s['disk_free_gb'] = round(st.f_bavail * st.f_frsize / 1e9, 1)
    except Exception: pass
    return s


def serve_jpeg(names, resize='720x'):
    """返回 JPEG 字节。浏览器不认 PPM/PGM: 优先 cv2 转(快, 车上现成),
    没有 cv2 再退 ImageMagick convert(有些机器没装, 这是摄像头黑屏的老病根)。"""
    try:
        width = int(str(resize).split('x')[0])
    except Exception:
        width = 720
    for n in names:
        p = os.path.join(GRAB_OUT, n)
        if not os.path.exists(p):
            continue
        if p.endswith('.jpg') or p.endswith('.jpeg'):
            try:
                return open(p, 'rb').read()
            except Exception:
                continue
        try:  # ① cv2: ppm/pgm 直接读 + 缩宽 + 编 JPEG, 单帧 ~20ms
            import cv2
            img = cv2.imread(p, cv2.IMREAD_UNCHANGED)
            if img is not None:
                h, w = img.shape[:2]
                if w > width:
                    img = cv2.resize(img, (width, int(h * width / w)))
                ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
                if ok:
                    return buf.tobytes()
        except Exception:
            pass
        try:  # ② 兜底: ImageMagick
            r = subprocess.run(['convert', p, '-resize', resize, 'jpg:-'],
                               stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, timeout=6)
            if r.returncode == 0 and r.stdout:
                return r.stdout
        except Exception:
            pass
    return None


PAGE = u"""<!doctype html><html><head><meta charset=utf-8><title>视觉跟随机器人 · 实验控制台</title>
<meta name=viewport content="width=device-width,initial-scale=1"><style>
:root{--paper:#eae5d6;--panel:#f6f3e8;--ink:#23241e;--sub:#6f6a5c;--rule:#c0b8a3;--rule2:#4f4b40;
--navy:#1e3a5f;--red:#9c2b23;--amber:#8a6a2b;--green:#2e6b3c;--violet:#5b4a8a;--blue:#2b5d8a;--gray:#857f70;
--mono:'SF Mono',Menlo,Consolas,'Courier New',monospace;--serif:Georgia,'Times New Roman','Songti SC','STSong',serif}
*{box-sizing:border-box}
body{margin:0 auto;padding:22px 20px;background:var(--paper);color:var(--ink);max-width:1300px;
 font-family:-apple-system,'PingFang SC','Helvetica Neue',sans-serif;font-size:14px;
 background-image:repeating-linear-gradient(0deg,rgba(79,75,64,.028) 0 1px,transparent 1px 28px)}
header.tb{border:2px solid var(--rule2);outline:1px solid var(--rule2);outline-offset:3px;background:var(--panel);
 display:flex;justify-content:space-between;align-items:center;gap:14px;padding:16px 20px;margin-bottom:20px;flex-wrap:wrap}
.doc{font:600 10px var(--mono);letter-spacing:.28em;color:var(--sub)}
h1{font:700 23px var(--serif);letter-spacing:.08em;margin:4px 0 5px;color:var(--ink)}
.subtt{font-size:12px;color:var(--sub);letter-spacing:.06em}
table.meta{border-collapse:collapse;font:12px var(--mono)}
table.meta th{font-weight:600;color:var(--sub);text-align:left;padding:3px 12px 3px 0;letter-spacing:.2em;white-space:nowrap}
table.meta td{padding:3px 0;border-bottom:1px dotted var(--rule);white-space:nowrap}
main{display:grid;grid-template-columns:430px 1fr;gap:18px;align-items:start}
@media(max-width:1020px){main{grid-template-columns:1fr}}
.panel{background:var(--panel);border:1px solid var(--rule2);box-shadow:3px 3px 0 rgba(70,64,52,.14);margin-bottom:16px}
.panel h2{margin:0;padding:8px 12px;border-bottom:1px solid var(--rule2);display:flex;align-items:center;gap:9px;
 font:700 13.5px var(--serif);letter-spacing:.14em;background:linear-gradient(#f0ecdc,#e7e1cd)}
.panel h2 .no{font:700 10.5px var(--mono);color:#f6f3e8;background:var(--rule2);padding:2px 6px;letter-spacing:.1em}
.panel h2 .en{font:400 9.5px var(--mono);color:var(--sub);letter-spacing:.24em}
.panel h2 .hr{margin-left:auto;display:flex;gap:5px;align-items:center;font-weight:400;letter-spacing:0}
.pb{padding:13px 14px}
button{font-family:inherit;font-size:13.5px;font-weight:600;letter-spacing:.05em;color:var(--ink);
 background:#f0ecdd;border:1.5px solid var(--rule2);border-radius:2px;padding:9px 15px;cursor:pointer;
 box-shadow:2px 2px 0 rgba(70,64,52,.28);margin:2px 6px 2px 0}
button:hover{background:var(--navy);border-color:var(--navy);color:#f2eee0}
button:active{transform:translate(2px,2px);box-shadow:none}
button:disabled{opacity:.38;cursor:default;box-shadow:none;transform:none}
button.primary{background:var(--navy);border-color:#152c49;color:#f2eee0}
button.primary:hover{background:#16304f}
button.warn{border-color:var(--amber);color:var(--amber);background:#f4efdc}
button.warn:hover{background:var(--amber);border-color:var(--amber);color:#fff}
button.danger{background:var(--red);border-color:#611712;color:#f7efe5}
button.danger:hover{background:#7f1f19;border-color:#611712}
button.estop{display:block;width:100%;margin:12px 0 0;background:var(--red);border:2px solid #5c1611;color:#fff;
 font-size:15px;letter-spacing:.5em;padding:13px;text-indent:.5em}
.steps{display:grid;gap:9px;margin-top:4px}
.steps button{width:100%;text-align:left;margin:0}
.steps .bn{font-family:var(--serif);font-weight:700;margin-right:7px}
.steps .bs{display:block;font-weight:400;font-size:11px;opacity:.75;letter-spacing:.03em;margin-top:3px}
.badge{display:inline-block;border:1px solid;padding:2px 8px;font:700 11.5px var(--mono);letter-spacing:.06em;background:transparent}
.FOLLOW{color:var(--green);border-color:var(--green);background:rgba(46,107,60,.08)}
.HOLD,.STEER_ONLY{color:var(--amber);border-color:var(--amber);background:rgba(138,106,43,.08)}
.STOP_NEAR{color:var(--red);border-color:var(--red);background:rgba(156,43,35,.08)}
.SEARCH{color:var(--violet);border-color:var(--violet);background:rgba(91,74,138,.08)}
.COAST,.RETURN{color:var(--blue);border-color:var(--blue);background:rgba(43,93,138,.08)}
.SEEK{color:var(--amber);border-color:var(--amber);background:rgba(138,106,43,.08)}
.OFF{color:var(--gray);border-color:var(--gray)}
.k{font-size:10.5px;color:var(--sub);letter-spacing:.08em}
.grid{display:grid;gap:7px;margin-top:9px}
.g3{grid-template-columns:repeat(3,1fr)}.g4{grid-template-columns:repeat(4,1fr)}
.kv{border:1px solid var(--rule);background:#faf7ec;padding:7px 9px}
.kv .v{font:600 19px var(--mono);margin-top:2px;color:var(--ink);white-space:nowrap}
.frow{display:flex;align-items:center;gap:8px;margin:10px 0;flex-wrap:wrap}
.fl{font:600 12px var(--serif);letter-spacing:.14em;min-width:64px}
.fu{font:11.5px var(--mono);color:var(--sub)}
input[type=number]{width:74px;font:600 14px var(--mono);padding:7px;border:1.5px solid var(--rule2);border-radius:2px;background:#fffdf3;color:var(--ink)}
input[type=number]:focus{outline:2px solid rgba(30,58,95,.3);border-color:var(--navy)}
input[type=checkbox]{accent-color:var(--navy)}
label{font-size:13px;margin-right:12px;cursor:pointer}
.note{font-size:12px;color:var(--sub);border-left:3px solid var(--rule);padding:3px 9px;margin-top:10px;line-height:1.6}
.chk{font:11.5px var(--mono);margin-top:10px;color:var(--sub);border:1px dashed var(--rule2);padding:7px 9px;letter-spacing:.04em}
.console{font:12px var(--mono);background:#26261f;color:#d8d1ba;border:1px solid var(--rule2);
 padding:9px 11px;min-height:38px;line-height:1.5;box-shadow:3px 3px 0 rgba(70,64,52,.14)}
.console:before{content:'>> ';color:#8f9e78}
.sub{font:700 12.5px var(--serif);letter-spacing:.15em;margin:2px 0 8px}
.subnote{font:400 11px var(--mono);color:var(--sub);letter-spacing:.02em}
hr.dr{border:0;border-top:1px dashed var(--rule2);margin:13px 0}
.files{margin-top:8px;font-size:12px;line-height:1.9}
.mtitle{font:700 12px var(--serif);letter-spacing:.15em;margin:0 0 6px;color:var(--ink);display:flex;justify-content:space-between;align-items:center}
img{width:100%;border:1px solid var(--rule2);background:#141412;display:block}
table.data{width:100%;border-collapse:collapse;font:12.5px var(--mono)}
table.data th{text-align:left;font:600 10.5px var(--mono);letter-spacing:.12em;color:var(--sub);border-bottom:1px solid var(--rule2);padding:4px 6px}
table.data td{padding:4px 6px;border-bottom:1px dotted var(--rule);color:var(--ink)}
footer{grid-column:1/-1;text-align:center;font:10.5px var(--mono);letter-spacing:.22em;color:var(--sub);
 border-top:1px solid var(--rule);margin-top:6px;padding-top:12px}
a{color:var(--navy)}
</style></head><body>

<header class=tb>
<div>
<div class=doc>PERSON-FOLLOWING ROBOT · EXPERIMENT CONSOLE</div>
<h1>视觉跟随机器人 · 实验控制台</h1>
<div class=subtt>双目立体视觉 — YOLOv5s·TensorRT 检测 — OSNet 行人重识别 — CAN 底盘控制</div>
</div>
<table class=meta>
<tr><th>修 订</th><td>REV.9 / 2026-07-15</td></tr>
<tr><th>平 台</th><td>Jetson AGX Xavier</td></tr>
<tr><th>时 钟</th><td id=clk>——</td></tr>
</table>
</header>

<main>
<div>

<section class=panel><h2><span class=no>01</span>感知系统<span class=en>PERCEPTION</span>
<span class=hr><span id=cgrab class="badge OFF">grabber</span><span id=cyolo class="badge OFF">yolo</span><span id=csel class="badge OFF">选择器</span></span></h2>
<div class=pb>
<button class=primary onclick="go('perception_start')">▶ 启动感知</button>
<button onclick="go('perception_stop')">■ 停止感知</button>
<div class=note>依次拉起 相机采集 → YOLO 检测 → ReID 选择器。三灯全绿后右侧监视器应有画面。</div>
</div></section>

<section class=panel><h2><span class=no>02</span>目标锁定<span class=en>TARGET LOCK · ReID</span>
<span class=hr><span id=clock class="badge OFF">未锁定</span></span></h2>
<div class=pb>
<button class=primary onclick="lockConfirm()">🔒 锁定我为主人</button>
<button onclick="go('unlock')">🔓 解锁</button>
<div class=note>本人站于相机前(绿框框住自己)再锁定。锁定后仅跟随注册者;画面中其他人以灰色虚线框标注外观相似度,不参与跟随。解锁即恢复「跟最显著者」。</div>
</div></section>

<section class=panel><h2><span class=no>03</span>跟随控制<span class=en>FOLLOW CONTROL</span>
<span class=hr><span id=cfollow class="badge OFF">未运行</span></span></h2>
<div class=pb>
<div class=steps>
<button onclick="go('follow_dry')"><span class=bn>Ⅰ</span>DRY-RUN 试运行<span class=bs>只计算决策,不发 CAN 帧 — 绝对安全,先看决策对不对</span></button>
<button class=warn onclick="steerConfirm()"><span class=bn>Ⅱ</span>仅转向跟随<span class=bs>速度锁 0,车不前进,只打方向轮 — 需急停释放</span></button>
<button class=danger onclick="armConfirm()"><span class=bn>Ⅲ</span>ARM 正式跟随<span class=bs>真发帧控车 — 须完成下方检查单并二次确认</span></button>
</div>
<button class=estop onclick="go('follow_stop')">■ 停止 / 急停</button>
<div class=chk>ARM 前置检查单 □ 车轮架空 □ 充电枪已拔 □ 急停已释放 □ 场地清空</div>
</div></section>

<section class=panel><h2><span class=no>04</span>参数整定<span class=en>PARAMETERS · 热生效</span></h2>
<div class=pb>
<div class=frow><span class=fl>保持距离</span><input id=dmin type=number step=0.5 min=0.5 value=2> —
<input id=dmax type=number step=0.5 value=4><span class=fu>m</span><button onclick="setDist()">设 定</button></div>
<div class=frow><span class=fl>速度上限</span><input id=vmax type=number step=0.1 min=0.1 max=2.2 value=0.4>
<span class=fu>m/s (≤2.2 硬件上限)</span><button onclick="setSpeed()">设 定</button><span id=vcur class=fu></span></div>
<div class=frow><span class=fl>模 式</span>
<label><input id=mgrass type=checkbox onchange="setMode()"> 草地模式 (最低驱动 0.5)</label>
<label><input id=mseek type=checkbox onchange="setMode()"> 丢失寻回 (满舵找人+倒车归位)</label></div>
</div></section>

<section class=panel><h2><span class=no>05</span>记录<span class=en>RECORDING</span></h2>
<div class=pb>
<div class=sub>DEMO 视频 <span class=subnote>数据栏 + 锁定/候选框直接烧进画面</span></div>
<button id=demostart onclick="go('demo_start')">⏺ 录制 Demo</button>
<button id=demostop onclick="go('demo_stop')" disabled>⏹ 停止并保存</button>
<span id=demostat class=fu></span>
<div id=demolist class=files></div>
<hr class=dr>
<div class=sub>数据集记录 <span class=subnote><span id=dot style="color:#857f70">●</span> <span id=stext>空闲</span></span></div>
<button id=start onclick="go('start')">▶ 开始记录</button>
<button id=stop onclick="go('stop')" disabled>■ 结束记录</button>
<div class=k id=msg style=margin-top:6px></div>
<div class="grid g3">
<div class=kv><div class=k>RUN</div><div class=v id=run>—</div></div>
<div class=kv><div class=k>图像帧</div><div class=v id=frames>—</div></div>
<div class=kv><div class=k>视差帧</div><div class=v id=depth>—</div></div>
<div class=kv><div class=k>障碍物行</div><div class=v id=obsrows>—</div></div>
<div class=kv><div class=k>时长 s</div><div class=v id=elapsed>—</div></div>
<div class=kv><div class=k>磁盘 GB</div><div class=v id=disk>—</div></div>
</div>
</div></section>

<div id=ctlmsg class=console>ARM 前务必: 车轮架空 / 充电枪拔出 / 急停释放</div>
</div>

<div>
<section class=panel><h2><span class=no>A</span>遥测<span class=en>TELEMETRY</span>
<span class=hr><span id=fstate class="badge OFF">OFF</span></span></h2>
<div class=pb>
<div style="font:12px var(--mono);color:var(--sub)" id=farm>控制器未运行</div>
<div class="grid g4">
<div class=kv><div class=k>目标距离 m</div><div class=v id=fdist>—</div></div>
<div class=kv><div class=k>横向偏移 m</div><div class=v id=flat>—</div></div>
<div class=kv><div class=k>下发车速 m/s</div><div class=v id=fspeed>—</div></div>
<div class=kv><div class=k>下发转向 °</div><div class=v id=fsteer>—</div></div>
<div class=kv><div class=k>相机 fps</div><div class=v id=cfps>—</div></div>
<div class=kv><div class=k>视差 fps</div><div class=v id=dfps>—</div></div>
<div class=kv><div class=k>置信度 %</div><div class=v id=fconf>—</div></div>
<div class=kv><div class=k>光照 0–255</div><div class=v id=clux>—</div></div>
</div>
</div></section>

<section class=panel><h2><span class=no>B</span>监视器<span class=en>MONITOR</span></h2>
<div class=pb>
<div class=mtitle>左目 · YOLO / ReID 标注
<label class=fu style="letter-spacing:0"><input type=checkbox id=ovlchk onchange="localStorage.setItem('ovl',this.checked?'1':'0')"> 叠加速度/转向</label></div>
<div id=prevwrap style="position:relative;line-height:0">
<img id=prev src="/stream">
<div id=cands style="position:absolute;left:0;top:0;width:100%;height:100%;pointer-events:none"></div>
<div id=ybox style="position:absolute;border:2px solid #30d158;box-shadow:0 0 0 1px rgba(0,0,0,.6);display:none;pointer-events:none"></div>
<div id=ylabel style="position:absolute;background:#30d158;color:#000;font-size:11px;font-weight:700;padding:1px 5px;display:none;pointer-events:none;white-space:nowrap"></div>
<div id=ovl style="position:absolute;left:8px;top:8px;background:rgba(20,20,16,.68);color:#f0ecdd;padding:7px 11px;font-size:13px;line-height:1.65;display:none;pointer-events:none;white-space:nowrap;font-variant-numeric:tabular-nums"></div>
</div>
<div class=mtitle style="margin-top:12px">视差图 · 伪彩</div>
<img id=disp src="/dispstream">
</div></section>

<section class=panel><h2><span class=no>C</span>电源<span class=en>POWER · BMS</span>
<span class=hr><span id=bchg class="badge OFF">—</span></span></h2>
<div class=pb>
<div style="display:flex;align-items:center;gap:16px">
<div style="font:700 30px var(--mono);min-width:92px" id=bsoc>—</div>
<div style="flex:1">
<div style="background:#ddd6c2;border:1px solid var(--rule2);height:15px"><div id=bbar style="height:100%;width:0%;background:#2e6b3c;transition:width .4s"></div></div>
<div class=k id=bdetail style=margin-top:5px>—</div></div></div>
<div class="grid g3">
<div class=kv><div class=k>电压 V</div><div class=v id=bvolt>—</div></div>
<div class=kv><div class=k>电流 A</div><div class=v id=bamp>—</div></div>
<div class=kv><div class=k>剩余 Ah</div><div class=v id=bah>—</div></div>
</div>
</div></section>

<section class=panel><h2><span class=no>D</span>障碍物<span class=en>OBSTACLES · 相机自带</span></h2>
<div class=pb>
<table class=data id=obstab><tr><th>类型</th><th>距离 m</th><th>横向 m</th></tr></table>
</div></section>
</div>

<footer>FOLLOW ROBOT PLATFORM · 控制回路 40 Hz · CAN 50 Hz · 面板轮询 1 Hz · ARM 操作须遵守检查单</footer>
</main>

<script>
function go(a){fetch('/'+a,{method:'POST'}).then(r=>r.json()).then(j=>{document.getElementById('ctlmsg').textContent=(j.msg||j.result||JSON.stringify(j));tick();}).catch(e=>{document.getElementById('ctlmsg').textContent=''+e;tick();});}
function armConfirm(){if(confirm('确认 ARM 真发帧控车?\\n确认: 车轮已架空 / 充电枪已拔 / 急停已释放?'))go('follow_arm');}
function setDist(){var mn=document.getElementById('dmin').value,mx=document.getElementById('dmax').value;
 fetch('/set_dist?min='+mn+'&max='+mx,{method:'POST'}).then(r=>r.json()).then(j=>{document.getElementById('ctlmsg').textContent=(j.msg||JSON.stringify(j));tick();}).catch(e=>{document.getElementById('ctlmsg').textContent=''+e;});}
function steerConfirm(){if(confirm('只转向不前进(车不会走, 速度锁0)。人站相机前测转向追人。开始?'))go('follow_steer');}
function setSpeed(){var v=parseFloat(document.getElementById('vmax').value);
 if(isNaN(v)||v<=0){document.getElementById('ctlmsg').textContent='速度无效';return;}
 if(v>0.6&&!confirm('设最高速度 '+v+' m/s (>0.6 已超保守值)。确认场地开阔、安全员在场?'))return;
 fetch('/set_speed?max='+v,{method:'POST'}).then(r=>r.json()).then(j=>{document.getElementById('ctlmsg').textContent=(j.msg||JSON.stringify(j));tick();}).catch(e=>{document.getElementById('ctlmsg').textContent=''+e;});}
function tick(){fetch('/status').then(r=>r.json()).then(s=>{
 var rec=s.recording, f=s.follow||{}, c=s.camera||{};
 var st=f.state||'OFF'; var b=document.getElementById('fstate'); b.textContent=st; b.className='badge '+st;
 document.getElementById('farm').textContent=f.running?(f.armed?'ARMED — 真发帧控车':'DRY-RUN — 不发帧'):'控制器未运行';
 document.getElementById('fdist').textContent=(f.dist_m==null?'—':f.dist_m);
 document.getElementById('flat').textContent=(f.lateral_m==null?'—':f.lateral_m);
 document.getElementById('fspeed').textContent=(f.cmd_speed==null?'—':f.cmd_speed);
 document.getElementById('fsteer').textContent=(f.cmd_steer==null?'—':f.cmd_steer);
 document.getElementById('cfps').textContent=c.left_fps||0;document.getElementById('dfps').textContent=c.disparity_fps||0;
 var tg0=s.target||{};document.getElementById('fconf').textContent=(tg0.conf!=null?Math.round(tg0.conf*100):'—');
 var lx=c.brightness;document.getElementById('clux').textContent=(lx!=null?lx+(lx<60?' 暗':(lx>190?' 亮':'')):'—');
 var ov=document.getElementById('ovl'),oc=document.getElementById('ovlchk');
 if(oc&&oc.checked){ov.style.display='block';
  var zh={FOLLOW:'跟随',HOLD:'保持',STOP_NEAR:'太近停',SEARCH:'搜索',COAST:'滑行',STEER_ONLY:'仅转向',SEEK:'寻回',RETURN:'归位',OFF:'停止'};
  var vmax=(f.max_speed!=null?f.max_speed:0.4);
  var sp=(f.cmd_speed!=null?f.cmd_speed:null),stv=(f.cmd_steer!=null?f.cmd_steer:null),la=(f.lateral_m!=null?f.lateral_m:null);
  function bar(r,color){return '<span style="position:relative;width:96px;height:8px;background:rgba(148,163,184,.3);display:inline-block;margin-left:6px"><span style="position:absolute;left:0;width:'+Math.round(Math.max(0,Math.min(1,r))*100)+'%;height:100%;background:'+color+'"></span></span>';}
  function cbar(r,color){r=Math.max(-1,Math.min(1,r));var w=Math.abs(r)*50,left=r<0?(50-w):50;
   return '<span style="position:relative;width:96px;height:8px;background:rgba(148,163,184,.3);display:inline-block;margin-left:6px"><span style="position:absolute;left:'+left+'%;width:'+w+'%;height:100%;background:'+color+'"></span><span style="position:absolute;left:50%;top:-2px;width:1px;height:12px;background:#cbd5e1"></span></span>';}
  ov.innerHTML='<div style="font-weight:700;margin-bottom:4px">'+(zh[f.state]||'—')+(f.dist_m!=null?' · '+f.dist_m.toFixed(2)+' m':'')+'</div>'
   +'<div>速 '+(sp!=null?sp.toFixed(2):'—')+bar(sp!=null?sp/vmax:0,'#22d3ee')+'</div>'
   +'<div>转 '+(stv!=null?((stv>0?'+':'')+stv.toFixed(1)):'—')+cbar(stv!=null?stv/25:0,'#f59e0b')+'</div>'
   +'<div>偏 '+(la!=null?((la>0?'+':'')+la.toFixed(2)):'—')+cbar(la!=null?la/2:0,'#a78bfa')+'</div>';
 }else{ov.style.display='none';}
 var ctl=s.ctl||{};var sb=function(id,on,t){var e=document.getElementById(id);if(!e)return;e.textContent=t;e.className='badge '+(on?'FOLLOW':'OFF');};
 sb('cgrab',ctl.grabber,'grabber'+(ctl.grabber?' ✓':' ✕'));sb('cyolo',ctl.yolo,'yolo'+(ctl.yolo?' ✓':' ✕'));
 sb('csel',ctl.selector,'选择器'+(ctl.selector?' ✓':' ✕'));
 var tg1=s.target||{},lk=document.getElementById('clock');
 if(lk){var zhT={TRACKING:'跟踪中',REACQ:'重捕获中',LOST:'目标丢失',NEED_ENROLL:'待注册'};
  if(tg1.locked){lk.textContent='🔒 '+(zhT[tg1.track]||'已锁定')+(tg1.lock_conf!=null?' 像'+(tg1.lock_conf*100).toFixed(0)+'%':'');
   lk.className='badge '+(tg1.track==='TRACKING'?'FOLLOW':(tg1.track==='LOST'?'STOP_NEAR':'HOLD'));}
  else{lk.textContent='未锁定 (跟最显著的人)';lk.className='badge OFF';}}
 var cf=document.getElementById('cfollow');if(cf){if(ctl.follow_running){cf.textContent=ctl.armed?'ARMED ⚡':'DRY-RUN';cf.className='badge '+(ctl.armed?'STOP_NEAR':'HOLD');}else{cf.textContent='未运行';cf.className='badge OFF';}}
 var b2=s.bms||{};
 if(b2.voltage_v!=null){var soc=(b2.soc_pct==null?0:b2.soc_pct);
  document.getElementById('bsoc').textContent=soc+'%';
  var bar2=document.getElementById('bbar');bar2.style.width=soc+'%';
  bar2.style.background=b2.charging?'#2b5d8a':(soc<20?'#9c2b23':(soc<40?'#8a6a2b':'#2e6b3c'));
  document.getElementById('bdetail').textContent='SOC 为电压线性估算 (39–54.6V), 充电时略偏高';
  document.getElementById('bvolt').textContent=b2.voltage_v;
  document.getElementById('bamp').textContent=(b2.current_a>0?'+':'')+b2.current_a;
  document.getElementById('bah').textContent=b2.remaining_ah;
  var bc=document.getElementById('bchg');bc.textContent=b2.charging?'充电中':'放电';bc.className='badge '+(b2.charging?'COAST':'FOLLOW');
 }else{document.getElementById('bsoc').textContent='—';document.getElementById('bdetail').textContent='无 BMS 数据';
  ['bvolt','bamp','bah'].forEach(function(i){document.getElementById(i).textContent='—';});
  var bc0=document.getElementById('bchg');bc0.textContent='—';bc0.className='badge OFF';}
 document.getElementById('dot').style.color=rec?'#9c2b23':'#857f70';document.getElementById('stext').textContent=rec?'录制中…':'空闲';
 document.getElementById('start').disabled=rec;document.getElementById('stop').disabled=!rec;
 document.getElementById('run').textContent=s.run||'—';
 document.getElementById('frames').textContent=rec?(s.frames||0):'—';document.getElementById('depth').textContent=rec?(s.depth_frames||0):'—';
 document.getElementById('obsrows').textContent=rec?(s.obstacle_rows||0):'—';document.getElementById('elapsed').textContent=rec?(s.elapsed||0):'—';
 document.getElementById('disk').textContent=s.disk_free_gb||'—';
 var t=document.getElementById('obstab'); t.innerHTML='<tr><th>类型</th><th>距离 m</th><th>横向 m</th></tr>';
 (s.obstacles||[]).forEach(function(o){var r=t.insertRow(); r.insertCell().textContent=o.type; r.insertCell().textContent=(o.distance_m||0).toFixed(2); r.insertCell().textContent=(o.center_x_m||0).toFixed(2);});
 var dm=s.demo||{};
 document.getElementById('demostart').disabled=!!dm.recording;
 document.getElementById('demostop').disabled=!dm.recording;
 document.getElementById('demostat').textContent=dm.recording?('⏺ '+(dm.file||'启动中…')+' · '+(dm.frames||0)+'帧 · '+(dm.elapsed||0)+'s'):(dm.error?('错误: '+dm.error):'');
 var dl=document.getElementById('demolist');dl.innerHTML='';
 (s.demos||[]).forEach(function(d){var a=document.createElement('a');a.href='/demo?f='+encodeURIComponent(d.name);a.textContent=d.name+' ('+d.mb+'MB)';a.style.cssText='color:#1e3a5f;display:inline-block;margin-right:14px';dl.appendChild(a);});
 var vm=(f.max_speed!=null)?f.max_speed:((s.config||{}).max_speed);
 document.getElementById('vcur').textContent=(vm!=null)?('当前 '+vm):'当前 0.4 (默认)';
 var cfg=s.config||{};
 if(document.activeElement.id!=='mgrass')document.getElementById('mgrass').checked=!!cfg.grass;
 if(document.activeElement.id!=='mseek')document.getElementById('mseek').checked=(cfg.seek===undefined)?true:!!cfg.seek;
 drawBox(s.target||{});
}).catch(e=>{});}
function setMode(){var g=document.getElementById('mgrass').checked?1:0,k=document.getElementById('mseek').checked?1:0;
 fetch('/set_mode?grass='+g+'&seek='+k,{method:'POST'}).then(r=>r.json()).then(j=>{document.getElementById('ctlmsg').textContent=(j.msg||'');}).catch(e=>{});}
function lockConfirm(){if(confirm('把画面里最显著的那个人锁定为「主人」?\\n锁定后只跟他, 路人再近也不跟。'))go('lock');}
function drawBox(tg){var pv=document.getElementById('prev'),yb=document.getElementById('ybox'),yl=document.getElementById('ylabel'),cd=document.getElementById('cands');
 var sx=0,sy=0;
 if(tg&&tg.img_w&&pv.clientWidth>0){sx=pv.clientWidth/tg.img_w;sy=pv.clientHeight/tg.img_h;}
 if(cd){cd.innerHTML='';
  if(tg&&tg.candidates&&sx>0){tg.candidates.forEach(function(c){
   if(!c.bbox)return;
   if(tg.bbox&&Math.abs(c.bbox[0]-tg.bbox[0])<2&&Math.abs(c.bbox[1]-tg.bbox[1])<2)return;
   var d=document.createElement('div');
   d.style.cssText='position:absolute;border:1.5px dashed rgba(148,163,184,.85);left:'+(c.bbox[0]*sx)+'px;top:'+(c.bbox[1]*sy)+'px;width:'+(c.bbox[2]*sx)+'px;height:'+(c.bbox[3]*sy)+'px';
   cd.appendChild(d);
   if(c.app!=null){var t=document.createElement('div');
    t.style.cssText='position:absolute;background:rgba(100,116,139,.9);color:#fff;font-size:10px;font-weight:600;padding:0 3px;left:'+(c.bbox[0]*sx)+'px;top:'+Math.max(0,c.bbox[1]*sy-13)+'px';
    t.textContent='像 '+(c.app*100).toFixed(0)+'%';cd.appendChild(t);}
  });}}
 if(tg&&tg.bbox&&sx>0){
  var bx=tg.bbox[0]*sx,by=tg.bbox[1]*sy,bw=tg.bbox[2]*sx,bh=tg.bbox[3]*sy;
  var col=tg.locked?'#22c55e':'#30d158';
  yb.style.left=bx+'px';yb.style.top=by+'px';yb.style.width=bw+'px';yb.style.height=bh+'px';yb.style.display='block';
  yb.style.borderColor=col;yb.style.borderWidth=tg.locked?'3px':'2px';
  yl.style.left=bx+'px';yl.style.top=Math.max(0,by-15)+'px';yl.style.display='block';yl.style.background=col;
  yl.textContent=(tg.locked?'🔒 主人 ':'人 ')+(tg.dist_m!=null?tg.dist_m.toFixed(2)+'m':'')
   +(tg.conf!=null?' · '+(tg.conf*100).toFixed(0)+'%':'')
   +(tg.lock_conf!=null?' · 像'+(tg.lock_conf*100).toFixed(0)+'%':'');
 }else{yb.style.display='none';yl.style.display='none';}}
function clkTick(){var d=new Date(),p=function(x){return (x<10?'0':'')+x};
 document.getElementById('clk').textContent=d.getFullYear()+'-'+p(d.getMonth()+1)+'-'+p(d.getDate())+' '+p(d.getHours())+':'+p(d.getMinutes())+':'+p(d.getSeconds());}
document.getElementById('ovlchk').checked=(localStorage.getItem('ovl')==='1');
setInterval(tick,1000);tick();
setInterval(clkTick,1000);clkTick();
setInterval(function(){fetch('/target').then(r=>r.json()).then(drawBox).catch(e=>{});},300);
</script></body></html>"""


# ================== /slate 电子场记板(多视角 Demo 的时间戳对齐) ==================
# 用途: 手机开录前先拍这个页面 3 秒 → Mac 侧 tools/sync_phone.py 从手机视频里解出
#       「手机第 0 帧 对应的车钟时刻」→ 与 demo CSV 的 ts 列对齐 → ffmpeg 合成四宫格。
#
# 为什么显示的是车钟而不是手机/浏览器钟: 页面加载时先跟 /now 做多次往返对时(取 RTT
# 最小那次, NTP 式), 算出 offset, 之后一律显示 Date.now()+offset。这样即使拿平板/
# 笔记本打开本页, 画面上的时间也是车上那台机器的钟, 消掉最大的一个误差源。
#
# 机读编码: 洋红(255,0,255)边框圈出的灰底色条里有 16 个格子(黑=0 白=1):
#   格0=1 格1=0 (同步头) | 格2~13 = unix_ms & 0xFFF 的 12 位(高位在左) |
#   格14 = 12 位的偶校验 | 格15=1 (同步尾)
#   12 位 = 4096ms ≈ 4 秒内无歧义, 配合明文的秒即可完全还原。
#   洋红在自然场景里极罕见 → 解码器先找洋红连通域定位色条, 再等分 16 格取中心亮度。
# 人读兜底: 上方大号等宽明文时间, 自动解码失败时肉眼读一眼手输即可。
#
# ⚠ 本常量用 r-string: JS 里的 \n 保持字面量, 天然免疫 PAGE 那个「单 \n 被 Python
#   变成真换行 → JS 字符串未闭合 → 整个 script 块 SyntaxError」的事故(见 2026-07-15)。
SLATE_PAGE = r"""<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>场记板 SLATE</title><style>
html,body{margin:0;padding:0;background:#000;color:#fff;height:100%;overflow:hidden;
 font-family:ui-monospace,Menlo,Consolas,monospace;-webkit-user-select:none;user-select:none}
#wrap{height:100%;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:2vh}
/* 日期小、时刻大: 对齐只用得上时刻。字号按等宽字符宽≈0.6em 反推, 12 字符 × 0.6 × 11vw
   = 79vw < 100vw, 留足边距不会像 8.5vw 那样把年份和毫秒切掉(实测 1280 宽溢出) */
#day{font-size:3.2vw;color:#8b98a5;letter-spacing:.08em}
#clk{font-size:11vw;font-weight:700;letter-spacing:.01em;line-height:1;white-space:nowrap}
#epo{font-size:2.6vw;color:#8b98a5;letter-spacing:.04em;white-space:nowrap}
/* 洋红边框必须直接贴住灰色色条: 中间若夹一圈黑底, 解码器按比例内缩剥不干净时
   会把那圈黑当成额外的"格子"(实测切出 18 段而非 16 段 → 解码失败) */
#frame{border:1.2vh solid #f0f;padding:0;background:#808080}
#strip{display:flex;gap:.9vw;background:#808080;padding:1.2vh 1.2vw}
.cell{width:4.6vw;height:13vh;background:#000}
.on{background:#fff}
#foot{font-size:2.1vw;color:#8b98a5;text-align:center;line-height:1.5}
#sync{font-size:2.1vw}
.ok{color:#22c55e}.bad{color:#f59e0b}
</style></head><body><div id=wrap>
<div id=day>-------_--</div>
<div id=clk>--:--:--.---</div>
<div id=epo>epoch --</div>
<div id=frame><div id=strip></div></div>
<div id=sync class=bad>对时中…</div>
<div id=foot>手机横屏拍这块屏幕 3 秒, 再转身拍车<br>显示的是<b>车上那台机器</b>的钟</div>
</div><script>
var CELLS=16, cells=[], off=0, bestRtt=1e9, synced=0;
(function(){var s=document.getElementById('strip');
 for(var i=0;i<CELLS;i++){var d=document.createElement('div');d.className='cell';s.appendChild(d);cells.push(d);}})();

/* NTP 式对时: 多次往返取 RTT 最小的一次, offset = t_car - (t0+t1)/2 */
function probe(n){
  if(n<=0){done();return;}
  var t0=Date.now();
  fetch('/now',{cache:'no-store'}).then(function(r){return r.json()}).then(function(j){
    var t1=Date.now(), rtt=t1-t0;
    if(rtt<bestRtt){bestRtt=rtt; off=j.ts*1000-(t0+t1)/2; synced=1;}
    setTimeout(function(){probe(n-1)},120);
  }).catch(function(){setTimeout(function(){probe(n-1)},400)});
}
function done(){
  var e=document.getElementById('sync');
  if(synced){e.className='ok';e.textContent='已对时 offset='+off.toFixed(0)+'ms  RTT='+bestRtt+'ms';}
  else{e.className='bad';e.textContent='对时失败! 只能用明文时间人工对齐';}
}
function p(n,w){var s=''+n;while(s.length<w){s='0'+s;}return s;}
function draw(){
  var tc=Date.now()+off, d=new Date(tc), ms=Math.floor(tc)&0xFFF;
  document.getElementById('day').textContent=
    d.getFullYear()+'-'+p(d.getMonth()+1,2)+'-'+p(d.getDate(),2);
  document.getElementById('clk').textContent=
    p(d.getHours(),2)+':'+p(d.getMinutes(),2)+':'+p(d.getSeconds(),2)+'.'+p(d.getMilliseconds(),3);
  document.getElementById('epo').textContent='epoch '+Math.floor(tc)+'   code 0x'+p(ms.toString(16).toUpperCase(),3);
  var bits=[1,0], par=0;
  for(var i=11;i>=0;i--){var b=(ms>>i)&1; bits.push(b); par^=b;}
  bits.push(par); bits.push(1);
  for(var k=0;k<CELLS;k++){cells[k].className=bits[k]?'cell on':'cell';}
  requestAnimationFrame(draw);
}
probe(7); requestAnimationFrame(draw);
</script></body></html>"""


class TS(ThreadingMixIn, HTTPServer):
    daemon_threads = True


class H(BaseHTTPRequestHandler):
    def _s(self, c, ct, b):
        self.send_response(c); self.send_header('Content-Type', ct)
        self.send_header('Content-Length', str(len(b))); self.send_header('Cache-Control', 'no-store')
        self.end_headers(); self.wfile.write(b)

    def do_GET(self):
        p = self.path
        if p == '/' or p.startswith('/index'):
            self._s(200, 'text/html; charset=utf-8', PAGE.encode('utf-8'))
        elif p.startswith('/slate'):
            self._s(200, 'text/html; charset=utf-8', SLATE_PAGE.encode('utf-8'))
        elif p.startswith('/now'):
            # 极轻量对时端点: 只回车钟, 不做任何文件/子进程操作(那会给 RTT 加抖动)
            self._s(200, 'application/json', json.dumps({'ts': time.time()}).encode())
        elif p.startswith('/status'):
            self._s(200, 'application/json', json.dumps(status()).encode())
        elif p.startswith('/target'):
            self._s(200, 'application/json',
                    json.dumps(_read_json(os.path.join(RUNTIME, 'target.json'), max_age=2) or {}).encode())
        elif p.startswith('/preview'):
            data = serve_jpeg(['left_latest.jpg', 'left_latest.ppm', 'latest.ppm'])
            if data: self._s(200, 'image/jpeg', data)
            else: self._s(404, 'text/plain', b'no frame')
        elif p.startswith('/disparity'):
            data = serve_jpeg(['disparity_latest.jpg', 'disparity_latest.pgm'])
            if data: self._s(200, 'image/jpeg', data)
            else: self._s(404, 'text/plain', b'no disparity')
        elif p.startswith('/stream'):
            self._mjpeg(['left_latest.jpg', 'left_latest.ppm', 'latest.ppm'])
        elif p.startswith('/dispstream'):
            self._mjpeg(['disparity_latest.jpg', 'disparity_latest.pgm'])
        elif p.startswith('/demo'):
            self._demo_file(p)
        else:
            self._s(404, 'text/plain', b'nf')

    def _demo_file(self, p):
        """下载 demo 视频: GET /demo?f=demo_YYYYmmdd_HHMMSS.mp4 (分块流式, 别整读进内存)。"""
        q = parse_qs(urlparse(p).query)
        n = os.path.basename(q.get('f', [''])[0])
        path = os.path.join(DEMO_DIR, n)
        if not n.startswith('demo_') or not os.path.isfile(path):
            self._s(404, 'text/plain', b'no such demo'); return
        try:
            size = os.path.getsize(path)
            self.send_response(200)
            self.send_header('Content-Type', 'video/mp4' if n.endswith('.mp4') else 'video/x-msvideo')
            self.send_header('Content-Length', str(size))
            self.send_header('Content-Disposition', 'attachment; filename="%s"' % n)
            self.end_headers()
            with open(path, 'rb') as fh:
                while True:
                    chunk = fh.read(1 << 20)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
        except Exception:
            return

    def _mjpeg(self, names):
        """MJPEG 实时流(multipart/x-mixed-replace), 浏览器 <img> 当视频播。"""
        try:
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            while True:
                data = serve_jpeg(names)
                if data:
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n')
                    self.wfile.write(('Content-Length: %d\r\n\r\n' % len(data)).encode())
                    self.wfile.write(data)
                    self.wfile.write(b'\r\n')
                time.sleep(0.2)   # ~4-5 fps
        except Exception:
            return

    def do_POST(self):
        p = self.path

        def j(ok, m, key='msg'):
            self._s(200, 'application/json', json.dumps({'ok': ok, key: m}).encode())
        if p.startswith('/perception_start'):
            ok, m = perception_start(); j(ok, m)
        elif p.startswith('/perception_stop'):
            ok, m = perception_stop(); j(ok, m)
        elif p.startswith('/demo_start'):
            ok, m = demo_start(); j(ok, m)
        elif p.startswith('/demo_stop'):
            ok, m = demo_stop(); j(ok, m)
        elif p.startswith('/set_speed'):
            ok, m = set_speed(p); j(ok, m)
        elif p.startswith('/set_mode'):
            try:
                q = parse_qs(urlparse(p).query)
                cfg = {}
                if 'grass' in q: cfg['grass'] = (q['grass'][0] == '1')
                if 'seek' in q:  cfg['seek'] = (q['seek'][0] == '1')
                _write_config(cfg)
                j(True, '模式: 草地=%s 寻回=%s (热生效)' % (
                    '开' if cfg.get('grass') else '关', '开' if cfg.get('seek') else '关'))
            except Exception as e:
                j(False, '设置失败: %s' % e)
        elif p.startswith('/lock'):
            ok, m = set_lock(True); j(ok, m)
        elif p.startswith('/unlock'):
            ok, m = set_lock(False); j(ok, m)
        elif p.startswith('/follow_dry'):
            ok, m = follow_start(False); j(ok, m)
        elif p.startswith('/follow_arm'):
            ok, m = follow_start(True); j(ok, m)
        elif p.startswith('/follow_steer'):
            ok, m = follow_start(True, True); j(ok, m)
        elif p.startswith('/follow_stop'):
            ok, m = follow_stop(); j(ok, m)
        elif p.startswith('/set_dist'):
            ok, m = set_dist(p); j(ok, m)
        elif p.startswith('/start'):
            ok, m = start_recording(); j(ok, m)
        elif p.startswith('/stop'):
            ok, m = stop_recording(); j(ok, m, 'result')
        else:
            self._s(404, 'text/plain', b'nf')

    def log_message(self, *a):
        pass


def _graceful(signum, frame):
    """stop_follow.sh 用 pkill(SIGTERM) 停面板: 先收尾 demo/录制再退, 防 mp4 缺 moov 损坏。"""
    try: demo_stop()
    except Exception: pass
    try: stop_recording()
    except Exception: pass
    os._exit(0)


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, _graceful)
    for d in (HOST_DS, RUNTIME, DEMO_DIR):
        try: os.makedirs(d)
        except OSError: pass
    # 自动拉起 BMS 电量监控(写 runtime/bms.json)
    try:
        subprocess.Popen(['python3', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'bms_monitor.py')],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as e:
        sys.stderr.write('bms_monitor launch failed: %s\n' % e)
    httpd = TS(('0.0.0.0', PORT), H)
    sys.stderr.write('web_ui (file-based) on 0.0.0.0:%d  grab_out=%s\n' % (PORT, GRAB_OUT)); sys.stderr.flush()
    httpd.serve_forever()
