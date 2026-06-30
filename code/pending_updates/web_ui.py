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
import os, sys, time, json, shutil, subprocess, threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

HOST_ROOT = '/home/nvidia/work/AutoApollo/apollo/follow_data'
HOST_DS   = HOST_ROOT + '/dataset'
RUNTIME   = HOST_ROOT + '/runtime'
BIN       = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = RUNTIME + '/follow_config.json'   # 保持距离(web 写, follow_controller 热读)
# 默认看我们自己 grabber 的输出(含真实米深度), 不再是 fr07 的
GRAB_OUT  = os.environ.get('GRAB_OUT', RUNTIME + '/grab')
PORT      = int(os.environ.get('FOLLOW_WEB_PORT', '8080'))
REC_FPS   = 5.0
CONTAINER = os.environ.get('APOLLO_CONTAINER', 'apollo_dev_nvidia')

# 容器内感知进程启动命令(grabber 写 grab/, yolo_follow 检人写 target.json)
# 帧率提到 grabber@9fps / yolo@15hz: 喂控制器更连续, 跟随更跟手(相机本身 ~9.5fps)。
GRAB_CMD = ("docker exec -d %s bash -c 'cd /apollo/follow_data/zkhy_grab && "
            "LD_LIBRARY_PATH=/apollo/follow_data/lib:/apollo/modules/drivers/zkhy/src/Bin "
            "./zkhy_grabber --out-dir /apollo/follow_data/runtime/grab --duration 0 --write-fps 9 "
            "> /tmp/grab.log 2>&1'") % CONTAINER
YOLO_CMD = ("docker exec -d %s bash -c 'cd /apollo/follow_data/trtx/build && "
            "LD_LIBRARY_PATH=/apollo/follow_data/trtx/build:/usr/lib/aarch64-linux-gnu/tegra:/usr/local/cuda-10.0/lib64 "
            "./yolo_follow --engine yolov5s.engine --grab-dir /apollo/follow_data/runtime/grab "
            "--runtime /apollo/follow_data/runtime --hz 15 > /tmp/yolo_follow.log 2>&1'") % CONTAINER

S = {'recording': False, 'run': None, 'host_run': None, 'start_ts': 0,
     'can_proc': None, 'rec_thread': None, 'rec_stop': None,
     'frames': 0, 'obs_rows': 0, 'depth': 0,
     'follow_proc': None, 'follow_armed': False}
lock = threading.Lock()


def _sh(cmd, timeout=20):
    try:
        p = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, timeout=timeout)
        return p.returncode, p.stdout.decode('utf-8', 'ignore')
    except Exception as e:
        return -1, str(e)


def perception_start():
    """容器内拉起 grabber + yolo_follow(各自若没在跑才起)。"""
    if _sh("docker exec %s pgrep -x zkhy_grabber" % CONTAINER)[0] != 0:
        _sh(GRAB_CMD)
    if _sh("docker exec %s pgrep -x yolo_follow" % CONTAINER)[0] != 0:
        _sh(YOLO_CMD)
    return True, '感知已启动(grabber+yolo_follow)'


def perception_stop():
    _sh("docker exec %s pkill -x yolo_follow" % CONTAINER)
    _sh("docker exec %s pkill -x zkhy_grabber" % CONTAINER)
    return True, '感知已停止'


def follow_start(arm, steer_only=False):
    """宿主机拉起 follow_controller。arm=真发帧; steer_only=只转向不前进(速度锁0)。"""
    with lock:
        p = S.get('follow_proc')
        if p and p.poll() is None:
            return False, '跟随控制器已在运行, 先停止'
        cmd = [sys.executable, os.path.join(BIN, 'follow_controller.py')]
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


def set_dist(path):
    """从 /set_dist?min=&max= 写 follow_config.json, follow_controller 热读生效。"""
    try:
        from urllib.parse import urlparse, parse_qs
        q = parse_qs(urlparse(path).query)
        mn = float(q.get('min', ['0'])[0]); mx = float(q.get('max', ['0'])[0])
        if not (0.3 < mn < mx < 20):
            return False, '非法: 需 0.3 < 近 < 远 < 20'
        tmp = CONFIG_FILE + '.tmp'
        json.dump({'desired_min': mn, 'desired_max': mx}, open(tmp, 'w'))
        os.replace(tmp, CONFIG_FILE)
        return True, '保持距离设为 %.1f~%.1f m (热生效)' % (mn, mx)
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
                   'obstacle_fps': round(cam.get('obstacle_fps', 0), 1) if cam else 0}
    obs = _read_json(os.path.join(GRAB_OUT, 'obstacles_latest.json'), max_age=5)
    s['obstacles'] = (obs or {}).get('obstacles', []) if obs else []
    # 跟随状态
    fol = _read_json(os.path.join(RUNTIME, 'follow_status.json'), max_age=3)
    s['follow'] = fol if fol else {'state': 'OFF', 'running': False}
    if fol: s['follow']['running'] = True
    s['bms'] = _read_json(os.path.join(RUNTIME, 'bms.json'), max_age=6) or {}
    # 控制进程状态(从文件新鲜度推断, 不每秒 docker exec)
    tgt = _read_json(os.path.join(RUNTIME, 'target.json'), max_age=3)
    fp = S.get('follow_proc')
    s['ctl'] = {'grabber': bool(s['camera']['online']),
                'yolo': tgt is not None,
                'follow_running': bool(fp and fp.poll() is None),
                'armed': bool(S.get('follow_armed'))}
    s['target'] = tgt or {}
    try:
        st = os.statvfs(HOST_DS); s['disk_free_gb'] = round(st.f_bavail * st.f_frsize / 1e9, 1)
    except Exception: pass
    return s


def serve_jpeg(names, resize='720x'):
    """返回 JPEG 字节。浏览器不认 PPM/PGM, 故用 ImageMagick convert 实时转 JPEG。"""
    for n in names:
        p = os.path.join(GRAB_OUT, n)
        if not os.path.exists(p):
            continue
        if p.endswith('.jpg') or p.endswith('.jpeg'):
            try:
                return open(p, 'rb').read()
            except Exception:
                continue
        try:  # ppm/pgm -> jpeg, 顺便缩到 720 宽减小体积
            r = subprocess.run(['convert', p, '-resize', resize, 'jpg:-'],
                               stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, timeout=6)
            if r.returncode == 0 and r.stdout:
                return r.stdout
        except Exception:
            pass
    return None


PAGE = u"""<!doctype html><html><head><meta charset=utf-8><title>跟随 · 采集 + 状态</title>
<meta name=viewport content="width=device-width,initial-scale=1"><style>
body{background:#111;color:#eee;font-family:-apple-system,Helvetica,Arial,sans-serif;margin:0;padding:16px}
h1{font-size:19px;margin:0 0 12px}.card{background:#1c1c1e;border-radius:12px;padding:14px;margin-bottom:12px}
.k{color:#8e8e93;font-size:12px}.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:9px}
.kv{background:#262629;border-radius:8px;padding:9px}.kv .v{font-size:19px;font-weight:600;margin-top:2px}
button{border:0;border-radius:10px;padding:14px 20px;font-size:17px;font-weight:700;color:#fff;cursor:pointer}
#start{background:#30d158}#stop{background:#ff453a}button:disabled{opacity:.4}
.badge{display:inline-block;padding:3px 10px;border-radius:7px;font-weight:700;font-size:15px}
.FOLLOW{background:#30d158}.HOLD{background:#ffd60a;color:#000}.STOP_NEAR{background:#ff453a}.SEARCH{background:#5e5ce6}.OFF{background:#48484a}
img{width:100%;border-radius:9px;background:#000;display:block}
table{width:100%;border-collapse:collapse;font-size:13px}td,th{text-align:left;padding:3px 6px;border-bottom:1px solid #333}
</style></head><body><h1>跟随行人 · 数据采集 + 实时状态</h1>

<div class=card><div class=k style=margin-bottom:8px>电量 / 电池</div>
<div style="display:flex;align-items:center;gap:14px">
<div style="font-size:30px;font-weight:700;min-width:70px" id=bsoc>—</div>
<div style="flex:1">
<div style="background:#262629;border-radius:6px;height:16px;overflow:hidden"><div id=bbar style="height:100%;width:0%;background:#30d158;transition:width .4s"></div></div>
<div class=k id=bdetail style=margin-top:5px>—</div></div></div></div>

<div class=card><div class=k style=margin-bottom:8px>小车跟随人 · 实时状态</div>
<div style="font-size:15px;margin-bottom:10px">状态 <span id=fstate class="badge OFF">OFF</span>
<span id=farm style="margin-left:10px;color:#8e8e93;font-size:13px"></span></div>
<div class=grid>
<div class=kv><div class=k>目标距离 m (保持2-4)</div><div class=v id=fdist>—</div></div>
<div class=kv><div class=k>横向偏移 m (右+)</div><div class=v id=flat>—</div></div>
<div class=kv><div class=k>下发车速 m/s</div><div class=v id=fspeed>—</div></div>
<div class=kv><div class=k>下发转向 °</div><div class=v id=fsteer>—</div></div>
<div class=kv><div class=k>相机 fps</div><div class=v id=cfps>—</div></div>
<div class=kv><div class=k>视差 fps</div><div class=v id=dfps>—</div></div>
</div></div>

<div class=card><div class=k style=margin-bottom:8px>控制台</div>
<div style="margin-bottom:7px;font-size:13px">感知 <span id=cgrab class="badge OFF">grabber</span> <span id=cyolo class="badge OFF">yolo</span></div>
<button id=pson onclick="go('perception_start')" style="background:#0a84ff;padding:11px 16px;font-size:15px">▶ 启动感知</button>
<button id=psoff onclick="go('perception_stop')" style="background:#48484a;padding:11px 16px;font-size:15px">■ 停止感知</button>
<div style="margin:13px 0 7px;font-size:13px">跟随控制 <span id=cfollow class="badge OFF">未运行</span></div>
<button id=fdry onclick="go('follow_dry')" style="background:#30d158;padding:11px 16px;font-size:15px">▶ DRY-RUN</button>
<button id=fsteer onclick="steerConfirm()" style="background:#ffd60a;color:#000;padding:11px 16px;font-size:15px">↔ 转向跟随(不前进)</button>
<button id=farmbtn onclick="armConfirm()" style="background:#ff9f0a;padding:11px 16px;font-size:15px">⚡ ARM 前进+转向</button>
<button id=fstopbtn onclick="go('follow_stop')" style="background:#ff453a;padding:11px 16px;font-size:15px">■ 停止/急停</button>
<div style="margin:13px 0 7px;font-size:13px">保持距离 m (热生效)</div>
<input id=dmin type=number step=0.5 min=0.5 value=2 style="width:64px;font-size:15px;padding:7px;border-radius:8px;border:1px solid #444;background:#262629;color:#eee"> ~
<input id=dmax type=number step=0.5 value=4 style="width:64px;font-size:15px;padding:7px;border-radius:8px;border:1px solid #444;background:#262629;color:#eee">
<button onclick="setDist()" style="background:#5e5ce6;padding:9px 16px;font-size:15px">设置</button>
<div class=k id=ctlmsg style=margin-top:8px>ARM 前务必: 车轮架空 / 充电枪拔出 / 急停释放</div></div>

<div class=card><div style="margin-bottom:10px"><span id=dot style="color:#8e8e93">●</span> <span id=stext>空闲</span></div>
<button id=start onclick="go('start')">▶ 开始记录</button>
<button id=stop onclick="go('stop')" disabled>■ 结束记录</button>
<div class=k id=msg style=margin-top:8px></div>
<div class=grid style=margin-top:10px>
<div class=kv><div class=k>run</div><div class=v id=run>—</div></div>
<div class=kv><div class=k>图像帧</div><div class=v id=frames>—</div></div>
<div class=kv><div class=k>视差帧</div><div class=v id=depth>—</div></div>
<div class=kv><div class=k>障碍物行</div><div class=v id=obsrows>—</div></div>
<div class=kv><div class=k>时长 s</div><div class=v id=elapsed>—</div></div>
<div class=kv><div class=k>磁盘 GB</div><div class=v id=disk>—</div></div>
</div></div>

<div class=card><div class=k style=margin-bottom:8px>当前障碍物 (相机自带检测)</div>
<table id=obstab><tr><th>类型</th><th>距离 m</th><th>横向 m</th></tr></table></div>

<div class=card><div class=grid style="grid-template-columns:1fr 1fr">
<div><div class=k style=margin-bottom:6px>左目 (实时 + YOLO框)</div>
<div id=prevwrap style="position:relative;line-height:0">
<img id=prev src="/stream" style="width:100%;border-radius:9px;background:#000;display:block">
<div id=ybox style="position:absolute;border:2px solid #30d158;box-shadow:0 0 0 1px rgba(0,0,0,.6);display:none;pointer-events:none"></div>
<div id=ylabel style="position:absolute;background:#30d158;color:#000;font-size:11px;font-weight:700;padding:1px 5px;border-radius:3px;display:none;pointer-events:none;white-space:nowrap"></div>
</div></div>
<div><div class=k style=margin-bottom:6px>视差 (实时)</div><img id=disp src="/dispstream"></div>
</div></div>

<script>
function go(a){fetch('/'+a,{method:'POST'}).then(r=>r.json()).then(j=>{document.getElementById('ctlmsg').textContent=(j.msg||j.result||JSON.stringify(j));tick();}).catch(e=>{document.getElementById('ctlmsg').textContent=''+e;tick();});}
function armConfirm(){if(confirm('确认 ARM 真发帧控车?\\n确认: 车轮已架空 / 充电枪已拔 / 急停已释放?'))go('follow_arm');}
function setDist(){var mn=document.getElementById('dmin').value,mx=document.getElementById('dmax').value;
 fetch('/set_dist?min='+mn+'&max='+mx,{method:'POST'}).then(r=>r.json()).then(j=>{document.getElementById('ctlmsg').textContent=(j.msg||JSON.stringify(j));tick();}).catch(e=>{document.getElementById('ctlmsg').textContent=''+e;});}
function steerConfirm(){if(confirm('只转向不前进(车不会走, 速度锁0)。人站相机前测转向追人。开始?'))go('follow_steer');}
function tick(){fetch('/status').then(r=>r.json()).then(s=>{
 var rec=s.recording, f=s.follow||{}, c=s.camera||{};
 var st=f.state||'OFF'; var b=document.getElementById('fstate'); b.textContent=st; b.className='badge '+st;
 document.getElementById('farm').textContent=f.running?(f.armed?'ARMED 真发帧':'dry-run 不发帧'):'控制器未运行';
 document.getElementById('fdist').textContent=(f.dist_m==null?'—':f.dist_m);
 document.getElementById('flat').textContent=(f.lateral_m==null?'—':f.lateral_m);
 document.getElementById('fspeed').textContent=(f.cmd_speed==null?'—':f.cmd_speed);
 document.getElementById('fsteer').textContent=(f.cmd_steer==null?'—':f.cmd_steer);
 document.getElementById('cfps').textContent=c.left_fps||0;document.getElementById('dfps').textContent=c.disparity_fps||0;
 var ctl=s.ctl||{};var sb=function(id,on,t){var e=document.getElementById(id);if(!e)return;e.textContent=t;e.className='badge '+(on?'FOLLOW':'OFF');};
 sb('cgrab',ctl.grabber,'grabber'+(ctl.grabber?' ✓':' ✕'));sb('cyolo',ctl.yolo,'yolo'+(ctl.yolo?' ✓':' ✕'));
 var cf=document.getElementById('cfollow');if(cf){if(ctl.follow_running){cf.textContent=ctl.armed?'ARMED ⚡':'dry-run';cf.className='badge '+(ctl.armed?'STOP_NEAR':'HOLD');}else{cf.textContent='未运行';cf.className='badge OFF';}}
 var b=s.bms||{};
 if(b.voltage_v!=null){var soc=(b.soc_pct==null?0:b.soc_pct);
  document.getElementById('bsoc').textContent=soc+'%';
  var bar=document.getElementById('bbar');bar.style.width=soc+'%';
  bar.style.background=b.charging?'#0a84ff':(soc<20?'#ff453a':(soc<40?'#ffd60a':'#30d158'));
  document.getElementById('bdetail').textContent=b.voltage_v+'V · '+b.current_a+'A · '+b.remaining_ah+'Ah'+(b.charging?' · 充电中 ⚡':'');
 }else{document.getElementById('bsoc').textContent='—';document.getElementById('bdetail').textContent='无 BMS 数据';}
 document.getElementById('dot').style.color=rec?'#ff453a':'#8e8e93';document.getElementById('stext').textContent=rec?'录制中…':'空闲';
 document.getElementById('start').disabled=rec;document.getElementById('stop').disabled=!rec;
 document.getElementById('run').textContent=s.run||'—';
 document.getElementById('frames').textContent=rec?(s.frames||0):'—';document.getElementById('depth').textContent=rec?(s.depth_frames||0):'—';
 document.getElementById('obsrows').textContent=rec?(s.obstacle_rows||0):'—';document.getElementById('elapsed').textContent=rec?(s.elapsed||0):'—';
 document.getElementById('disk').textContent=s.disk_free_gb||'—';
 var t=document.getElementById('obstab'); t.innerHTML='<tr><th>类型</th><th>距离 m</th><th>横向 m</th></tr>';
 (s.obstacles||[]).forEach(function(o){var r=t.insertRow(); r.insertCell().textContent=o.type; r.insertCell().textContent=(o.distance_m||0).toFixed(2); r.insertCell().textContent=(o.center_x_m||0).toFixed(2);});
 drawBox(s.target||{});
}).catch(e=>{});}
function drawBox(tg){var pv=document.getElementById('prev'),yb=document.getElementById('ybox'),yl=document.getElementById('ylabel');
 if(tg&&tg.bbox&&tg.img_w&&pv.clientWidth>0){var sx=pv.clientWidth/tg.img_w,sy=pv.clientHeight/tg.img_h;
  var bx=tg.bbox[0]*sx,by=tg.bbox[1]*sy,bw=tg.bbox[2]*sx,bh=tg.bbox[3]*sy;
  yb.style.left=bx+'px';yb.style.top=by+'px';yb.style.width=bw+'px';yb.style.height=bh+'px';yb.style.display='block';
  yl.style.left=bx+'px';yl.style.top=Math.max(0,by-15)+'px';yl.style.display='block';
  yl.textContent='人 '+(tg.dist_m!=null?tg.dist_m.toFixed(2)+'m':(tg.conf!=null?(tg.conf*100).toFixed(0)+'%':''));
 }else{yb.style.display='none';yl.style.display='none';}}
setInterval(tick,1000);tick();
setInterval(function(){fetch('/target').then(r=>r.json()).then(drawBox).catch(e=>{});},300);
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
        else:
            self._s(404, 'text/plain', b'nf')

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


if __name__ == '__main__':
    for d in (HOST_DS, RUNTIME):
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
