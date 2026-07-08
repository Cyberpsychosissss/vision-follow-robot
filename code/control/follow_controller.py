#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
follow_controller.py — 跟随控制器: 读 target.json → 控制律(与人保持 3~4m) → car_control 发 CAN → 写 follow_status.json

数据流(共享文件 IPC):
  目标源(容器内, 二选一):
    - parse_zkhy_obs.py --publish   (相机障碍物: 给真实米距 dist_m + 横向米 lateral_m, 推荐)
    - yolo_person.py    --publish   (YOLO: 给归一 off_x + 框高 box_h_norm, 无真实距离)
        都把目标写到 RUNTIME/target.json
  本控制器(宿主机):  读 target.json → 算(speed,steer) → car_control.CarController(can0) → 写 follow_status.json
  web_ui.py(宿主机): 读 follow_status.json + target.json → 显示跟随面板

控制律(保持 3~4m):
  距离(speed): 有真实米距时 —— dist>4m: 前进, 越远越快(夹硬限速)=FOLLOW; 3~4m: 停=HOLD; <3m: 停(太近)=STOP。
               只有 YOLO 框高时 —— 用 box_h_norm 当距离代理(需标定, 不准, 仅兜底)。
  转向(steer): 有横向米时 steer=atan2(lateral,dist) 的真实方位角×增益; 否则 steer=off_x×增益。死区防抖。
  目标丢失/数据过期 → 停 + SEARCH。

安全: 复用 car_control 的硬限速 + 死人开关; 默认 dry-run(不发帧)。--arm 才真发(需车轮架空+人在场)。
"""
from __future__ import print_function
import os
import sys
import time
import json
import math
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from car_control import (CarController, MAX_SPEED, MAX_STEER, ABS_MAX_SPEED,
                         GEAR_DRIVE, GEAR_REVERSE)

# ---- 共享文件路径(宿主机视角; 容器内目标源写的是 /apollo/follow_data/runtime/, 同一物理目录) ----
RUNTIME = os.environ.get("FOLLOW_RUNTIME",
                         "/home/nvidia/work/AutoApollo/apollo/follow_data/runtime")
TARGET_FILE = os.path.join(RUNTIME, "target.json")
STATUS_FILE = os.path.join(RUNTIME, "follow_status.json")
CONFIG_FILE = os.path.join(RUNTIME, "follow_config.json")   # web 写保持距离, 运行时热读

# ---- 跟随参数(可调) ----
DESIRED_MIN = 2.0          # m  保持距离下限(更近就停)
DESIRED_MAX = 4.0          # m  保持距离上限(更远就追)
KV = 0.4                   # 速度增益: speed = KV*(dist-中心), 越远越快(夹硬限速)
KP_BEARING = 1.5           # 转向增益(真实方位角, 度→度)
KSTEER_OFFX = 60.0         # 转向增益(YOLO 归一 off_x → 度)
STEER_DEADZONE_DEG = 1.5   # 方位角死区(度), 内不转
TARGET_TIMEOUT = 0.6       # s  target.json 超过这么旧就当丢目标(5fps下留~3帧余量)
LOST_HOLD = 0.0            # 丢目标时车速(0=停)
LOOP_HZ = 40.0             # 控制频率。信息瓶颈在相机~9fps, 提这个只降低指令延迟+让斜坡更细腻;
                           # CAN 发帧恒 50Hz(fr 协议 20ms 周期, 在 car_control 里, 别动)

# ---- 平滑参数(解决"一冲一停"顿挫, 核心改动) ----
# ① 连续斜坡(取代死区 bang-bang): 不再"4m内全停", 而是向中心距收敛, 速度随距离平滑增减
DIST_CENTER  = (DESIRED_MIN + DESIRED_MAX) / 2.0  # m  目标保持在中心(=3.0), 自然落在 2~4 带内
HOLD_BAND    = 0.30        # m  中心±此值内不前进(防在设定点反复抽动)
# ② 一阶低通(EMA): 压视差距离/横向的逐帧抖动. 越小越平滑但越迟钝
#    只在 target.json 出新样本时滤一次(与 LOOP_HZ 解耦); 0.60 ≈ 旧版 0.40@20Hz 的等效手感
DIST_EMA     = 0.60        # 新值权重(0~1): dist_filt = 0.6*新 + 0.4*旧
LAT_EMA      = 0.60
# ③ slew 限幅: 用物理单位定义(不随 LOOP_HZ 变), 每周期步长运行时换算
ACCEL_UP     = 0.9         # m/s²  加速上限(草地阻力大, 起步果断些; 0→0.5 约 0.56s)
ACCEL_DOWN   = 2.5         # m/s²  减速上限(安全优先, 停得更快; 上限提到2.2后刹车同步加强)
STEER_RATE   = 80.0        # °/s   转向变化率上限
MAX_DSPEED_UP   = ACCEL_UP / LOOP_HZ      # m/s 每周期
MAX_DSPEED_DOWN = ACCEL_DOWN / LOOP_HZ
MAX_DSTEER      = STEER_RATE / LOOP_HZ    # 度 每周期
# ④ 丢帧宽限: 短暂丢目标时保持上次转向+缓减速, 别立刻急停打嗝
LOST_GRACE   = 0.40        # s  这段时间内按"滑行"处理, 超过才 SEARCH 硬停

# YOLO 兜底(无真实距离)用框高代理距离, 需按实测标定:
BH_FAR = 0.45              # 归一框高 < 此值 → 人远 → 前进
BH_NEAR = 0.75            # 归一框高 > 此值 → 人近 → 停

# ---- 近距防撞门 + 远距限速 + 转向增益调度(07-07 实车两问题修复) ----
# 问题①: 人贴近车头时, 双目在人身上大量无效 → 框内深度中位数采到"背景" → dist 虚大(实测人~1m报2.9m),
#         且 YOLO 常只框到腿/半身, 框高也失真 → 车带速度朝人开。用人体尺寸的硬几何给 dist 加上界。
PERSON_H_M  = 1.65         # 成人身高假设(像高↔距离的几何校验)
PERSON_W_M  = 0.45         # 肩宽假设(框宽推距离)
FOCUS_PX    = 1065.0       # 焦距 px(与 grabber 标定一致)
NEAR_LEGS_D = 1.2          # m  只框到腿(框底贴边+框顶在画面中下) → 人就在车头, 距离上界
NEAR_WIDE   = 0.40         # 框宽占画宽 ≥ 此值 → 按人宽推距离上界
SIZE_GATE_K = 1.5          # 完整人框时: 视差距离 > K×像高距离 → 视差虚大, 用像高距离
# 问题②: 远处高速时左右画龙——横摆响应∝车速, 常数转向增益在 1.3m/s 时环路增益×3 → 振荡。
STEER_V_REF = 0.4          # m/s 转向增益随速度衰减: kp_eff = KP/(1+v/REF)
COAST_STEER_DECAY = 0.90   # COAST 期转向每周期衰减(丢目标不再握死上次舵角画大弧)
FAR_LIMIT   = 8.0          # m  目标太远大概率误锁路人 → 只许爬行接近
FAR_CREEP   = 0.3          # m/s
# ---- 最低驱动速度(0<v<它 → 抬到它): 面板「草地模式」开关热切换 ----
MIN_DRIVE_GRASS = 0.5      # m/s  草地: 低于它电机推不过草的阻力
MIN_DRIVE_HARD  = 0.15     # m/s  硬地: 保留一点下限防蠕行, 顺滑优先
MIN_DRIVE = MIN_DRIVE_HARD # 生效值(apply_config 按 grass 开关热改)

# ---- 丢目标寻回 SEEK / 倒车归位 RETURN(P1, 面板「丢失寻回」开关) ----
SEEK_ON      = True        # 总开关(热配置 seek)
SEEK_SPEED   = 0.4         # m/s  小速度……
SEEK_STEER   = 25.0        # deg  ……打满方向盘, 朝丢失侧走弧线找人
SEEK_TIME    = 4.0         # s    最长盲走时间(0.4m/s × 4s = 1.6m 弧长上限)
SEEK_MIN_OFF = 0.10        # 丢失前 |off_x| 至少这么大才认为是"侧向出画", 值得寻回
SEEK_OBS_D   = 1.5         # m    寻回路上前方出现障碍(<此距离)→ 立即放弃转入归位
RETURN_PAUSE = 0.6         # s    倒车归位前后停稳时间(换挡互锁需要静止)
OBSTACLES_FILE = os.path.join(RUNTIME, "grab", "obstacles_latest.json")


def near_gate(t):
    """近距防撞门: 用人体尺寸的硬几何给 dist_m 一个上界(视差超近会虚大, 骗不了像素尺寸)。
    就地收紧 t['dist_m'], 返回生效的上界; 没触发返回 None。"""
    bb = t.get("bbox")
    if not bb or len(bb) < 4:
        return None
    ih = float(t.get("img_h") or 720)
    iw = float(t.get("img_w") or 1280)
    top, w_px, h_px = float(bb[1]), float(bb[2]), float(bb[3])
    bot = top + h_px
    top_tr, bot_tr = top <= 3, bot >= ih - 4
    bounds = []
    if top_tr and bot_tr:                        # 整人上下都出框 → Z < f·H/画面高(≈2.4m)
        bounds.append(FOCUS_PX * PERSON_H_M / ih)
    elif bot_tr and top > 0.25 * ih:             # 只看到腿 → 人贴在车头
        bounds.append(NEAR_LEGS_D)
    if w_px / iw >= NEAR_WIDE:                   # 半身横向充满 → 按人宽推距离
        bounds.append(FOCUS_PX * PERSON_W_M / w_px)
    dist = t.get("dist_m")
    if not bounds and dist is not None and not top_tr and not bot_tr and h_px > 30:
        d_size = FOCUS_PX * PERSON_H_M / h_px    # 完整人框: 像高↔距离一致性
        if d_size < 3.5 and dist > SIZE_GATE_K * d_size:
            bounds.append(d_size)
    if not bounds:
        return None
    b = min(bounds)
    if dist is None or dist > b:
        t["dist_m"] = b
    return b


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def _ema(old, new, alpha):
    """一阶低通: 压逐帧抖动。old 为 None 时直接取 new。"""
    return new if old is None else (alpha * new + (1.0 - alpha) * old)


def _slew(prev, target, up, down):
    """限幅: 把 target 相对 prev 的变化夹到 [-down, +up], 防一步到位的顿挫。"""
    d = target - prev
    if d > up:
        d = up
    elif d < -down:
        d = -down
    return prev + d


def read_config():
    """读 runtime/follow_config.json(web 面板写, 热生效)。返回 dict, 没有/坏了返回空 dict。"""
    try:
        c = json.load(open(CONFIG_FILE))
        return c if isinstance(c, dict) else {}
    except Exception:
        return {}


def apply_config(cfg, ctl):
    """把面板配置热应用到控制律: 保持距离 + 最高速度(≤车控天花板 1.5)。"""
    try:
        mn, mx = float(cfg['desired_min']), float(cfg['desired_max'])
        if 0.3 < mn < mx < 20:
            globals()['DESIRED_MIN'], globals()['DESIRED_MAX'] = mn, mx
            globals()['DIST_CENTER'] = (mn + mx) / 2.0
    except Exception:
        pass
    try:
        ms = float(cfg['max_speed'])
        if 0.05 <= ms <= ABS_MAX_SPEED:
            globals()['MAX_SPEED'] = ms       # compute_cmd 的上限
            ctl.max_speed = ms                # car_control 编码层的硬限(仍受 ABS_MAX_SPEED 兜底)
    except Exception:
        pass
    try:                                      # 草地模式: 切最低驱动速度
        globals()['MIN_DRIVE'] = MIN_DRIVE_GRASS if cfg.get('grass') else MIN_DRIVE_HARD
    except Exception:
        pass
    try:                                      # 丢失寻回开关
        globals()['SEEK_ON'] = bool(cfg.get('seek', True))
    except Exception:
        pass


_obs_cache = {"m": None, "close": False}


def obstacle_ahead():
    """SEEK 盲走保险: 相机自带障碍物列表里, 正前方 SEEK_OBS_D 内有东西 → True。"""
    try:
        m = os.path.getmtime(OBSTACLES_FILE)
        if m != _obs_cache["m"]:
            _obs_cache["m"] = m
            close = False
            data = json.load(open(OBSTACLES_FILE))
            for o in (data or {}).get("obstacles", []):
                d, cx = o.get("distance_m"), (o.get("center_x_m") or 0.0)
                if d is not None and d < SEEK_OBS_D and abs(cx) < 1.2:
                    close = True
                    break
            _obs_cache["close"] = close
        return _obs_cache["close"]
    except Exception:
        return False


def read_target():
    try:
        with open(TARGET_FILE) as f:
            t = json.load(f)
    except Exception:
        return None
    if (time.time() - t.get("ts", 0)) > TARGET_TIMEOUT or not t.get("valid"):
        return None
    return t


def compute_cmd(t, cur_speed=0.0):
    """返回 (speed_mps, steer_ctrl_deg, state)。steer 用控制器约定(正=右), car_control 内部会取负成 fr。
    cur_speed = 当前下发车速, 用于转向增益调度(横摆响应∝车速, 高速必须降增益否则画龙)。"""
    if t is None:
        return 0.0, 0.0, "SEARCH"

    dist = t.get("dist_m")
    lateral = t.get("lateral_m")
    off_x = t.get("off_x")
    box_h = t.get("box_h_norm")

    # ---- 转向(增益随速度衰减 + 软死区) ----
    kv_gain = 1.0 / (1.0 + max(0.0, cur_speed) / STEER_V_REF)
    if lateral is not None and dist is not None and dist > 0.1:
        bearing_deg = math.degrees(math.atan2(lateral, dist))   # 正=人在右
        if abs(bearing_deg) < STEER_DEADZONE_DEG:
            steer = 0.0
        else:   # 软死区: 出死区连续起步不跳变(硬死区在中线附近会来回打)
            steer = KP_BEARING * kv_gain * (bearing_deg - math.copysign(STEER_DEADZONE_DEG, bearing_deg))
    elif off_x is not None:
        steer = 0.0 if abs(off_x) < 0.06 else KSTEER_OFFX * kv_gain * (off_x - math.copysign(0.06, off_x))
    else:
        steer = 0.0
    steer = _clamp(steer, -MAX_STEER, MAX_STEER)

    # ---- 速度(连续斜坡, 收敛到中心距, 取代 bang-bang 死区) ----
    # 旧版: 2~4m 内速度一律0 → 人匀速走时车冲到4m急停再冲, 一冲一停。
    # 新版: 向中心距(=3m)收敛, 速度随"超出中心的量"平滑增减, 只在中心附近窄带内停。
    center = DIST_CENTER
    if dist is not None and dist > 0.1:
        if dist < DESIRED_MIN:
            speed = 0.0; state = "STOP_NEAR"            # 太近: 硬停(车不能倒)
        elif dist <= center + HOLD_BAND:
            speed = 0.0; state = "HOLD"                 # 中心窄带内: 停稳(防抽动)
        else:
            speed = KV * (dist - center); state = "FOLLOW"  # 超出: 越远越快, 平滑收敛
    elif box_h is not None:                 # YOLO 兜底: 框高代理距离
        if box_h < BH_FAR:
            speed = KV * 1.0; state = "FOLLOW"
        elif box_h > BH_NEAR:
            speed = 0.0; state = "STOP_NEAR"
        else:
            speed = 0.0; state = "HOLD"
    else:
        speed = 0.0; state = "HOLD"

    speed = _clamp(speed, 0.0, MAX_SPEED)
    if 0.0 < speed < MIN_DRIVE:
        speed = MIN_DRIVE        # 草地: 微小速度推不动车, 要走就给足最低驱动速度
    if dist is not None and dist > FAR_LIMIT and speed > FAR_CREEP:
        speed = FAR_CREEP        # 目标太远大概率误锁路人 → 只爬行接近, 不满速冲
    return speed, steer, state


def write_status(d):
    try:
        tmp = STATUS_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(d, f)
        os.replace(tmp, STATUS_FILE)
    except Exception:
        pass


def main():
    ap = argparse.ArgumentParser(description="跟随控制器(保持3-4m)")
    ap.add_argument("--arm", action="store_true", help="真发帧(默认dry-run)。车轮架空+人在场!")
    ap.add_argument("--steer-only", action="store_true", help="只转向不前进(速度锁0): 最安全的实车测试, 也可确认转向符号")
    ap.add_argument("--max-speed", type=float, default=MAX_SPEED)
    ap.add_argument("--channel", default="can0")
    args = ap.parse_args()

    try:
        os.makedirs(RUNTIME)
    except OSError:
        pass

    ctl = CarController(channel=args.channel, dry_run=not args.arm,
                        max_speed=args.max_speed, verbose=False)
    if args.arm:
        mode = "只转向(不前进, 速度锁0)" if args.steer_only else "前进+转向"
        print("⚠ ARMED 跟随[%s]: 真发帧到 %s, 保持 %.1f~%.1fm。%s 3秒后开始, Ctrl-C停。"
              % (mode, args.channel, DESIRED_MIN, DESIRED_MAX,
                 "人站相机前即可。" if args.steer_only else "车轮架空+人在场!"))
        time.sleep(3)
    else:
        print("DRY-RUN 跟随(不发帧): 保持 %.1f~%.1fm。读 %s" % (DESIRED_MIN, DESIRED_MAX, TARGET_FILE))
    ctl.start()

    period = 1.0 / LOOP_HZ
    last_log = 0.0
    # ---- 平滑状态(跨周期保持) ----
    f_dist = f_lat = None          # EMA 滤波后的距离/横向
    last_tgt_ts = None             # 上次滤波的样本 ts(EMA 只对新样本滤一次)
    prev_speed = prev_steer = 0.0  # 上周期下发值(slew 用)
    last_valid_t = 0.0             # 上次有效目标时刻(丢帧宽限用)
    last_steer_cmd = 0.0           # 丢帧时保持的转向
    # ---- SEEK/RETURN 寻回状态 ----
    last_off = 0.0                 # 丢失前最后的 off_x(判断人从哪侧出画)
    seek_phase = None              # None|'SEEK'|'PAUSE1'|'RETURN'|'DONE'
    seek_dur = 0.0                 # 实际盲走时长(倒车按它原路退)
    seek_steer = 0.0               # 寻回舵角(满舵, 带方向)
    phase_t0 = 0.0
    try:
        while True:
            apply_config(read_config(), ctl)         # web 可热改保持距离/最高速度
            t = read_target()
            now = time.time()

            gate_b = None
            if t is not None:
                gate_b = near_gate(t)                # 近距防撞门: 可能就地收紧 t["dist_m"]
                # ① EMA 低通: 只在出"新样本"时滤一次(同一帧反复平均会让手感随 LOOP_HZ 漂移)
                if t.get("ts") != last_tgt_ts:
                    last_tgt_ts = t.get("ts")
                    if t.get("dist_m") is not None:
                        f_dist = _ema(f_dist, t["dist_m"], DIST_EMA)
                    if t.get("lateral_m") is not None:
                        f_lat = _ema(f_lat, t["lateral_m"], LAT_EMA)
                if gate_b is not None and f_dist is not None:
                    f_dist = min(f_dist, gate_b)     # 防撞门直接压滤波值, 不等 EMA 收敛
                if f_dist is not None:
                    t["dist_m"] = f_dist
                if f_lat is not None:
                    t["lateral_m"] = f_lat
                tgt_speed, tgt_steer, state = compute_cmd(t, prev_speed)
                last_valid_t = now
                last_steer_cmd = tgt_steer
                if t.get("off_x") is not None:
                    last_off = t["off_x"]            # 记住人最后在画面哪侧
                seek_phase = None                    # 重捕获 → 寻回状态复位
            elif (now - last_valid_t) < LOST_GRACE:
                # ④ 短暂丢目标: 滑行——转向逐周期衰减(握死上次舵角会画大弧), 速度缓减到0
                last_steer_cmd *= COAST_STEER_DECAY
                tgt_speed, tgt_steer, state = 0.0, last_steer_cmd, "COAST"
            else:
                # 真丢了: 先「惯性寻回」(小速度满舵朝丢失侧走弧线), 找不到倒车原路归位, 再原地等
                if (seek_phase is None and SEEK_ON and not args.steer_only
                        and abs(last_off) >= SEEK_MIN_OFF
                        and (now - last_valid_t) < LOST_GRACE + 1.5):
                    seek_phase, phase_t0 = 'SEEK', now
                    seek_steer = SEEK_STEER * (1.0 if last_off > 0 else -1.0)
                if seek_phase == 'SEEK':
                    if obstacle_ahead() or (now - phase_t0) >= SEEK_TIME:
                        seek_dur = now - phase_t0    # 走了多久, 倒车就退多久
                        seek_phase, phase_t0 = 'PAUSE1', now
                    tgt_speed = SEEK_SPEED if seek_phase == 'SEEK' else 0.0
                    tgt_steer, state = seek_steer, "SEEK"
                elif seek_phase == 'PAUSE1':         # 停稳再换倒挡(车控有静止换挡互锁)
                    tgt_speed, tgt_steer, state = 0.0, seek_steer, "SEEK"
                    if now - phase_t0 >= RETURN_PAUSE:
                        seek_phase, phase_t0 = 'RETURN', now
                elif seek_phase == 'RETURN':         # 同舵角倒退 = 沿同一圆弧原路回
                    if now - phase_t0 >= seek_dur:
                        seek_phase = 'DONE'
                        tgt_speed, tgt_steer, state = 0.0, 0.0, "SEARCH"
                    else:
                        tgt_speed, tgt_steer, state = SEEK_SPEED, seek_steer, "RETURN"
                else:                                # DONE/未触发: 原地等人回来
                    tgt_speed, tgt_steer, state = 0.0, 0.0, "SEARCH"
                f_dist = f_lat = None

            if args.steer_only:                          # 只转向不前进: 速度锁0
                tgt_speed = 0.0
                if state == "FOLLOW":
                    state = "STEER_ONLY"

            # ③ slew 限幅: 把本周期下发值相对上周期的变化夹住, 杜绝 0→0.4 一步到位
            speed = _slew(prev_speed, tgt_speed, MAX_DSPEED_UP, MAX_DSPEED_DOWN)
            steer = _slew(prev_steer, tgt_steer, MAX_DSTEER, MAX_DSTEER)
            prev_speed, prev_steer = speed, steer

            gear = GEAR_REVERSE if state == "RETURN" else GEAR_DRIVE
            ctl.set_cmd_from_follow(speed, steer, gear=gear)   # 内部取负成 fr 转向
            status = {
                "ts": now, "armed": bool(args.arm), "steer_only": bool(args.steer_only), "state": state,
                "cmd_speed": round(speed, 3), "cmd_steer": round(steer, 2),
                "target_valid": t is not None,
                "dist_m": (round(f_dist, 2) if f_dist is not None else None),
                "lateral_m": (round(f_lat, 2) if f_lat is not None else None),
                "off_x": (round(t["off_x"], 3) if t and t.get("off_x") is not None else None),
                "source": (t.get("source") if t else None),
                "desired": [DESIRED_MIN, DESIRED_MAX],
                "max_speed": round(MAX_SPEED, 2),
                "gate": (round(gate_b, 2) if gate_b is not None else None),
                "grass": MIN_DRIVE >= MIN_DRIVE_GRASS,
                "seek_on": SEEK_ON,
                "gear": ("R" if gear == GEAR_REVERSE else "D"),
            }
            write_status(status)
            if now - last_log >= 1.0:
                ds = ("dist=%.2fm" % f_dist) if f_dist is not None else \
                     (("box_h=%.2f" % t["box_h_norm"]) if (t and t.get("box_h_norm") is not None) else "无距离")
                print("[%s] %-9s %s steer=%+.1f speed=%.2f%s" %
                      (time.strftime("%H:%M:%S"), state, ds, steer, speed,
                       "" if args.arm else " (dry)"))
                last_log = now
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nCtrl-C 停。")
    finally:
        write_status({"ts": time.time(), "armed": False, "state": "OFF",
                      "cmd_speed": 0.0, "cmd_steer": 0.0, "target_valid": False})
        ctl.stop()


if __name__ == "__main__":
    main()
