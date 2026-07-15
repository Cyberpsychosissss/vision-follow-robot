#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
target_selector.py — 特定目标锁定选择器 (ReID)

在 yolo_follow(检人写 detections.json) 和 follow_controller(读 target.json 控车) 之间插一层:
  yolo_follow  --out detections.json  (top-N person 候选框, 每框 bbox/conf/off_x/box_h_norm/dist_m?)
        │
  target_selector.py  ← 读 detections.json + left_latest.ppm → OSNet 外观特征 → 打分选目标
        │
  target.json  (契约与旧版完全一致: 顶层就是被选中那个框; 新增 locked/lock_conf/track/candidates)
        │
  follow_controller.py  (零改动, 照常读 target.json)

行为:
  · 未锁定  → 透传最大框 (= 旧版 yolo_follow 行为, 严格等价)
  · 锁定中  → 面板写 follow_config.json {"lock":true,"lock_ts":T}, 边沿触发注册当时最大框为「主人」,
              之后每帧对候选打分 = 外观相似 + 位置连续 + 深度连续, 选主人; 找不到 → valid:false
              (控制器自然走 COAST/SEEK/SEARCH, 不会误跟路人)
  · 重捕获  → 丢失后只认外观(更高阈值+连续同一候选N帧), 防 SEEK 扫过路人误锁

安全兜底:
  · 本进程挂掉 → target.json 不再更新 → 控制器 0.6s 超时 → 停车 (现有机制)
  · fail-loud: detections.json 停更但 target.json 仍被别人刷新(= 旧二进制在直写, 双写打架) → 打日志退出
  · ORT/权重加载失败 → 退化为纯几何(HSV 直方图)描述子, 打警告继续 (救场, 非交付路径)

环境: follow_yolo2026 容器 (py3.6 + onnxruntime 1.10 CPU + numpy + pillow), 无 cv2。
"""
from __future__ import print_function
import os
import sys
import json
import time
import math

RUNTIME = os.environ.get("FOLLOW_RUNTIME", "/apollo/follow_data/runtime")
GRAB_DIR = os.environ.get("GRAB_DIR", os.path.join(RUNTIME, "grab"))
DET_FILE = os.path.join(RUNTIME, "detections.json")
TARGET_FILE = os.path.join(RUNTIME, "target.json")
CONFIG_FILE = os.path.join(RUNTIME, "follow_config.json")
GALLERY_FILE = os.path.join(RUNTIME, "lock_gallery.json")
LEFT_PPM = os.path.join(GRAB_DIR, "left_latest.ppm")
MODEL = os.environ.get("OSNET_ONNX", "/apollo/follow_data/models/osnet_x0_25_msmt17.onnx")

# ---- 打分权重(常态) ----
W_APP = 0.60     # 外观相似(OSNet 余弦, gallery 取最大)
W_IOU = 0.25     # 位置连续(与上一帧预测框的 IoU)
W_DEPTH = 0.15   # 深度连续(与上次距离的接近度)
# ---- 近距半身框: 外观不可靠(只框到腿/半身), 让位置连续性主导 ----
W_APP_NEAR, W_IOU_NEAR, W_DEPTH_NEAR = 0.20, 0.60, 0.20
NEAR_BOTTOM_FRAC = 0.90   # 框底 >= 画面 90% 高度处
NEAR_BIG_FRAC = 0.55      # 且框高 >= 画面 55% → 判为"贴近/半身"

# ---- 阈值(2026-07-15 用车载 demo 真人素材实测标定: 同一人 cos 0.72~0.77, 不同人 0.32~0.44) ----
ACCEPT = 0.55        # 跟踪中: 最高分(融合分)候选 >= 此值才认
APP_MIN = 0.50       # 跟踪中的外观硬底线(原始余弦): 防"站在主人位置上的路人"靠 IoU 分蹭过融合分
                     #   (实测不同人最高 0.435 → 0.50 挡得住; 近距半身模式下豁免, 那时外观本来失真)
REACQ = 0.60         # 重捕获(丢失后)外观门槛: 高于不同人上界 0.435, 低于同一人下界 0.716, 各留余量
REACQ_FRAMES = 3     # 且连续这么多帧命中"同一个"候选(帧间 IoU 关联)才恢复
LOST_APP_ONLY = 1.0  # s  丢失超过这么久后, 打分只看外观(位置/深度失效)

# ---- gallery(主人外观库) ----
GALLERY_MAX = 10
GAL_ADD_COS = 0.68      # 高于此才追加进库(同一人中位 0.75, 旧值 0.75 会把一半真样本挡掉)
GAL_ADD_MARGIN = 0.15   # 且与次高候选分差够大(不确定时冻结, 防被路人劫持)

DEPTH_SCALE = 1.0    # m  深度连续性: exp(-|Δd|/scale)
IOU_PRED_ALPHA = 1.0 # 恒速预测: pred = last + alpha*(last-prev)
POLL_HZ = 20.0
STALE_DET = 2.0      # s  detections 停更超过此值算感知没了
DBL_WRITE_GUARD = 2.0  # s  detections 停更但 target 仍新鲜 → 双写打架, 退出


def log(msg):
    print("[selector] %s %s" % (time.strftime("%H:%M:%S"), msg), file=sys.stderr)
    sys.stderr.flush()


# ============ 外观特征后端 ============
class OSNetBackend(object):
    """OSNet ONNX (onnxruntime CPU). 输入 1x3x256x128 RGB, ImageNet 归一, 输出 512 维。"""
    MEAN = (0.485, 0.456, 0.406)
    STD = (0.229, 0.224, 0.225)
    IN_W, IN_H = 128, 256

    def __init__(self, path):
        import numpy as np
        import onnxruntime as ort
        self.np = np
        so = ort.SessionOptions()
        so.intra_op_num_threads = int(os.environ.get("SEL_THREADS", "3"))
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.sess = ort.InferenceSession(path, sess_options=so,
                                         providers=["CPUExecutionProvider"])
        self.iname = self.sess.get_inputs()[0].name
        self.dim = 512
        log("OSNet loaded: %s (in=%s)" % (os.path.basename(path), self.iname))

    def embed(self, pil_crop):
        np = self.np
        im = pil_crop.convert("RGB").resize((self.IN_W, self.IN_H))
        a = np.asarray(im, dtype=np.float32) / 255.0   # HWC
        a = (a - np.array(self.MEAN, np.float32)) / np.array(self.STD, np.float32)
        a = a.transpose(2, 0, 1)[None]                 # 1CHW
        f = self.sess.run(None, {self.iname: a.astype(np.float32)})[0][0]
        n = float(np.linalg.norm(f)) + 1e-9
        return f / n


class HSVBackend(object):
    """救场备胎: 上/下半身 HSV 直方图 (无 ORT/权重时用, 不作为交付路径)。"""
    def __init__(self):
        import numpy as np
        self.np = np
        self.dim = 256
        log("!! OSNet 不可用, 退化到 HSV 直方图描述子(仅救场)")

    def embed(self, pil_crop):
        np = self.np
        im = pil_crop.convert("HSV").resize((32, 64))
        a = np.asarray(im, dtype=np.float32)
        h, w = a.shape[:2]
        cw0, cw1 = int(w * 0.2), int(w * 0.8)   # 取中间 60% 宽, 躲背景
        parts = []
        for y0, y1 in ((int(h * 0.15), int(h * 0.55)), (int(h * 0.55), int(h * 0.92))):
            reg = a[y0:y1, cw0:cw1]
            if reg.size == 0:
                parts.append(np.zeros(128, np.float32)); continue
            hh = np.histogram2d(reg[..., 0].ravel(), reg[..., 1].ravel(),
                                bins=(16, 8), range=((0, 255), (0, 255)))[0].ravel()
            s = hh.sum()
            parts.append(hh / s if s > 0 else hh)
        f = np.concatenate(parts).astype(np.float32)
        n = float(np.linalg.norm(f)) + 1e-9
        return f / n


def cos(np, u, v):
    return float(np.dot(u, v))   # 都已 L2 归一


# ============ IO ============
def read_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except Exception:
        return None


def write_atomic(path, obj):
    tmp = path + ".tmp"
    try:
        with open(tmp, "w") as f:
            json.dump(obj, f)
        os.replace(tmp, path)
    except Exception as e:
        log("write %s fail: %s" % (path, e))


def load_pil(path, max_age=1.0):
    """读左目 PPM。可能撞上半写帧 → 返回 None 跳过。"""
    try:
        if time.time() - os.path.getmtime(path) > max_age:
            return None
        from PIL import Image
        im = Image.open(path)
        im.load()   # 强制立即解码, 半写帧在这里抛异常
        return im
    except Exception:
        return None


def iou(a, b):
    ax, ay, aw, ah = a[:4]; bx, by, bw, bh = b[:4]
    x0 = max(ax, bx); y0 = max(ay, by)
    x1 = min(ax + aw, bx + bw); y1 = min(ay + ah, by + bh)
    iw = max(0.0, x1 - x0); ih = max(0.0, y1 - y0)
    inter = iw * ih
    ua = aw * ah + bw * bh - inter
    return inter / ua if ua > 0 else 0.0


def crop_box(im, bbox):
    x, y, w, h = bbox
    W, H = im.size
    x0 = max(0, int(x)); y0 = max(0, int(y))
    x1 = min(W, int(x + w)); y1 = min(H, int(y + h))
    if x1 - x0 < 4 or y1 - y0 < 8:
        return None
    return im.crop((x0, y0, x1, y1))


def largest(cands):
    best, ba = None, -1
    for c in cands:
        b = c.get("bbox")
        if not b:
            continue
        a = b[2] * b[3]
        if a > ba:
            ba, best = a, c
    return best


class Gallery(object):
    def __init__(self, np):
        self.np = np
        self.embs = []          # list of np arrays
        self.lock_ts = None

    def load(self):
        d = read_json(GALLERY_FILE)
        if d and isinstance(d.get("embs"), list) and d.get("embs"):
            self.embs = [self.np.array(e, self.np.float32) for e in d["embs"]]
            self.lock_ts = d.get("lock_ts")
            log("gallery restored: %d embs lock_ts=%s" % (len(self.embs), self.lock_ts))
            return True
        return False

    def save(self):
        write_atomic(GALLERY_FILE, {
            "lock_ts": self.lock_ts,
            "embs": [[round(float(x), 4) for x in e] for e in self.embs],
        })

    def enroll(self, emb, lock_ts):
        self.embs = [emb]
        self.lock_ts = lock_ts
        self.save()

    def clear(self):
        self.embs = []
        self.lock_ts = None
        try:
            os.remove(GALLERY_FILE)
        except Exception:
            pass

    def best_cos(self, emb):
        if not self.embs:
            return -1.0
        return max(cos(self.np, emb, g) for g in self.embs)

    def maybe_add(self, emb, top, second):
        if top >= GAL_ADD_COS and (top - second) >= GAL_ADD_MARGIN:
            self.embs.append(emb)
            if len(self.embs) > GALLERY_MAX:
                self.embs.pop(0)
            self.save()


def main():
    try:
        import numpy as np
    except Exception as e:
        log("numpy 缺失, 无法运行: %s" % e); sys.exit(2)

    # 后端: 优先 OSNet, 失败退 HSV
    backend = None
    if os.path.exists(MODEL):
        try:
            backend = OSNetBackend(MODEL)
        except Exception as e:
            log("OSNet 加载失败(%s), 退 HSV" % e)
    else:
        log("模型不存在: %s → 退 HSV" % MODEL)
    if backend is None:
        backend = HSVBackend()

    gal = Gallery(np)
    gal.load()

    period = 1.0 / POLL_HZ
    last_det_ts = None            # detections.json 里的 ts(内容时间戳, 非 mtime)
    last_det_change = time.time() # 上次 det ts 变化的墙钟
    det_alive = False             # 见过 det ts 真的变过(感知确认活过) → 才武装双写自检
    waited_first = False          # 「等首帧」日志只打一次
    registered_lock_ts = gal.lock_ts
    # 跟踪状态
    prev_box = None               # 上上帧选中框(算恒速预测)
    last_box = None               # 上一帧选中框
    last_dist = None
    last_seen = 0.0               # 上次成功锁定命中墙钟
    reacq_cand = None             # 重捕获候选(帧间 IoU 关联)
    reacq_hits = 0
    warned_dblwrite = False

    log("started. runtime=%s model=%s" % (RUNTIME, MODEL))

    while True:
        t0 = time.time()
        det = read_json(DET_FILE)
        cfg = read_json(CONFIG_FILE) or {}
        locked = bool(cfg.get("lock"))
        cfg_lock_ts = cfg.get("lock_ts")

        # ---- 感知新鲜度 / 双写自检 ----
        det_ts = det.get("ts") if det else None
        if det_ts is not None and det_ts != last_det_ts:
            if last_det_ts is not None:
                det_alive = True  # ts 从一个已知值变到新值 = 感知真的在刷新
            last_det_ts = det_ts
            last_det_change = time.time()
        det_stale = (time.time() - last_det_change) > STALE_DET
        if det_stale:
            tgt_now = read_json(TARGET_FILE)
            # det_alive 门: 冷启动时 yolo 加载引擎要 ~15s, detections 天然停更,
            # 此时 target.json 的新鲜 ts 可能是残留/自己写的 → 没确认感知活过之前不许自杀
            if det_alive and tgt_now and (time.time() - (tgt_now.get("ts") or 0)) < DBL_WRITE_GUARD:
                # detections 停更但 target 还在被刷新 = 别人(旧二进制)在直写 target.json
                if not warned_dblwrite:
                    log("!! 检测到 target.json 被外部刷新而 detections 停更 → 双写打架, 退出避让")
                    warned_dblwrite = True
                    sys.exit(3)
            # 否则就是感知真没了: 不写 target, 让它自然过期, 控制器超时停车
            time.sleep(period); continue
        if det_ts is None:
            # detections.json 还没出现(首启引擎加载中): 绝不打自己的钟写 target, 干等
            if not waited_first:
                log("等 detections.json 首帧(yolo 引擎加载中)...")
                waited_first = True
            time.sleep(period); continue

        cands = (det.get("candidates") if det else None) or []
        # 兼容: 若 det 是旧格式(顶层单框无 candidates)
        if not cands and det and det.get("valid") and det.get("bbox"):
            cands = [det]

        # ---- 解锁 / 未锁定: 透传最大框(严格等价旧版) ----
        if not locked:
            if registered_lock_ts is not None:
                gal.clear(); registered_lock_ts = None
                last_box = prev_box = None; last_dist = None; reacq_hits = 0
            best = largest(cands)
            out = passthrough(det, best, len(cands))
            write_atomic(TARGET_FILE, out)
            sleep_to(t0, period); continue

        im = load_pil(LEFT_PPM)

        # ---- 边沿触发: 新锁定命令 → 注册当时最大框为主人 ----
        if cfg_lock_ts is not None and cfg_lock_ts != registered_lock_ts:
            best = largest(cands)
            emb = embed_box(backend, im, best) if best else None
            if emb is not None:
                gal.enroll(emb, cfg_lock_ts)
                registered_lock_ts = cfg_lock_ts
                last_box = best.get("bbox"); prev_box = None
                last_dist = best.get("dist_m"); last_seen = time.time()
                reacq_hits = 0; reacq_cand = None
                log("ENROLLED 主人: bbox=%s dist=%s" % (last_box, last_dist))
            else:
                # 还没法注册(无候选/图没到) → 报 NEED_ENROLL, 下帧再试
                write_atomic(TARGET_FILE, lost_out(det, len(cands), "NEED_ENROLL"))
                sleep_to(t0, period); continue

        if not gal.embs:
            write_atomic(TARGET_FILE, lost_out(det, len(cands), "NEED_ENROLL"))
            sleep_to(t0, period); continue

        # ---- 对每个候选打分 ----
        now = time.time()
        lost_dur = now - last_seen
        pred_box = predict_box(prev_box, last_box)
        scored = []
        for c in cands:
            bbox = c.get("bbox")
            if not bbox:
                continue
            emb = embed_box(backend, im, c)
            # app_raw = 原始余弦(标准 ReID 尺度, 阈值/gallery 判定都用它; 也是最好解读的置信度)
            app_raw = gal.best_cos(emb) if emb is not None else -1.0
            # app01 = 映射到 [0,1] 只为和 iou/depth 同尺度做加权融合
            app01 = max(0.0, (app_raw + 1.0) / 2.0) if backend.dim == 512 else max(0.0, app_raw)
            iou_s = iou(pred_box, bbox) if pred_box else 0.0
            dep_s = depth_score(np, c.get("dist_m"), last_dist)
            wa, wi, wd, near = weights_for(bbox, det)
            if lost_dur > LOST_APP_ONLY:
                score = app01           # 丢久了只信外观
            else:
                score = wa * app01 + wi * iou_s + wd * dep_s
            scored.append((score, app_raw, app01, iou_s, near, c, emb))
        scored.sort(key=lambda x: x[0], reverse=True)

        if not scored:
            # 画面无人: 目标丢失, 但保留 last_box/last_dist 作重捕获锚点
            write_lost(det, len(cands), last_box, last_dist, lost_dur)
            reacq_hits = 0; reacq_cand = None
            sleep_to(t0, period); continue

        top_score, top_app, top_app01, top_iou, top_near, top_c, top_emb = scored[0]
        second_app = scored[1][1] if len(scored) > 1 else -1.0   # 次高候选的原始余弦

        tracking = lost_dur <= LOST_APP_ONLY
        if tracking:
            # 融合分过线 + 外观硬底线(防路人站进主人的预测框里靠 IoU 蹭分);
            # 近距半身模式豁免底线——那时外观本来失真, 由 IoU 连续性主导, 冒充者也没法瞬移进框
            hit = top_score >= ACCEPT and (top_near or top_app >= APP_MIN
                                           or (backend.dim != 512))
        else:
            # 重捕获: 外观(原始余弦)必须过更高门槛 + 连续 REACQ_FRAMES 帧命中同一候选
            if top_app >= REACQ:
                if reacq_cand is not None and iou(reacq_cand, top_c["bbox"]) > 0.3:
                    reacq_hits += 1
                else:
                    reacq_hits = 1
                reacq_cand = top_c["bbox"]
                hit = reacq_hits >= REACQ_FRAMES
            else:
                reacq_hits = 0; reacq_cand = None
                hit = False

        if hit:
            bbox = top_c["bbox"]
            prev_box, last_box = last_box, bbox
            last_dist = top_c.get("dist_m", last_dist)
            last_seen = now
            reacq_hits = 0; reacq_cand = None
            if top_emb is not None and backend.dim == 512:
                gal.maybe_add(top_emb, top_app, second_app)   # 用原始余弦判是否入库
            out = passthrough(det, top_c, len(cands))
            out["locked"] = True
            out["lock_conf"] = round(top_app, 3)              # 报原始余弦(可直接对标阈值)
            out["track"] = "TRACKING" if tracking else "REACQ"
            out["candidates"] = slim_cands(scored)
            write_atomic(TARGET_FILE, out)
        else:
            write_lost(det, len(cands), last_box, last_dist, lost_dur, scored)
        sleep_to(t0, period)


# ---- 小工具 ----
def embed_box(backend, im, cand):
    if im is None or cand is None:
        return None
    crop = crop_box(im, cand["bbox"])
    if crop is None:
        return None
    try:
        return backend.embed(crop)
    except Exception as e:
        log("embed fail: %s" % e)
        return None


def predict_box(prev_box, last_box):
    if last_box is None:
        return None
    if prev_box is None:
        return last_box
    return [last_box[i] + IOU_PRED_ALPHA * (last_box[i] - prev_box[i]) for i in range(4)]


def depth_score(np, d, last_d):
    if d is None or last_d is None:
        return 0.5
    return float(math.exp(-abs(d - last_d) / DEPTH_SCALE))


def weights_for(bbox, det):
    """近距/半身框 → 外观让位给位置连续性。返回 (w_app, w_iou, w_depth, near_mode)。"""
    ih = float((det or {}).get("img_h") or 720)
    bottom = bbox[1] + bbox[3]
    if bottom >= NEAR_BOTTOM_FRAC * ih and bbox[3] >= NEAR_BIG_FRAC * ih:
        return W_APP_NEAR, W_IOU_NEAR, W_DEPTH_NEAR, True
    return W_APP, W_IOU, W_DEPTH, False


def passthrough(det, cand, n):
    """把选中框写成旧契约的 target.json 顶层结构。ts 只透传 det 的钟, 绝不打自己的
    (打自己的钟会让双写自检把自己当成外部写者 → 启动即自杀; 也会骗过控制器超时)。"""
    ts = (det or {}).get("ts")
    if cand is None:
        return {"ts": ts, "valid": False, "source": "yolo_reid", "n_persons": n}
    out = {"ts": ts, "valid": True, "source": "yolo_reid",
           "off_x": cand.get("off_x"), "box_h_norm": cand.get("box_h_norm"),
           "conf": cand.get("conf"), "n_persons": n,
           "bbox": cand.get("bbox"),
           "img_w": (det or {}).get("img_w"), "img_h": (det or {}).get("img_h")}
    if cand.get("dist_m") is not None:
        out["dist_m"] = cand["dist_m"]
        if cand.get("lateral_m") is not None:
            out["lateral_m"] = cand["lateral_m"]
        out["depth"] = True
    else:
        out["depth"] = False
    return out


def lost_out(det, n, track):
    # 带上 img_w/img_h: 目标丢了但面板仍要按原图尺寸画出「画面里的其他人」(灰框)
    return {"ts": (det or {}).get("ts"), "valid": False,
            "source": "yolo_reid", "n_persons": n, "locked": True, "track": track,
            "img_w": (det or {}).get("img_w"), "img_h": (det or {}).get("img_h")}


def write_lost(det, n, last_box, last_dist, lost_dur, scored=None):
    out = lost_out(det, n, "LOST")
    out["lost_s"] = round(lost_dur, 2)
    if scored is not None:
        out["candidates"] = slim_cands(scored)
    write_atomic(TARGET_FILE, out)


def slim_cands(scored):
    """给面板画多框用: 每候选 bbox + 分数(不含 embedding)。app=原始余弦(面板显示「像 X%」)。"""
    out = []
    for sc, app_raw, app01, iou_s, near, c, _emb in scored[:6]:
        out.append({"bbox": c.get("bbox"), "score": round(sc, 3),
                    "app": round(max(0.0, app_raw), 3), "conf": c.get("conf")})
    return out


def sleep_to(t0, period):
    dt = time.time() - t0
    if dt < period:
        time.sleep(period - dt)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
