#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""train_follow.py —— 端到端跟随 BC 模型(转向+速度双头), Mac 本地训练。

架构(按 experiment_plan Phase2, 加速度头):
  5帧RGB序列(各160x120, 间隔0.25s) → 共享PilotNet式CNN → FC256 → GRU(128)
  → steer头 tanh(单位: /25deg)  +  speed头 sigmoid(单位: /1.5m/s)
说明: 当前示范数据速度与距离弱相关(r≈0.06), speed头会先学成"巡航均值",
     架构留好, 以后采了刻意变速数据重跑本脚本即可。部署时急停仍走规则硬门槛。

用法: python3 train_follow.py [--epochs 20] [--val-runs run_007] [--data <root>]
输出: code/training/checkpoints/bcnet_best.pt / bcnet_last.pt + train_log.csv
"""
import os, csv, json, math, argparse, random
import numpy as np
from PIL import Image
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader

HERE      = os.path.dirname(os.path.abspath(__file__))
DS_DEF    = os.path.expanduser('~/Desktop/follow_data/follow_data_collector/dataset')
RUNS_DEF  = ['run_004', 'run_006', 'run_007', 'run_008', 'run_009']   # run_010 静止废数据, 不用
SEQ_LEN   = 5
SEQ_DT    = 0.25    # s, 序列帧间隔(按时间取最近邻, 兼容 4.6/9.6fps 混采)
SEQ_TOL   = 0.15    # s, 最近邻容差, 超出则该锚点作废
IMG_W, IMG_H = 160, 120
STEER_SCALE  = 25.0   # deg, 底盘限幅
SPEED_SCALE  = 1.5    # m/s, 固件上限


# ---------------- 数据 ----------------
def load_run(run_dir):
    """读 aligned.csv + 图像缓存(首次把 ppm/jpg 统一转 160x120 uint8 存 .npz)"""
    rows = []
    with open(os.path.join(run_dir, 'aligned.csv')) as f:
        for r in csv.DictReader(f):
            rows.append((r['filename'], float(r['ts_wall']),
                         float(r['speed_mps']), float(r['steer_deg']),
                         int(r.get('tgt_valid') or 0)))
    cache = os.path.join(run_dir, 'cache_%dx%d.npz' % (IMG_W, IMG_H))
    names = [r[0] for r in rows]
    # JPEG 压缩缓存(为慢链路传输做的, make_jpg_cache.py 生成), 启动时解码进内存
    jcache = os.path.join(run_dir, 'cache_%dx%d_jpg.npz' % (IMG_W, IMG_H))
    if os.path.exists(jcache):
        import io
        z = np.load(jcache, allow_pickle=False)
        if list(z['names']) == names:
            blob, offs = z['blob'], z['offs']
            imgs = np.zeros((len(names), IMG_H, IMG_W, 3), dtype=np.uint8)
            for i in range(len(names)):
                imgs[i] = np.asarray(Image.open(io.BytesIO(blob[offs[i]:offs[i + 1]].tobytes())).convert('RGB'))
            return rows, imgs
    if os.path.exists(cache):
        z = np.load(cache, allow_pickle=False)
        if list(z['names']) == names:
            return rows, z['imgs']
    print('  [cache] 生成 %s (%d 帧)...' % (os.path.basename(run_dir), len(rows)))
    imgs = np.zeros((len(rows), IMG_H, IMG_W, 3), dtype=np.uint8)
    for i, (fn, _, _, _) in enumerate(rows):
        im = Image.open(os.path.join(run_dir, 'images', fn)).convert('RGB')
        w, h = im.size                      # 1280x720 → 中央裁 4:3 再缩, 避免压扁
        cw = int(h * IMG_W / IMG_H)
        if cw <= w:
            x0 = (w - cw) // 2
            im = im.crop((x0, 0, x0 + cw, h))
        imgs[i] = np.asarray(im.resize((IMG_W, IMG_H), Image.BILINEAR))
    np.savez(cache, imgs=imgs, names=np.array(names))
    return rows, imgs


class SeqDataset(Dataset):
    STRAIGHT_DEG = 2.0   # |转向|<2° 视为直行(用于 --balance 降采样)

    def __init__(self, runs, root, augment=False, jitter=False,
                 valid_only=False, balance=None, seed=1234):
        self.augment = augment
        self.jitter = jitter
        self.samples = []         # (idx_t-4..idx_t, steer_norm, speed_norm)
        self.imgs = []
        rng = random.Random(seed)                     # balance 用独立 RNG, 结果可复现
        n_straight = n_drop = n_invalid = 0
        offset = 0
        for rn in runs:
            rows, imgs = load_run(os.path.join(root, rn))
            self.imgs.append(imgs)
            ts = [r[1] for r in rows]
            # run 里一条 tgt_valid 都没有(如 run_004 老格式没录 target.csv)= 标签缺失,
            # 不等于画面没人, valid_only 对这种 run 不生效
            has_tgt = any(r[4] for r in rows)
            for i in range(len(rows)):
                if valid_only and has_tgt and not rows[i][4]:   # 画面无有效目标的帧不当锚点
                    n_invalid += 1
                    continue
                idxs = []
                ok = True
                for k in range(SEQ_LEN - 1, -1, -1):
                    want = ts[i] - k * SEQ_DT
                    j = min(range(max(0, i - 40), i + 1), key=lambda x: abs(ts[x] - want))
                    if abs(ts[j] - want) > SEQ_TOL:
                        ok = False
                        break
                    idxs.append(offset + j)
                if not ok:
                    continue
                if balance is not None and abs(rows[i][3]) < self.STRAIGHT_DEG:
                    n_straight += 1
                    if rng.random() > balance:        # 直行帧只保留 balance 比例
                        n_drop += 1
                        continue
                steer = max(-1.0, min(1.0, rows[i][3] / STEER_SCALE))
                speed = max(0.0, min(1.0, rows[i][2] / SPEED_SCALE))
                self.samples.append((idxs, steer, speed))
            offset += len(rows)
        self.imgs = np.concatenate(self.imgs, axis=0)
        if valid_only or balance is not None:
            print('  [clean] 剔无目标帧 %d, 直行 %d 中降采样丢弃 %d → 剩 %d 序列'
                  % (n_invalid, n_straight, n_drop, len(self.samples)))

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, i):
        idxs, steer, speed = self.samples[i]
        x = self.imgs[idxs].astype(np.float32) / 255.0          # (T,H,W,3)
        if self.augment and random.random() < 0.5:              # 水平翻转: 转向取负
            x = x[:, :, ::-1, :].copy()
            steer = -steer
        if self.jitter:                                         # 光照增强: 整段序列同一变换
            g = random.uniform(0.7, 1.3)                        # 亮度增益
            c = random.uniform(0.8, 1.2)                        # 对比度
            m = x.mean()
            x = np.clip((x * g - m) * c + m, 0.0, 1.0)
        x = torch.from_numpy(np.ascontiguousarray(x)).permute(0, 3, 1, 2)   # (T,3,H,W)
        return x, torch.tensor([steer], dtype=torch.float32), torch.tensor([speed], dtype=torch.float32)


# ---------------- 模型 ----------------
class BCNet(nn.Module):
    def __init__(self, hidden=128, dropout=0.3):
        super().__init__()
        # Dropout 层无论 p 多少都在(p=0 等于关), 保证 state_dict 键稳定
        # BatchNorm: 从零训练的小CNN在清洗后(难)数据上会塌缩成常数输出, BN 破解(2026-07-07 实测)
        self.cnn = nn.Sequential(
            nn.Conv2d(3, 24, 5, 2, bias=False), nn.BatchNorm2d(24), nn.ReLU(),
            nn.Conv2d(24, 36, 5, 2, bias=False), nn.BatchNorm2d(36), nn.ReLU(),
            nn.Conv2d(36, 48, 5, 2, bias=False), nn.BatchNorm2d(48), nn.ReLU(),
            nn.Conv2d(48, 64, 3, 1, bias=False), nn.BatchNorm2d(64), nn.ReLU(),
            nn.Conv2d(64, 64, 3, 1, bias=False), nn.BatchNorm2d(64), nn.ReLU(), nn.Flatten(),
            nn.Linear(6656, 256), nn.ReLU(),
        )
        self.drop = nn.Dropout(dropout)
        self.gru = nn.GRU(256, hidden, batch_first=True)
        self.steer_head = nn.Sequential(nn.Linear(hidden, 64), nn.ReLU(), nn.Linear(64, 1), nn.Tanh())
        self.speed_head = nn.Sequential(nn.Linear(hidden, 64), nn.ReLU(), nn.Linear(64, 1), nn.Sigmoid())

    def forward(self, x):                       # x: (B,T,3,H,W)
        B, T = x.shape[:2]
        f = self.cnn(x.reshape(B * T, *x.shape[2:])).reshape(B, T, -1)
        f = self.drop(f)                        # 帧特征 dropout
        _, h = self.gru(f)
        h = self.drop(h[-1])                    # (B,hidden)
        return self.steer_head(h), self.speed_head(h)


# ---------------- 训练 ----------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', default=DS_DEF)
    ap.add_argument('--epochs', type=int, default=20)
    ap.add_argument('--batch', type=int, default=64)
    ap.add_argument('--lr', type=float, default=1e-3)
    ap.add_argument('--val-runs', default='run_007')
    ap.add_argument('--flip', action='store_true',
                    help='水平翻转增强(镜像场景+转向取负), 默认关; 想A/B对比再开')
    ap.add_argument('--balance', type=float, default=None, metavar='KEEP',
                    help='直行帧(|转向|<2°)降采样, 只保留 KEEP 比例(推荐 0.3); 只作用于训练集')
    ap.add_argument('--jitter', action='store_true', help='亮度/对比度抖动增强(训练集)')
    ap.add_argument('--valid-only', action='store_true',
                    help='只用画面有有效目标的帧当锚点(注意: run_004 无 target 标签, 开了等于整个排除)')
    ap.add_argument('--dropout', type=float, default=0.3)
    ap.add_argument('--wd', type=float, default=5e-4, help='weight decay')
    ap.add_argument('--speed-w', type=float, default=0.5)
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()

    random.seed(args.seed); np.random.seed(args.seed); torch.manual_seed(args.seed)
    dev = torch.device('mps' if torch.backends.mps.is_available()
                       else 'cuda' if torch.cuda.is_available() else 'cpu')
    val_runs = args.val_runs.split(',')
    train_runs = [r for r in RUNS_DEF if r not in val_runs]
    print('device=%s  train=%s  val=%s' % (dev, train_runs, val_runs))
    print('flags: flip=%s balance=%s jitter=%s valid_only=%s dropout=%.2f wd=%g'
          % (args.flip, args.balance, args.jitter, args.valid_only, args.dropout, args.wd))

    tr = SeqDataset(train_runs, args.data, augment=args.flip, jitter=args.jitter,
                    valid_only=args.valid_only, balance=args.balance)
    va = SeqDataset(val_runs, args.data)     # 验证集永远原样, 各轮实验指标可比
    print('train %d 序列 / val %d 序列' % (len(tr), len(va)))
    trl = DataLoader(tr, batch_size=args.batch, shuffle=True, num_workers=0)
    val = DataLoader(va, batch_size=args.batch, shuffle=False, num_workers=0)

    model = BCNet(dropout=args.dropout).to(dev)
    opt = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=args.wd)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=args.epochs)
    hub = nn.SmoothL1Loss(); mse = nn.MSELoss()

    ckdir = os.path.join(HERE, 'checkpoints'); os.makedirs(ckdir, exist_ok=True)
    logf = open(os.path.join(ckdir, 'train_log.csv'), 'w')
    logf.write('epoch,train_loss,val_loss,val_steer_mae_deg,val_speed_mae_mps\n')
    best = float('inf')

    for ep in range(1, args.epochs + 1):
        model.train(); tl = 0.0
        for it, (x, ys, yv) in enumerate(trl):
            x, ys, yv = x.to(dev), ys.to(dev), yv.to(dev)
            ps, pv = model(x)
            loss = hub(ps, ys) + args.speed_w * mse(pv, yv)
            opt.zero_grad(); loss.backward()
            if os.environ.get('DBG_ITERS') and ep == 1 and it < int(os.environ['DBG_ITERS']):
                gn = sum(p.grad.norm().item() for p in model.parameters() if p.grad is not None)
                print('  it%03d loss %.4f grad %.3f steer_std %.4f' % (
                    it, loss.item(), gn, ps.detach().std().item()), flush=True)
            opt.step()
            tl += loss.item() * len(x)
        tl /= len(tr); sched.step()

        model.eval(); vl = smae = vmae = 0.0
        with torch.no_grad():
            for x, ys, yv in val:
                x, ys, yv = x.to(dev), ys.to(dev), yv.to(dev)
                ps, pv = model(x)
                vl += (hub(ps, ys) + args.speed_w * mse(pv, yv)).item() * len(x)
                smae += (ps - ys).abs().sum().item()
                vmae += (pv - yv).abs().sum().item()
        vl /= len(va)
        smae = smae / len(va) * STEER_SCALE
        vmae = vmae / len(va) * SPEED_SCALE
        star = ''
        if vl < best:
            best = vl
            torch.save({'model': model.state_dict(), 'epoch': ep,
                        'val_steer_mae_deg': smae, 'val_speed_mae_mps': vmae},
                       os.path.join(ckdir, 'bcnet_best.pt'))
            star = '  *best'
        torch.save({'model': model.state_dict(), 'epoch': ep}, os.path.join(ckdir, 'bcnet_last.pt'))
        print('ep%02d  train %.4f  val %.4f  转向MAE %.2f°  速度MAE %.3fm/s%s'
              % (ep, tl, vl, smae, vmae, star))
        logf.write('%d,%.5f,%.5f,%.3f,%.4f\n' % (ep, tl, vl, smae, vmae)); logf.flush()
    logf.close()
    print('done. best val loss %.4f → %s' % (best, os.path.join(ckdir, 'bcnet_best.pt')))


if __name__ == '__main__':
    main()
