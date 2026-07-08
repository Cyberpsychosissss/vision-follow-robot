#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""export_onnx.py —— 把训练好的 BCNet 导出 ONNX(opset 11, 兼容车上 TRT6 路线)。

用法: python3 export_onnx.py [--ckpt checkpoints/bcnet_best.pt] [--out bcnet.onnx]
输入: (1, 5, 3, 120, 160) float32 [0,1] RGB 序列
输出: steer_norm(×25=deg), speed_norm(×1.5=m/s)
"""
import os, argparse
import torch
from train_follow import BCNet, SEQ_LEN, IMG_W, IMG_H

HERE = os.path.dirname(os.path.abspath(__file__))

ap = argparse.ArgumentParser()
ap.add_argument('--ckpt', default=os.path.join(HERE, 'checkpoints', 'bcnet_best.pt'))
ap.add_argument('--out', default=os.path.join(HERE, 'checkpoints', 'bcnet.onnx'))
args = ap.parse_args()

ck = torch.load(args.ckpt, map_location='cpu', weights_only=False)
model = BCNet()
model.load_state_dict(ck['model'])
model.eval()

x = torch.zeros(1, SEQ_LEN, 3, IMG_H, IMG_W)
torch.onnx.export(model, x, args.out, opset_version=11,
                  input_names=['frames'], output_names=['steer_norm', 'speed_norm'])
print('exported %s (epoch %s, val_steer_mae %.2f°)' % (
    args.out, ck.get('epoch', '?'), ck.get('val_steer_mae_deg', float('nan'))))

# 自检: onnxruntime 有装就比对一次前向
try:
    import onnxruntime as ort
    import numpy as np
    s = ort.InferenceSession(args.out, providers=['CPUExecutionProvider'])
    xin = np.random.rand(1, SEQ_LEN, 3, IMG_H, IMG_W).astype('float32')
    o = s.run(None, {'frames': xin})
    with torch.no_grad():
        ps, pv = model(torch.from_numpy(xin))
    ds = abs(o[0][0][0] - ps.item()); dv = abs(o[1][0][0] - pv.item())
    print('onnxruntime 比对: dsteer=%.2e dspeed=%.2e %s' % (ds, dv, 'OK' if max(ds, dv) < 1e-4 else 'MISMATCH!'))
except ImportError:
    print('(未装 onnxruntime, 跳过前向比对)')
