#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""make_jpg_cache.py —— 把原始 uint8 缓存转成 JPEG(q92)压缩缓存, 供慢链路上传。
存储格式: npz{blob: uint8一维拼接, offs: int64 N+1 偏移, names: 文件名数组}
"""
import os, io, sys
import numpy as np
from PIL import Image

DS = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1
                        else '~/Desktop/follow_data/follow_data_collector/dataset')
for d in sorted(os.listdir(DS)):
    raw = os.path.join(DS, d, 'cache_160x120.npz')
    out = os.path.join(DS, d, 'cache_160x120_jpg.npz')
    if not os.path.exists(raw):
        continue
    z = np.load(raw, allow_pickle=False)
    imgs, names = z['imgs'], z['names']
    parts, offs = [], [0]
    for im in imgs:
        b = io.BytesIO()
        Image.fromarray(im).save(b, 'JPEG', quality=92)
        parts.append(np.frombuffer(b.getvalue(), dtype=np.uint8))
        offs.append(offs[-1] + len(parts[-1]))
    np.savez(out, blob=np.concatenate(parts), offs=np.array(offs, dtype=np.int64), names=names)
    print('%s: %d 帧  %.1fMB → %.1fMB' % (d, len(imgs),
          os.path.getsize(raw) / 1e6, os.path.getsize(out) / 1e6))
