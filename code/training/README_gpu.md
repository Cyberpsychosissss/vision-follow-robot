# GPU 服务器训练包(视觉跟随 BC 模型)

包内已含 5 个 run 的对齐表 + 160×120 图像缓存,**不需要原始 19G 数据**。

## 1. 上传(Mac 端, 端口/密码从 AutoDL 控制台拿当前的)

```bash
scp -P <端口> ~/Desktop/follow_data/gpu_train_pkg.tar root@connect.westb.seetacloud.com:/root/autodl-tmp/
```

## 2. 服务器上解包 + 环境

```bash
cd /root/autodl-tmp && tar xf gpu_train_pkg.tar && cd gpu_train_pkg
python3 -c "import torch, PIL, numpy"   # AutoDL 镜像一般全有; 缺啥 pip install 啥
```

## 3. 训练(带日志归档, 推荐照抄)

```bash
cd /root/autodl-tmp/gpu_train_pkg && mkdir -p logs
PY=/root/miniconda3/bin/python
TS=$(date +%m%d_%H%M)

# 后台训练, 全程日志落 logs/train_<时间>.log
nohup $PY -u train_follow.py --data ./dataset --epochs 20 > logs/train_$TS.log 2>&1 &
tail -f logs/train_$TS.log        # 实时看(Ctrl-C 只退出tail不停训练)

# 跑完把本轮曲线和模型归档(train_log.csv/bcnet_best.pt 每轮会被覆盖, 记得存)
cp checkpoints/train_log.csv logs/train_log_$TS.csv
cp checkpoints/bcnet_best.pt logs/bcnet_$TS.pt
```

**v2 推荐配置(清洗+正则, 2026-07-07 加)**:

```bash
nohup $PY -u train_follow.py --data ./dataset --epochs 30 \
  --balance 0.3 --jitter --valid-only > logs/train_${TS}_v2.log 2>&1 &
```

- `--balance 0.3` 直行帧(|转向|<2°)只留 30%(v1 里直行占 61%, 模型学会偷懒输出小角度)
- `--jitter` 亮度/对比度抖动(补光照多样性)
- `--valid-only` 剔画面无人帧; **注意 run_004 没有 target 标签, 开这个等于整个排除它**(训练量约剩 4300 序列)
- dropout 0.3 + weight decay 5e-4 已是新默认(`--dropout/--wd` 可调)
- 验证集永远不清洗不增强, 各轮 val 指标可直接互比; v1 基准: 恒输出0° 的 val 转向 MAE=3.54°

其他变体:

```bash
nohup $PY -u train_follow.py --data ./dataset --epochs 20 --flip > logs/train_${TS}_flip.log 2>&1 &   # A/B: 翻转增强
nohup $PY -u train_follow.py --data ./dataset --epochs 40 --lr 5e-4 > logs/train_${TS}_lr5e4.log 2>&1 &
nohup $PY -u train_follow.py --data ./dataset --val-runs run_008 > logs/train_${TS}_val8.log 2>&1 &    # 换验证run
pkill -f train_follow.py          # 停掉在跑的训练
```

产出 `checkpoints/bcnet_best.pt`(val最优) + `checkpoints/train_log.csv`(每 epoch: train/val loss、val 转向MAE°、速度MAE)。
判断标准: val 转向 MAE 要显著低于"恒输出0°"基准(≈val集平均|转向|, 约4-5°)才算学到东西。

## 4. 导出 ONNX(为车上 TRT6 部署准备)

```bash
python3 export_onnx.py        # → checkpoints/bcnet.onnx (opset 11)
```

## 5. 取回结果(Mac 端)

```bash
scp -P <端口> root@connect.westb.seetacloud.com:/root/autodl-tmp/gpu_train_pkg/checkpoints/\{bcnet_best.pt,bcnet.onnx,train_log.csv\} ~/Desktop/follow_data/code/training/checkpoints/
```

## 备注

- 验证集按 run 切(默认留 run_007),**不要改成随机切帧**(相邻帧泄漏)。
- 速度头当前数据下会学成近似巡航均值(示范速度与距离 r≈0.06),正常;
  以后采了"刻意随距离变速"的数据,重跑同一脚本即可。
- run_010 是静止废数据,已排除;新增 run 后先在 Mac 跑 `build_dataset.py`
  重新生成 aligned.csv + 缓存,再把新 run 的两个文件补传上来。
