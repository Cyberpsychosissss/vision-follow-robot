#!/bin/bash
# run_yolo_follow.sh — 在 apollo_dev_nvidia 容器内启动 GPU 人体检测 → runtime/target.json
# 前提: zkhy_grabber 已在写 grab/left_latest.ppm (有深度则 +depth_latest.pgm)。
# 用法(容器内): bash /apollo/follow_data/trtx/run_yolo_follow.sh [额外参数如 --conf 0.4 --focus 1000]
set -e
TRTX=/apollo/follow_data/trtx
GRAB=${GRAB:-/apollo/follow_data/runtime/grab}
RUNTIME=${RUNTIME:-/apollo/follow_data/runtime}
HZ=${HZ:-10}
export LD_LIBRARY_PATH=$TRTX/build:/usr/lib/aarch64-linux-gnu/tegra:/usr/local/cuda-10.0/lib64
cd "$TRTX/build"
echo "[run_yolo_follow] grab=$GRAB runtime=$RUNTIME hz=$HZ"
exec ./yolo_follow --engine yolov5s.engine --grab-dir "$GRAB" --runtime "$RUNTIME" --hz "$HZ" "$@"
