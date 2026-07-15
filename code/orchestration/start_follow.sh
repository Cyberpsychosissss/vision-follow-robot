#!/usr/bin/env bash
# start_follow.sh —— 一键拉起视觉跟随全栈(网页面板 + 感知)。在车上宿主机(nvidia 用户)跑。
#   ./start_follow.sh             网页 + 感知(grabber+yolo)都起 (默认)
#   ./start_follow.sh --web-only  只起网页面板(感知到网页点「▶ 启动感知」)
# 注意: grabber 需要相机空闲。若相机被 fr07 占用, grabber 会起不来——
#       先 `docker exec apollo_dev_nvidia pkill -f zkhy_frame_grabber` 释放(确认 fr07 没人用)。
set -u
BIN="$(cd "$(dirname "$0")" && pwd)"
CONTAINER="${APOLLO_CONTAINER:-apollo_dev_nvidia}"
SEL_CONTAINER="${SELECTOR_CONTAINER:-follow_yolo2026}"   # ReID 选择器容器(有 onnxruntime)
PORT="${FOLLOW_WEB_PORT:-8080}"
GRAB_OUT=/apollo/follow_data/runtime/grab
WEB_ONLY=0
for a in "$@"; do case "$a" in --web-only|--no-cam) WEB_ONLY=1;; esac; done

echo "== 视觉跟随 · 一键启动 =="

# 1) 网页面板(host python3, 会自动带起 bms_monitor)
pkill -f 'web_ui.py' 2>/dev/null && sleep 1
cd "$BIN"
setsid python3 web_ui.py > /tmp/web_ui.log 2>&1 &
sleep 1
IP=$(hostname -I 2>/dev/null | awk '{print $1}')
echo "[1/2] 面板: http://${IP:-<车IP>}:$PORT   (日志 /tmp/web_ui.log)"

if [ "$WEB_ONLY" = "1" ]; then
  echo "[2/2] 跳过感知(--web-only)。到网页点「▶ 启动感知」即可。"
  exit 0
fi

# 2) 感知: grabber(真实米深度) → yolo_follow(检人写 detections.json) → target_selector(ReID 挑主人写 target.json)
#    grabber --write-fps 15 = 不限流(相机实测 ~12.5fps 全吃满; 旧值 9 会白扔近 30% 帧)。
docker exec "$CONTAINER" pgrep -x zkhy_grabber >/dev/null 2>&1 || \
  docker exec -d "$CONTAINER" bash -c "cd /apollo/follow_data/zkhy_grab && \
    LD_LIBRARY_PATH=/apollo/follow_data/lib:/apollo/modules/drivers/zkhy/src/Bin \
    ./zkhy_grabber --out-dir $GRAB_OUT --duration 0 --write-fps 15 > /tmp/grab.log 2>&1"
docker exec "$CONTAINER" pgrep -x yolo_follow >/dev/null 2>&1 || \
  docker exec -d "$CONTAINER" bash -c "cd /apollo/follow_data/trtx/build && \
    LD_LIBRARY_PATH=/apollo/follow_data/trtx/build:/usr/lib/aarch64-linux-gnu/tegra:/usr/local/cuda-10.0/lib64 \
    ./yolo_follow --engine yolov5s.engine --grab-dir $GRAB_OUT \
    --runtime /apollo/follow_data/runtime --out detections.json --hz 15 > /tmp/yolo_follow.log 2>&1"
docker exec "$SEL_CONTAINER" pgrep -f '[t]arget_selector' >/dev/null 2>&1 || \
  docker exec -d "$SEL_CONTAINER" bash -c "cd /apollo/follow_data/bin && \
    OPENBLAS_CORETYPE=ARMV8 PYTHONIOENCODING=utf-8 \
    FOLLOW_RUNTIME=/apollo/follow_data/runtime \
    OSNET_ONNX=/apollo/follow_data/models/osnet_x0_25_msmt17.onnx \
    python3 -u target_selector.py > /tmp/selector.log 2>&1"
sleep 2
G=$(docker exec "$CONTAINER" pgrep -x zkhy_grabber >/dev/null 2>&1 && echo ✓ || echo "✗(看 /tmp/grab.log, 多半相机被占)")
Y=$(docker exec "$CONTAINER" pgrep -x yolo_follow  >/dev/null 2>&1 && echo ✓ || echo "✗(看 /tmp/yolo_follow.log)")
S=$(docker exec "$SEL_CONTAINER" pgrep -f '[t]arget_selector' >/dev/null 2>&1 && echo ✓ || echo "✗(看容器 $SEL_CONTAINER:/tmp/selector.log)")
echo "[2/2] 感知: grabber $G   yolo $Y   选择器 $S"
echo
echo "下一步: 浏览器开面板 → 「左目+YOLO框」里看到人 → 「🔒 锁定我为主人」→ 先「▶ DRY-RUN」看决策 → 再 ARM。"
echo "全部停止: $BIN/stop_follow.sh"
