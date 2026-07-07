#!/usr/bin/env bash
# start_follow.sh —— 一键拉起视觉跟随全栈(网页面板 + 感知)。在车上宿主机(nvidia 用户)跑。
#   ./start_follow.sh             网页 + 感知(grabber+yolo)都起 (默认)
#   ./start_follow.sh --web-only  只起网页面板(感知到网页点「▶ 启动感知」)
# 注意: grabber 需要相机空闲。若相机被 fr07 占用, grabber 会起不来——
#       先 `docker exec apollo_dev_nvidia pkill -f zkhy_frame_grabber` 释放(确认 fr07 没人用)。
set -u
BIN="$(cd "$(dirname "$0")" && pwd)"
CONTAINER="${APOLLO_CONTAINER:-apollo_dev_nvidia}"
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

# 2) 感知: 容器内 grabber(写真实米深度) + yolo_follow(检人写 target.json)
#    帧率提到 grabber@9fps / yolo@15hz —— 喂控制器更连续, 跟随更跟手。
docker exec "$CONTAINER" pgrep -x zkhy_grabber >/dev/null 2>&1 || \
  docker exec -d "$CONTAINER" bash -c "cd /apollo/follow_data/zkhy_grab && \
    LD_LIBRARY_PATH=/apollo/follow_data/lib:/apollo/modules/drivers/zkhy/src/Bin \
    ./zkhy_grabber --out-dir $GRAB_OUT --duration 0 --write-fps 9 > /tmp/grab.log 2>&1"
docker exec "$CONTAINER" pgrep -x yolo_follow >/dev/null 2>&1 || \
  docker exec -d "$CONTAINER" bash -c "cd /apollo/follow_data/trtx/build && \
    LD_LIBRARY_PATH=/apollo/follow_data/trtx/build:/usr/lib/aarch64-linux-gnu/tegra:/usr/local/cuda-10.0/lib64 \
    ./yolo_follow --engine yolov5s.engine --grab-dir $GRAB_OUT \
    --runtime /apollo/follow_data/runtime --hz 15 > /tmp/yolo_follow.log 2>&1"
sleep 2
G=$(docker exec "$CONTAINER" pgrep -x zkhy_grabber >/dev/null 2>&1 && echo ✓ || echo "✗(看 /tmp/grab.log, 多半相机被占)")
Y=$(docker exec "$CONTAINER" pgrep -x yolo_follow  >/dev/null 2>&1 && echo ✓ || echo "✗(看 /tmp/yolo_follow.log)")
echo "[2/2] 感知: grabber $G   yolo $Y"
echo
echo "下一步: 浏览器开面板 → 「左目+YOLO框」里看到人 → 先「▶ DRY-RUN」看决策对不对 → 再 ARM。"
echo "全部停止: $BIN/stop_follow.sh"
