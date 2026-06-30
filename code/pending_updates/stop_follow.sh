#!/usr/bin/env bash
# stop_follow.sh —— 一键停掉跟随全栈(控制器→感知→网页→BMS)。在车上宿主机跑。
CONTAINER="${APOLLO_CONTAINER:-apollo_dev_nvidia}"
echo "停止跟随控制器(退出会自动下发停车)..."; pkill -f 'follow_controller.py' 2>/dev/null
echo "停止感知(容器内 yolo + grabber)...";       docker exec "$CONTAINER" pkill -x yolo_follow   2>/dev/null
                                                  docker exec "$CONTAINER" pkill -x zkhy_grabber 2>/dev/null
echo "停止网页面板...";                            pkill -f 'web_ui.py' 2>/dev/null
echo "停止 BMS 监控...";                           pkill -f 'bms_monitor.py' 2>/dev/null
sleep 1
echo "全部已停。"
