#!/usr/bin/env bash
# restore_old.sh —— 一键还原到部署前的旧版。在车上宿主机跑: ./restore_old.sh [备份目录]
# 不给参数就用 bin/backup_* 里最新的一份(部署脚本在覆盖前自动创建)。
set -u
BIN="$(cd "$(dirname "$0")" && pwd)"
BK="${1:-$(ls -d "$BIN"/backup_* 2>/dev/null | sort | tail -1)}"
if [ -z "${BK:-}" ] || [ ! -d "$BK" ]; then
  echo "✗ 没找到备份目录 $BIN/backup_*"; exit 1
fi
echo "== 一键还原 · 来源: $BK =="

echo "[1/3] 停掉跟随全栈..."
if [ -x "$BIN/stop_follow.sh" ]; then "$BIN/stop_follow.sh"; else
  pkill -f 'follow_controller.py' 2>/dev/null
  pkill -f 'web_ui.py' 2>/dev/null
  pkill -f 'bms_monitor.py' 2>/dev/null
fi
sleep 1

echo "[2/3] 拷回旧版文件..."
cp -v "$BK"/* "$BIN"/

echo "[3/3] 重启旧版面板..."
cd "$BIN"
setsid python3 web_ui.py > /tmp/web_ui.log 2>&1 &
sleep 1
IP=$(hostname -I 2>/dev/null | awk '{print $1}')
echo "✔ 已还原并重启面板: http://${IP:-<车IP>}:8080"
echo "  (还原后是旧版控制律; 新版文件在 Mac 仓库 code/pending_updates/ 可随时重新部署)"
