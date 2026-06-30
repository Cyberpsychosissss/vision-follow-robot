#!/usr/bin/env bash
# ============================================================================
# start_collector.sh — one-click launcher
#
#   bash scripts/start_collector.sh
#
# Runs environment / dependency / CAN / camera / storage checks, then launches
# the interactive collector (collect.py). PHASE 1 is LISTEN-ONLY: this tool
# records data while a human drives manually. It never sends CAN frames.
#
# Design notes:
#   * set -u  (error on unset variables)  but NOT set -e — we want to RUN ALL
#     checks and show every result, not bail out at the first problem.
#   * Output tags:  [OK] [WARN] [ERROR] [FIX]
#   * All output is mirrored to logs/startup_YYYYmmdd_HHMMSS.log
# ============================================================================
set -u

# ---- resolve project root (this script lives in scripts/) ------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${PROJECT_ROOT}" || { echo "[ERROR] cannot cd to project root"; exit 1; }

CONFIG="${PROJECT_ROOT}/config.yaml"
REQ="${PROJECT_ROOT}/requirements.txt"
LOG_DIR="${PROJECT_ROOT}/logs"
mkdir -p "${LOG_DIR}"
TS="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_DIR}/startup_${TS}.log"

# mirror everything to the log file as well as the terminal
exec > >(tee -a "${LOG_FILE}") 2>&1

ERRORS=0
WARNS=0

ok()    { echo "[OK]    $*"; }
warn()  { echo "[WARN]  $*"; WARNS=$((WARNS+1)); }
err()   { echo "[ERROR] $*"; ERRORS=$((ERRORS+1)); }
fix()   { echo "[FIX]   $*"; }
hr()    { echo "------------------------------------------------------------"; }

echo "============================================================"
echo " follow_data_collector — one-click launcher"
echo " $(date)"
echo " log: ${LOG_FILE}"
echo " SAFETY: listen-only CAN, no vehicle control (phase 1)"
echo "============================================================"

# --------------------------------------------------------------------------
# 1. Operating system
# --------------------------------------------------------------------------
hr; echo "1) Operating system"
OS="$(uname -s)"
if [ "${OS}" = "Linux" ]; then
  if [ -r /etc/os-release ]; then
    . /etc/os-release
    ok "Linux detected: ${PRETTY_NAME:-Linux}"
  else
    ok "Linux detected"
  fi
  if uname -a | grep -qiE "tegra|jetson"; then
    ok "NVIDIA Jetson platform detected"
  fi
else
  warn "Non-Linux OS detected (${OS}). SocketCAN/CAN checks may not apply."
  fix "Run this on the Jetson / Ubuntu industrial PC for full functionality."
fi

# --------------------------------------------------------------------------
# 2. Python3 & pip
# --------------------------------------------------------------------------
hr; echo "2) Python & pip"
if command -v python3 >/dev/null 2>&1; then
  ok "python3: $(python3 --version 2>&1)"
else
  err "python3 not found."
  fix "Install Python 3:  sudo apt-get install -y python3"
fi

PIP_OK=0
if command -v pip3 >/dev/null 2>&1; then
  ok "pip3: $(pip3 --version 2>&1 | awk '{print $1, $2}')"
  PIP_OK=1
elif python3 -m pip --version >/dev/null 2>&1; then
  ok "pip available via 'python3 -m pip'"
  PIP_OK=1
else
  warn "pip3 not found."
  fix "Install pip:  sudo apt-get install -y python3-pip"
fi

# --------------------------------------------------------------------------
# 3. config.yaml (generate default if missing)
# --------------------------------------------------------------------------
hr; echo "3) Configuration"
if [ -f "${CONFIG}" ]; then
  ok "config.yaml found"
else
  warn "config.yaml not found — generating a default one."
  if python3 -c "import sys; sys.argv=['x']; \
import importlib.util as u; spec=u.spec_from_file_location('c','${PROJECT_ROOT}/collect.py'); \
m=u.module_from_spec(spec); spec.loader.exec_module(m); \
from pathlib import Path; m.write_default_config(Path('${CONFIG}'))" 2>/dev/null; then
    ok "default config.yaml written"
  else
    err "could not auto-generate config.yaml"
    fix "Copy the example config from the README into config.yaml"
  fi
fi

# --------------------------------------------------------------------------
# 4. Python dependencies
# --------------------------------------------------------------------------
hr; echo "4) Python dependencies"
check_import() {
  local mod="$1"; local pretty="$2"
  if python3 -c "import ${mod}" >/dev/null 2>&1; then
    ok "${pretty} import OK"
    return 0
  else
    return 1
  fi
}

MISSING=()
check_import "cv2"  "OpenCV (cv2)"        || { warn "OpenCV not importable"; MISSING+=("opencv-python"); }
check_import "can"  "python-can"          || { warn "python-can not importable"; MISSING+=("python-can"); }
check_import "yaml" "PyYAML"              || { err  "PyYAML not importable";  MISSING+=("PyYAML"); }
check_import "rich" "rich (dashboard)"    || { warn "rich not importable (will use plain output)"; MISSING+=("rich"); }
check_import "pandas" "pandas (tools)"    || { warn "pandas not importable (needed by tools/)"; MISSING+=("pandas"); }

if [ "${#MISSING[@]}" -gt 0 ]; then
  echo
  warn "Missing/!importable packages: ${MISSING[*]}"
  if [ "${PIP_OK}" -eq 1 ] && [ -f "${REQ}" ]; then
    # China mirror (Tsinghua TUNA) + no-cache to avoid root-owned ~/.cache/pip
    PIP_MIRROR="${PIP_INDEX_URL:-https://pypi.tuna.tsinghua.edu.cn/simple}"
    PIP_HOST="$(echo "${PIP_MIRROR}" | sed -E 's#https?://([^/]+)/.*#\1#')"
    PIP_ARGS="--no-cache-dir --user -i ${PIP_MIRROR} --trusted-host ${PIP_HOST}"
    echo "  Using mirror: ${PIP_MIRROR}"
    read -r -p "  Install dependencies now from this mirror? [y/N]: " ans
    if [ "${ans:-N}" = "y" ] || [ "${ans:-N}" = "Y" ]; then
      # Detect Python minor version. Several deps dropped Python 3.6 support:
      #   python-can>=4.0, numpy>=1.20, rich>=13, pandas>=1.2 all need 3.7+.
      # On 3.6 we must pin to the last compatible releases.
      PYMM="$(python3 -c 'import sys;print(sys.version_info[0]*100+sys.version_info[1])' 2>/dev/null || echo 306)"
      if [ "${PYMM}" -le 306 ]; then
        warn "Python <=3.6 detected: pinning 3.6-compatible package versions."
        CAN_PKG="python-can==3.3.4"   # last version supporting Python 3.6
        OPT_PKGS="rich==12.6.0"       # optional; pandas/numpy left to system/apt
      else
        CAN_PKG="python-can"
        OPT_PKGS="rich pandas cantools"
      fi
      # OpenCV is NEVER pip-installed here:
      #   - Jetson/aarch64 has no opencv-python wheel; building from source is painful.
      #   - Use the system build instead:  sudo apt-get install -y python3-opencv
      REQUIRED="${CAN_PKG} PyYAML"
      echo "Installing required: ${REQUIRED}"
      if pip3 install ${PIP_ARGS} ${REQUIRED}; then
        ok "required deps installed (mirror)"
      else
        warn "Tsinghua mirror failed; retrying with Aliyun mirror"
        pip3 install --no-cache-dir --user \
          -i https://mirrors.aliyun.com/pypi/simple \
          --trusted-host mirrors.aliyun.com ${REQUIRED} \
          || err "required install failed (check internet / captive portal)"
      fi
      # optional extras: best-effort, never fail the launcher
      echo "Installing optional (best-effort): ${OPT_PKGS}"
      pip3 install ${PIP_ARGS} ${OPT_PKGS} 2>/dev/null \
        || warn "optional packages skipped (not required for collection)"
      fix "OpenCV must come from apt:  sudo apt-get install -y python3-opencv"
      fix "Then verify:  python3 -c 'import cv2; print(cv2.__version__)'"
    else
      fix "Install later:  pip3 install --user --no-cache-dir -i ${PIP_MIRROR} python-can==3.3.4 PyYAML"
      fix "OpenCV:  sudo apt-get install -y python3-opencv"
    fi
  else
    fix "pip3 install -r requirements.txt"
  fi
fi

# --------------------------------------------------------------------------
# 5. CAN device & permissions
# --------------------------------------------------------------------------
hr; echo "5) CAN bus"
# read channel/bitrate from config (best effort, falls back to defaults)
CAN_CH="$(python3 -c "import yaml;print(yaml.safe_load(open('${CONFIG}'))['can']['channel'])" 2>/dev/null || echo can0)"
CAN_BR="$(python3 -c "import yaml;print(yaml.safe_load(open('${CONFIG}'))['can']['bitrate'])" 2>/dev/null || echo 500000)"
CAN_EN="$(python3 -c "import yaml;print(yaml.safe_load(open('${CONFIG}'))['can']['enabled'])" 2>/dev/null || echo True)"

if [ "${CAN_EN}" = "False" ]; then
  warn "CAN disabled in config.yaml — image-only collection."
else
  if [ ! -d "/sys/class/net/${CAN_CH}" ]; then
    warn "${CAN_CH} not found."
    fix "ip -details link show ${CAN_CH}   # confirm USB-CAN driver is installed (dmesg | grep -i can)"
  else
    ok "${CAN_CH} device present"
    OPERSTATE="$(cat /sys/class/net/${CAN_CH}/operstate 2>/dev/null || echo unknown)"
    # CAN often shows 'unknown' while actually up; check flags bit IFF_UP (0x1)
    FLAGS_HEX="$(cat /sys/class/net/${CAN_CH}/flags 2>/dev/null || echo 0x0)"
    IS_UP=0
    if [ $(( FLAGS_HEX & 0x1 )) -eq 1 ]; then IS_UP=1; fi
    if [ "${IS_UP}" -eq 1 ]; then
      ok "${CAN_CH} is UP"
    else
      warn "${CAN_CH} is DOWN (operstate=${OPERSTATE})"
      read -r -p "  Bring ${CAN_CH} up now (sudo ip link set ${CAN_CH} up type can bitrate ${CAN_BR})? [y/N]: " ans
      if [ "${ans:-N}" = "y" ] || [ "${ans:-N}" = "Y" ]; then
        if sudo -n true 2>/dev/null || sudo true 2>/dev/null; then
          sudo ip link set "${CAN_CH}" down 2>/dev/null || true
          if sudo ip link set "${CAN_CH}" up type can bitrate "${CAN_BR}"; then
            ok "${CAN_CH} brought up at ${CAN_BR}"
          else
            err "failed to bring up ${CAN_CH}"
            fix "Run manually: sudo ip link set ${CAN_CH} up type can bitrate ${CAN_BR}"
          fi
        else
          warn "no sudo privileges available."
          fix "Ask an admin or run: sudo ip link set ${CAN_CH} up type can bitrate ${CAN_BR}"
        fi
      else
        fix "sudo ip link set ${CAN_CH} up type can bitrate ${CAN_BR}"
      fi
    fi
  fi
fi

# --------------------------------------------------------------------------
# 6. Camera
# --------------------------------------------------------------------------
hr; echo "6) Camera"
CAM_SRC="$(python3 -c "import yaml;print(yaml.safe_load(open('${CONFIG}'))['camera']['source'])" 2>/dev/null || echo 0)"
if python3 -c "import cv2" >/dev/null 2>&1; then
  RESULT="$(python3 - "${CAM_SRC}" <<'PY' 2>/dev/null
import sys, cv2
src = sys.argv[1]
cap = cv2.VideoCapture(int(src)) if src.isdigit() else cv2.VideoCapture(src)
opened = cap.isOpened()
ok = False
if opened:
    ok, _ = cap.read()
cap.release()
print("OK" if ok else ("OPEN_NOFRAME" if opened else "FAIL"))
PY
)"
  case "${RESULT}" in
    OK)           ok "camera source '${CAM_SRC}' opened and produced a frame" ;;
    OPEN_NOFRAME) warn "camera '${CAM_SRC}' opened but no frame yet (give it a moment / check stream)" ;;
    *)            warn "camera source '${CAM_SRC}' could not be opened"
                  fix "Try source 0 or 1, or an RTSP URL. Run: python3 scripts/check_camera.py" ;;
  esac
else
  warn "OpenCV not available; skipping camera open test."
  fix "Install OpenCV or test on the Jetson with system cv2."
fi

# --------------------------------------------------------------------------
# 7. Storage / disk space
# --------------------------------------------------------------------------
hr; echo "7) Storage"
ROOT_DIR="$(python3 -c "import yaml;print(yaml.safe_load(open('${CONFIG}'))['dataset']['root_dir'])" 2>/dev/null || echo ./dataset)"
mkdir -p "${ROOT_DIR}" 2>/dev/null || true
if [ -w "${ROOT_DIR}" ]; then
  ok "output directory writable: ${ROOT_DIR}"
else
  err "output directory not writable: ${ROOT_DIR}"
  fix "Fix permissions or change dataset.root_dir in config.yaml"
fi

FREE_KB="$(df -Pk "${ROOT_DIR}" 2>/dev/null | awk 'NR==2{print $4}')"
if [ -n "${FREE_KB:-}" ]; then
  FREE_GB=$(( FREE_KB / 1024 / 1024 ))
  if [ "${FREE_GB}" -lt 5 ]; then
    warn "low disk space: ${FREE_GB} GB free (< 5 GB)"
    fix "Free space or change dataset.root_dir to a larger disk"
  else
    ok "disk space: ${FREE_GB} GB free"
  fi
else
  warn "could not determine free disk space"
fi

# --------------------------------------------------------------------------
# Summary
# --------------------------------------------------------------------------
hr
echo "Summary:  ${ERRORS} error(s), ${WARNS} warning(s)."
echo "Full log: ${LOG_FILE}"
hr

if [ "${ERRORS}" -gt 0 ]; then
  echo
  warn "There were ERRORS above. You can still continue (image-only is possible)."
  read -r -p "Launch the collector anyway? [y/N]: " ans
  if [ "${ans:-N}" != "y" ] && [ "${ans:-N}" != "Y" ]; then
    echo "Aborted. Fix the [ERROR] items above and re-run."
    exit 1
  fi
fi

echo
echo "==> Launching: python3 collect.py --config config.yaml"
echo
exec python3 "${PROJECT_ROOT}/collect.py" --config "${CONFIG}"
