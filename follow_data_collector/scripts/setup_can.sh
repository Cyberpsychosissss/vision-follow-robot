#!/usr/bin/env bash
# ============================================================
# setup_can.sh — bring up a SocketCAN interface (e.g. can0)
#
# PHASE 1 SAFETY: this only configures the interface for LISTENING.
# The collector never transmits. Do not run any CAN sender.
#
# Usage:
#   bash scripts/setup_can.sh            # uses can0 @ 500000
#   bash scripts/setup_can.sh can0 500000
# ============================================================
set -u

CAN_IF="${1:-can0}"
BITRATE="${2:-500000}"

echo "==> Configuring ${CAN_IF} at ${BITRATE} bit/s (listen-only use)"

if ! command -v ip >/dev/null 2>&1; then
  echo "[ERROR] 'ip' command not found (iproute2). Install it first."
  exit 1
fi

# Bring the interface down first (ignore error if already down / absent)
sudo ip link set "${CAN_IF}" down 2>/dev/null || true

# Bring it up with the requested bitrate
if sudo ip link set "${CAN_IF}" up type can bitrate "${BITRATE}"; then
  echo "[OK] ${CAN_IF} is up at ${BITRATE} bit/s"
else
  echo "[ERROR] Failed to bring up ${CAN_IF}."
  echo "[FIX] Check the device exists:  ip link show ${CAN_IF}"
  echo "[FIX] Confirm the USB-CAN driver is loaded (dmesg | grep -i can)."
  exit 1
fi

echo
echo "==> Interface details:"
ip -details link show "${CAN_IF}"

echo
echo "Tip: verify live frames while the car is driven manually:"
echo "     candump ${CAN_IF}"
