#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
car_control.py — fr 小车 CAN 控制库 (ctrl_cmd_98c4d2d0)

协议来源: /apollo/modules/canbus/vehicle/fr/protocol/ctrl_cmd_98c4d2d0.cc
  CAN ID  : 0x98C4D2D0  (扩展帧/29bit), DLC=8, 周期 20ms(50Hz), Intel/小端
  字段布局 (set_value(value, start_bit, len)):
    gear     data[0] bit0-3                          4=前进(D) 3=空 2=倒 1=驻 0=无效
    velocity data[0]b4-7 | data[1] | data[2]b0-3     x=round(v/0.001) 无符号   范围 0~1.5 m/s
    steering data[2]b4-7 | data[3] | data[4]b0-3     x=round(deg/0.01) 有符号16bit  范围 ±25°
    brake    data[4]b4-7 | data[5]b0-3               整数 0~127
    alive    data[6] b4-7                            滚动计数, MCU 靠它判控制器是否在线
    bcc      data[7] = XOR(data[0..6])               异或校验

  ⚠ 转向符号: fr 车「负=右转」, 而我们 4090 上验证的跟随控制器是「正=右」。
    接跟随控制器时务必取负:  steer_fr = STEER_SIGN * steer_controller   (STEER_SIGN = -1)

  ⚠ 无独立使能握手: 这台 fr 车只要「持续 50Hz 发本帧」即进入受控状态
    (fr_controller.cc::EnableAutoMode 里 set_*enable(true) 全是注释掉的, 真正动作只有持续发帧)。
    总线前提: 底盘 MCU 在线广播 18C4D2EF 等反馈帧 (已只读嗅探确认)。

安全设计 (本脚本的硬约束):
  - 默认 **dry-run**: 只打印将要发送的帧, 不调用 bus.send。必须显式 --arm 才真发。
  - **硬限速**: MAX_SPEED 默认 0.40 m/s; 绝对天花板 ABS_MAX_SPEED=1.50 m/s(=固件上限,
    2026-07-07 用户确认放开; 实际上限由 web 面板「最高速度」在 0~1.5 内热调, 默认仍 0.4)。
  - **死人开关**: 超过 CMD_TIMEOUT 没有新指令 → 自动降到 0 速 + 刹车。
  - **退出归零**: Ctrl-C / stop() 时连发若干「0 速 + 刹车」帧再关闭总线。
  - 上车真发前: 车轮架空台架先行; 人在场带物理急停。

依赖: python-can (宿主机已装 3.3.4)。channel=can0, bustype=socketcan。
"""

import sys
import time
import threading
import argparse

# ---------------- 协议常量 ----------------
CTRL_CMD_ID   = 0x98C4D2D0   # 扩展帧
DLC           = 8
PERIOD        = 0.02         # 50Hz
VEL_RES       = 0.001        # m/s per LSB
STEER_RES     = 0.01         # deg per LSB

GEAR_DRIVE, GEAR_NEUTRAL, GEAR_REVERSE, GEAR_PARK, GEAR_INVALID = 4, 3, 2, 1, 0
STEER_SIGN    = -1           # 跟随控制器(正=右) → fr(负=右) 的换算符号

# ---------------- 安全包络 (硬限) ----------------
MAX_SPEED      = 0.40        # m/s  默认硬限速 (跟随用, 室内慢速; web 面板可热调)
ABS_MAX_SPEED  = 1.50        # m/s  绝对天花板 = 固件 velocity 上限, 任何入参/CLI/面板都不得超过
MAX_STEER      = 25.0        # deg  固件范围 ±25
CMD_TIMEOUT    = 0.30        # s    超过这么久没有新指令就当死人开关触发
BRAKE_ON_STOP  = 25          # 停车/死人开关时给的刹车量 (0~127)
SPEED_SLEW     = 1.2         # m/s/s 速度变化兜底限幅(防猛冲), 0=不限。加速实际由控制器 ACCEL_UP(0.6)管;
                             # 放到 1.2 是别把控制器减速(ACCEL_DOWN=1.6→受此夹到1.2)卡死在旧值0.6——停车要快, 安全优先


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def encode_ctrl_cmd(speed_mps, steer_deg, gear=GEAR_DRIVE, brake=0, alive=0,
                    max_speed=MAX_SPEED):
    """把(速度, 转向, 档位, 刹车, alive) 编码成 8 字节 ctrl_cmd_98c4d2d0 帧。

    speed_mps : 期望车速 (m/s), 会被夹到 [0, min(max_speed, ABS_MAX_SPEED)]
    steer_deg : 转向角(度), fr 原生符号(负=右), 会被夹到 ±MAX_STEER
                注意: 若来自跟随控制器(正=右), 调用前先乘 STEER_SIGN
    返回: bytes(8)
    """
    cap = min(max_speed, ABS_MAX_SPEED)
    speed_mps = _clamp(float(speed_mps), 0.0, cap)
    steer_deg = _clamp(float(steer_deg), -MAX_STEER, MAX_STEER)
    brake     = int(_clamp(int(brake), 0, 127))
    gear      = int(_clamp(int(gear), 0, 4))

    v = int(round(speed_mps / VEL_RES)) & 0xFFFF          # 无符号
    s = int(round(steer_deg / STEER_RES)) & 0xFFFF        # 有符号 → 16bit 二进制补码

    d = bytearray(DLC)
    # gear: byte0 bit0-3
    d[0] |= (gear & 0xF)
    # velocity: byte0 b4-7, byte1 b0-7, byte2 b0-3
    d[0] |= (v & 0xF) << 4
    d[1] |= (v >> 4) & 0xFF
    d[2] |= (v >> 12) & 0xF
    # steering: byte2 b4-7, byte3 b0-7, byte4 b0-3
    d[2] |= (s & 0xF) << 4
    d[3] |= (s >> 4) & 0xFF
    d[4] |= (s >> 12) & 0xF
    # brake: byte4 b4-7, byte5 b0-3
    d[4] |= (brake & 0xF) << 4
    d[5] |= (brake >> 4) & 0xF
    # alive: byte6 b4-7
    d[6] |= (alive & 0xF) << 4
    # bcc: byte7 = XOR(byte0..6)
    bcc = 0
    for i in range(7):
        bcc ^= d[i]
    d[7] = bcc & 0xFF
    return bytes(d)


def decode_ctrl_cmd(data):
    """把 8 字节帧解回物理量, 用于自检/核对。返回 dict。"""
    d = bytearray(data)
    gear = d[0] & 0xF
    v = (d[0] >> 4) | (d[1] << 4) | ((d[2] & 0xF) << 12)
    s = ((d[2] >> 4) | (d[3] << 4) | ((d[4] & 0xF) << 12)) & 0xFFFF
    if s >= 0x8000:
        s -= 0x10000                                       # 16bit 有符号
    brake = (d[4] >> 4) | ((d[5] & 0xF) << 4)
    alive = d[6] >> 4
    bcc = 0
    for i in range(7):
        bcc ^= d[i]
    return {
        "gear": gear,
        "speed_mps": round(v * VEL_RES, 3),
        "steer_deg": round(s * STEER_RES, 2),
        "brake": brake,
        "alive": alive,
        "bcc_ok": (bcc & 0xFF) == d[7],
    }


def hexstr(data):
    return " ".join("%02X" % b for b in data)


class CarController(object):
    """50Hz 后台发帧 + 死人开关 + dry-run 的控制器。

    用法:
        ctl = CarController(dry_run=True, max_speed=0.4)
        ctl.start()
        ctl.set_cmd(speed_mps=0.3, steer_deg=5.0)   # 跟随循环里持续调用
        ...
        ctl.stop()
    """

    def __init__(self, channel="can0", bustype="socketcan",
                 dry_run=True, max_speed=MAX_SPEED, verbose=True):
        self.channel = channel
        self.bustype = bustype
        self.dry_run = dry_run
        self.max_speed = min(max_speed, ABS_MAX_SPEED)
        self.verbose = verbose

        self._bus = None
        self._alive = 0
        self._lock = threading.Lock()
        self._target_speed = 0.0
        self._target_steer = 0.0
        self._last_cmd_t = 0.0
        self._cur_speed = 0.0          # 经过 slew 的实际下发速度
        self._running = False
        self._thread = None

    # ---- 对外指令接口 (跟随循环调用) ----
    def set_cmd(self, speed_mps, steer_deg):
        """更新期望(速度, 转向)。线程安全。steer_deg 为 fr 原生符号(负=右)。"""
        with self._lock:
            self._target_speed = float(speed_mps)
            self._target_steer = float(steer_deg)
            self._last_cmd_t = time.monotonic()

    def set_cmd_from_follow(self, speed_mps, steer_ctrl_deg):
        """跟随控制器专用: steer_ctrl_deg 为控制器符号(正=右), 内部取负。"""
        self.set_cmd(speed_mps, STEER_SIGN * steer_ctrl_deg)

    # ---- 生命周期 ----
    def start(self):
        if not self.dry_run:
            import can
            self._bus = can.interface.Bus(channel=self.channel, bustype=self.bustype)
        self._running = True
        self._last_cmd_t = time.monotonic()
        self._thread = threading.Thread(target=self._loop, name="car_ctrl_50hz")
        self._thread.daemon = True
        self._thread.start()
        mode = "DRY-RUN(不发帧)" if self.dry_run else "ARMED(真发帧!)"
        if self.verbose:
            print("[CarController] 启动 %s  channel=%s  硬限速=%.2f m/s" %
                  (mode, self.channel, self.max_speed))

    def _send(self, data):
        if self.dry_run:
            return
        import can
        msg = can.Message(arbitration_id=CTRL_CMD_ID, is_extended_id=True, data=data)
        self._bus.send(msg)

    def _loop(self):
        next_t = time.monotonic()
        tick = 0
        while self._running:
            now = time.monotonic()
            with self._lock:
                tgt_speed = self._target_speed
                tgt_steer = self._target_steer
                age = now - self._last_cmd_t

            # 死人开关: 太久没新指令 → 强制 0 速 + 刹车
            deadman = age > CMD_TIMEOUT
            if deadman:
                tgt_speed = 0.0
                brake = BRAKE_ON_STOP
            else:
                brake = 0

            # 速度 slew 限制(防猛冲/急停抖动)
            if SPEED_SLEW > 0:
                max_step = SPEED_SLEW * PERIOD
                dv = tgt_speed - self._cur_speed
                if dv > max_step:
                    dv = max_step
                elif dv < -max_step:
                    dv = -max_step
                self._cur_speed += dv
            else:
                self._cur_speed = tgt_speed

            self._alive = (self._alive + 1) & 0xF
            data = encode_ctrl_cmd(self._cur_speed, tgt_steer, gear=GEAR_DRIVE,
                                   brake=brake, alive=self._alive,
                                   max_speed=self.max_speed)
            try:
                self._send(data)
            except Exception as e:                          # 总线异常 → 安全停
                print("[CarController] 发送异常, 安全停止:", e)
                self._running = False
                break

            if self.verbose and (self.dry_run or tick % 25 == 0):
                dec = decode_ctrl_cmd(data)
                flag = " DEADMAN" if deadman else ""
                print("  [%6.2fs] %s | v=%.3f steer=%+.2f brake=%d alive=%d%s" %
                      (now % 10000, hexstr(data), dec["speed_mps"],
                       dec["steer_deg"], dec["brake"], dec["alive"], flag))
            tick += 1

            next_t += PERIOD
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()                   # 落后了就重置节拍

    def stop(self):
        """安全停止: 连发若干 0速+刹车 帧, 再关闭。"""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        # 收尾刹停帧
        for i in range(10):
            self._alive = (self._alive + 1) & 0xF
            data = encode_ctrl_cmd(0.0, 0.0, gear=GEAR_DRIVE, brake=BRAKE_ON_STOP,
                                   alive=self._alive, max_speed=self.max_speed)
            try:
                self._send(data)
            except Exception:
                break
            time.sleep(PERIOD)
        if self._bus is not None:
            try:
                self._bus.shutdown()
            except Exception:
                pass
        if self.verbose:
            print("[CarController] 已安全停止。")


# ======================= 命令行 =======================
def _selftest():
    """无 CAN 自检: 编码→解码 round-trip + 几个向量, 顺带打印帧。"""
    print("=== 编解码自检 (不连 CAN) ===")
    cases = [(0.0, 0.0), (0.3, 5.0), (0.4, -25.0), (0.123, 12.34), (1.5, 25.0)]
    ok = True
    for sp, st in cases:
        data = encode_ctrl_cmd(sp, st, max_speed=ABS_MAX_SPEED)  # 自检放开到天花板看编码
        dec = decode_ctrl_cmd(data)
        exp_sp = min(sp, ABS_MAX_SPEED)
        sp_err = abs(dec["speed_mps"] - exp_sp)
        st_err = abs(dec["steer_deg"] - max(-25.0, min(25.0, st)))
        good = dec["bcc_ok"] and sp_err <= 0.001 and st_err <= 0.01
        ok = ok and good
        print("  in v=%.3f st=%+.2f -> %s -> v=%.3f st=%+.2f bcc=%s %s" %
              (sp, st, hexstr(data), dec["speed_mps"], dec["steer_deg"],
               dec["bcc_ok"], "OK" if good else "**FAIL**"))
    print("自检", "通过 ✅" if ok else "失败 ❌")
    return ok


def _teleop(ctl):
    """WASD 台架遥控 (车轮架空时用): w/s 加减速, a/d 左右转, 空格停, q 退出。"""
    import termios, tty, select
    speed, steer = 0.0, 0.0
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print("台架遥控: w/s=±速度 a/d=±转向(fr原生:负=右) 空格=停 q=退出")
    try:
        tty.setcbreak(fd)
        while True:
            ctl.set_cmd(speed, steer)
            r, _, _ = select.select([sys.stdin], [], [], PERIOD)
            if r:
                c = sys.stdin.read(1)
                if c == 'q':
                    break
                elif c == 'w':
                    speed += 0.05
                elif c == 's':
                    speed -= 0.05
                elif c == 'a':
                    steer -= 2.0
                elif c == 'd':
                    steer += 2.0
                elif c == ' ':
                    speed, steer = 0.0, 0.0
                speed = _clamp(speed, 0.0, ctl.max_speed)
                steer = _clamp(steer, -MAX_STEER, MAX_STEER)
                print("  期望 v=%.2f steer=%+.1f" % (speed, steer))
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    ap = argparse.ArgumentParser(description="fr 小车 CAN 控制 (ctrl_cmd_98c4d2d0)")
    ap.add_argument("--arm", action="store_true",
                    help="真发帧到 CAN (默认 dry-run 只打印)。务必车轮架空 + 人在场!")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--max-speed", type=float, default=MAX_SPEED,
                    help="硬限速 m/s (绝对天花板 %.2f)" % ABS_MAX_SPEED)
    ap.add_argument("--teleop", action="store_true", help="WASD 台架遥控模式")
    ap.add_argument("--demo", action="store_true", help="跑一段固定指令演示(默认dry-run)")
    ap.add_argument("--seconds", type=float, default=3.0)
    args = ap.parse_args()

    if not args.arm and not args.teleop and not args.demo:
        _selftest()
        print("\n(只做了离线自检。--demo 演示 / --teleop 遥控 / --arm 真发帧)")
        return

    ctl = CarController(channel=args.channel, dry_run=not args.arm,
                        max_speed=args.max_speed)
    if args.arm:
        print("\n⚠⚠⚠ ARMED: 即将真发控制帧到 %s。确认车轮架空、人在场带急停。" % args.channel)
        print("    3 秒后开始, Ctrl-C 立即安全停...")
        try:
            time.sleep(3)
        except KeyboardInterrupt:
            print("已取消。"); return
    ctl.start()
    try:
        if args.teleop:
            _teleop(ctl)
        else:  # demo: 直行→左偏→右偏→停
            t0 = time.monotonic()
            while time.monotonic() - t0 < args.seconds:
                ctl.set_cmd(0.3, 0.0); time.sleep(args.seconds / 3.0)
                ctl.set_cmd(0.3, -8.0); time.sleep(args.seconds / 3.0)
                ctl.set_cmd(0.3, 8.0); time.sleep(args.seconds / 3.0)
                break
    except KeyboardInterrupt:
        print("\nCtrl-C 急停。")
    finally:
        ctl.stop()


if __name__ == "__main__":
    main()
