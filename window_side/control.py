#!/usr/bin/env python3
"""
robot_control.py  —  PC side  (combined DDSM + Servo/Arm controller)

LEFT PANEL  — DDSM115 wheel motors (UDP float pair to Pi → ROS2 cmd_vel)
RIGHT PANEL — Servo arm (UDP 0xAA+JSON to Pi → Arduino Mega serial)

UDP ports — fully separated, ESP32 has priority:
  Port 3390  struct('ff') / struct('Bb') → ESP32  (motion + lock)  ★ PRIORITY ★
  Port 3391  bytes([0xAA])+JSON          → Mega   (servo/arm/posture)

──────────────────────────────────────────────────────────────
  WHEEL MOTOR KEYS
  W A S D Q E Z C   — movement
  X                 — brake toggle
  < >               — speed decrease / increase

  ARM / SERVO KEYS
  Y  — Front motor  (servo 1)
  U  — Back motor   (servo 2)
  I  — Arm 1        (servo 3)
  O  — Arm 2        (servo 4)
  H  — Arm 3        (servo 5)
  J  — Arm 4        (servo 6)
  K  — Arm 5        (servo 7)
  L  — Gripper motor(servo 8)  ← special

  -  (minus)  → select servo: decrease angle  |  gripper: REVERSE RAMP (45)
  =  (equals) → select servo: increase angle  |  gripper: GRIP RAMP    (140)
  SPACE       → gripper motor STOP instantly  (90)  — works any time

  1 2 3 4 5         → nudge selected servo +1° +2° +3° +4° +5°
  ! @ # $ %         → nudge selected servo -1° -2° -3° -4° -5°  (Shift+1-5)

  F1 home  F2 horizontal  F3 guard  F4 giraff  F5 stair

  `  — quit
──────────────────────────────────────────────────────────────
"""

import curses
import socket
import struct
import serial
import serial.tools.list_ports
import json
import time
import threading
import sys
import os
from pynput import keyboard as pynput_kb

# ═══════════════════════════════════════════════════════════════
#  Configuration
# ═══════════════════════════════════════════════════════════════
PI_IP         = "192.168.1.101"   # ← Your Pi IP

ESP32_PORT    = 3390   # motion + lock  → robot_udp.py → ESP32 via ROS2  ★ PRIORITY ★
MEGA_PORT_UDP = 3391   # servo/arm JSON → mega_udp.py  → Arduino Mega serial

COM_PORT      = None              # ESP32 serial — set to e.g. "COM3" or leave None
BAUD_RATE     = 115200
LINEAR_SPEED  = 0.3               # m/s
ANGULAR_SPEED = 0.8               # rad/s
BASE_RPM      = 80
SERVO_STEP    = 1                 # degrees per - / = press (non-motor servos)
MOTOR_RAMP_STEP = 5               # degrees per ramp tick for gripper
MOTOR_RAMP_HZ   = 20             # ramp ticks per second
# ═══════════════════════════════════════════════════════════════

# ── Packet markers ────────────────────────────────────────────
SERVO_MARKER = 0xAA
LOCK_MARKER  = 0xFF

# ── DDSM constants ────────────────────────────────────────────
CMD_DDSM_STOP = 10000
CMD_DDSM_CTRL = 10010
ACT_SPEED     = 2
CMD_DELAY     = 0.005

ESP32_KEYWORDS = ['CP210', 'CH340', 'FTDI', 'USB Serial', 'ESP32', 'Silicon Labs']

MOVE_KEYS = {'w', 'a', 's', 'd', 'q', 'e', 'z', 'c'}

MOTOR_LABELS = {
    'w': 'Forward',
    's': 'Backward',
    'a': 'Left',
    'd': 'Right',
    'q': 'Fwd-Left  (diag)',
    'e': 'Fwd-Right (diag)',
    'z': 'Bwd-Left  (diag)',
    'c': 'Bwd-Right (diag)',
}

# ── Servo arm config ─────────────────────────────────────────
NUM_SERVOS = 8

SERVO_KEYS = {
    'y': 0,   # front
    'u': 1,   # back
    'i': 2,   # arm1
    'o': 3,   # arm2
    'h': 4,   # arm3
    'j': 5,   # arm4
    'k': 6,   # arm5
    'l': 7,   # gripper motor (special)
}
KEY_SERVO = {v: k for k, v in SERVO_KEYS.items()}

SERVO_NAMES = [
    "Front motor",
    "Back motor",
    "Arm 1",
    "Arm 2",
    "Arm 3",
    "Arm 4",
    "Arm 5",
    "Gripper motor",
]
# Default angles match sethorizontal() posture in Arduino sketch
SERVO_DEFAULTS = [85, 115, 105, 180, 0, 90, 110, 90]
SERVO_MINS     = [  0,   0,  0,  100,  0,  45,   60, 45]
SERVO_MAXS     = [180, 180,105, 180,180,125, 180,140]
MOTOR_STOP    = 90
MOTOR_GRIP    = 140
MOTOR_REVERSE = 45

POSTURE_KEYS = {
    pynput_kb.Key.f1: "home",
    pynput_kb.Key.f2: "horizontal",
    pynput_kb.Key.f3: "guard",
    pynput_kb.Key.f4: "giraff",
    pynput_kb.Key.f5: "stair",
}

# ══════════════════════════════════════════════════════════════
#  Color IDs
# ══════════════════════════════════════════════════════════════
C_TITLE=1; C_KEY=2; C_VAL=3; C_LOCK=4; C_DIM=5
C_ACT=6;   C_BDR=7; C_LOG=8; C_GOOD=9; C_WARN=10; C_DANGER=11

def init_colors():
    curses.start_color(); curses.use_default_colors()
    curses.init_pair(C_TITLE,  curses.COLOR_CYAN,   -1)
    curses.init_pair(C_KEY,    curses.COLOR_WHITE,  -1)
    curses.init_pair(C_VAL,    curses.COLOR_YELLOW, -1)
    curses.init_pair(C_LOCK,   curses.COLOR_RED,    -1)
    curses.init_pair(C_DIM,    8,                   -1)
    curses.init_pair(C_ACT,    curses.COLOR_BLACK,  curses.COLOR_CYAN)
    curses.init_pair(C_BDR,    curses.COLOR_BLUE,   -1)
    curses.init_pair(C_LOG,    8,                   -1)
    curses.init_pair(C_GOOD,   curses.COLOR_GREEN,  -1)
    curses.init_pair(C_WARN,   curses.COLOR_YELLOW, -1)
    curses.init_pair(C_DANGER, curses.COLOR_RED,    -1)

def P(n, bold=False, rev=False):
    a = curses.color_pair(n)
    if bold: a |= curses.A_BOLD
    if rev:  a |= curses.A_REVERSE
    return a

def put(win, y, x, text, attr=0):
    h, w = win.getmaxyx()
    if not (0 <= y < h): return
    if x >= w: return
    if x < 0: text = text[-x:]; x = 0
    avail = w - x
    if avail <= 0: return
    try: win.addstr(y, x, text[:avail], attr)
    except curses.error: pass

def draw_box(win, y, x, h, w, title=""):
    b = P(C_BDR, bold=True)
    put(win, y,     x, "╔"+"═"*(w-2)+"╗", b)
    put(win, y+h-1, x, "╚"+"═"*(w-2)+"╝", b)
    for r in range(y+1, y+h-1):
        put(win, r, x,     "║", b)
        put(win, r, x+w-1, "║", b)
    if title:
        tx = x + max(1, (w-len(title)-2)//2)
        put(win, y, tx, f" {title} ", P(C_TITLE, bold=True))

def key_row(win, y, x, key_label, desc, active=False):
    lbl = f" {key_label.upper():2s} "
    if active:
        put(win, y, x,   lbl,          P(C_ACT, bold=True))
        put(win, y, x+4, f"● {desc}",  P(C_ACT))
    else:
        put(win, y, x,   lbl,          P(C_KEY, bold=True))
        put(win, y, x+4, f"○ {desc}",  P(C_DIM))


# ══════════════════════════════════════════════════════════════
#  COMBINED CONTROLLER
# ══════════════════════════════════════════════════════════════
def find_esp32_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        desc = (p.description or '') + (p.manufacturer or '')
        if any(kw.lower() in desc.lower() for kw in ESP32_KEYWORDS):
            return p.device
    return ports[0].device if len(ports) == 1 else None


class RobotController:
    def __init__(self):
        self._mu       = threading.Lock()
        self._log_mu   = threading.Lock()
        self.running   = True
        self.quit_flag = False

        # ── Wheel motor state ─────────────────────────────────
        self.active_keys   = set()
        self.is_locked     = False
        self.linear_speed  = LINEAR_SPEED
        self.angular_speed = ANGULAR_SPEED
        self.base_rpm      = BASE_RPM
        self.serial_ok     = False

        # ── Servo arm state ───────────────────────────────────
        self.angles       = list(SERVO_DEFAULTS)
        self.selected     = 0
        self.servo_step   = 1                    # degrees per - / = (set by 1-5 keys)
        self.motor_state  = "stop"               # "stop"|"grip"|"reverse"
        self._ramp_thread = None                 # gripper ramp thread
        self._ramp_target = MOTOR_STOP           # where ramp is heading

        self.log_lines = []

        # UDP socket (shared for both wheel and servo packets)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4096)

        # Serial to ESP32 (optional — DDSM direct serial)
        self.ser = None
        port = COM_PORT or find_esp32_port()
        if port:
            try:
                self.ser = serial.Serial(port, BAUD_RATE, timeout=1)
                time.sleep(1.5)
                self.serial_ok = True
                self._log(f"Serial OK → {port}")
                threading.Thread(target=self._serial_rx, daemon=True).start()
            except serial.SerialException as e:
                self._log(f"Serial failed: {e}")
        else:
            self._log("No serial found — UDP only mode")

        # Wheel motor send loop (50 Hz)
        threading.Thread(target=self._send_loop, daemon=True).start()

    # ── Logging ───────────────────────────────────────────────
    def _log(self, msg):
        ts = time.strftime("%H:%M:%S")
        with self._log_mu:
            self.log_lines.append(f"{ts}  {msg}")
            if len(self.log_lines) > 30:
                self.log_lines.pop(0)

    # ══════════════════════════════════════════════════════════
    #  WHEEL MOTOR  (UDP float pair + optional serial DDSM)
    # ══════════════════════════════════════════════════════════
    def _udp_motion(self, lin, ang):
        try:
            self.sock.sendto(struct.pack('ff', lin, ang), (PI_IP, ESP32_PORT))
        except Exception:
            pass

    def _udp_lock(self, state):
        try:
            self.sock.sendto(struct.pack('Bb', LOCK_MARKER, state), (PI_IP, ESP32_PORT))
        except Exception:
            pass

    def _serial_json(self, cmd):
        if not self.ser: return
        try:
            self.ser.write((json.dumps(cmd) + '\n').encode())
        except Exception:
            pass

    def _serial_motor(self, mid, rpm):
        self._serial_json({"T": CMD_DDSM_CTRL, "id": mid, "cmd": rpm, "act": ACT_SPEED})
        time.sleep(CMD_DELAY)

    def _serial_drive(self, l, r):
        self._serial_motor(1,  l); self._serial_motor(3,  l)
        self._serial_motor(2, -r); self._serial_motor(4, -r)

    def _serial_stop_all(self):
        for mid in [1, 2, 3, 4]:
            self._serial_json({"T": CMD_DDSM_STOP, "id": mid})
            time.sleep(CMD_DELAY)

    def _serial_rx(self):
        while self.running and self.ser:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode(errors='ignore').strip()
                    if line:
                        self._log(f"ESP32: {line}")
            except Exception:
                break
            time.sleep(0.01)

    def do_lock(self):
        self.is_locked = True
        self._udp_motion(0.0, 0.0)
        self._udp_lock(1)
        self._log("BRAKE ENGAGED")

    def do_unlock(self):
        self.is_locked = False
        self._udp_lock(0)
        self._log("BRAKE RELEASED")

    def _calc_vel(self):
        lin = self.linear_speed
        ang = self.angular_speed
        rpm = self.base_rpm
        k   = self.active_keys
        lv = av = lr = rr = 0
        if   'w' in k: lv= lin; lr= rpm;     rr= rpm
        elif 's' in k: lv=-lin; lr=-rpm;     rr=-rpm
        elif 'a' in k: av= ang; lr=-rpm;     rr= rpm
        elif 'd' in k: av=-ang; lr= rpm;     rr=-rpm
        elif 'q' in k: lv= lin; av= ang; lr=rpm//2; rr= rpm
        elif 'e' in k: lv= lin; av=-ang; lr=rpm;    rr=rpm//2
        elif 'z' in k: lv=-lin; av= ang; lr=-rpm//2;rr=-rpm
        elif 'c' in k: lv=-lin; av=-ang; lr=-rpm;   rr=-rpm//2
        return lv, av, int(lr), int(rr)

    def _send_loop(self):
        while self.running:
            with self._mu:
                locked = self.is_locked
                lv, av, lr, rr = self._calc_vel()
            if locked:
                self._udp_lock(1)
                time.sleep(0.1)
            else:
                self._udp_motion(lv, av)
                if self.ser and (lr or rr):
                    self._serial_drive(lr, rr)
                time.sleep(0.02)

    # ══════════════════════════════════════════════════════════
    #  SERVO ARM  (UDP 0xAA+JSON)
    # ══════════════════════════════════════════════════════════
    def _udp_servo(self, payload: dict):
        try:
            data = bytes([SERVO_MARKER]) + json.dumps(payload).encode()
            self.sock.sendto(data, (PI_IP, MEGA_PORT_UDP))
        except Exception as e:
            self._log(f"Servo UDP error: {e}")

    def _send_servo(self, idx, angle):
        self._udp_servo({"cmd": "servo", "id": idx + 1, "angle": angle})

    # ── Gripper motor — instant stop ──────────────────────────
    def _motor_stop_instant(self):
        """Stop ramp thread and write 90 immediately."""
        self._ramp_target = MOTOR_STOP   # signal ramp thread to abort
        with self._mu:
            self.motor_state = "stop"
            self.angles[7]   = MOTOR_STOP
        self._udp_servo({"cmd": "motor", "state": "stop"})
        self._log("Motor STOP (90) — instant")

    # ── Gripper motor — smooth ramp ───────────────────────────
    def _start_ramp(self, target: int, state_name: str):
        """
        Start a background thread that ramps the gripper angle toward `target`
        by MOTOR_RAMP_STEP degrees every tick.
        Sets motor_state immediately so UI reflects the intent right away.
        A running ramp is cancelled by changing _ramp_target before it reaches goal.
        """
        self._ramp_target = target
        with self._mu:
            self.motor_state = state_name

        def _ramp():
            interval = 1.0 / MOTOR_RAMP_HZ
            while self.running:
                # Check if we've been cancelled or redirected
                if self._ramp_target != target:
                    break
                with self._mu:
                    current = self.angles[7]
                if current == target:
                    break
                # Step toward target
                if current < target:
                    new = min(current + MOTOR_RAMP_STEP, target)
                else:
                    new = max(current - MOTOR_RAMP_STEP, target)
                with self._mu:
                    # Double-check target hasn't changed while we computed
                    if self._ramp_target != target:
                        break
                    self.angles[7] = new
                # Send current ramp position as a direct angle servo command
                # (Arduino motor safety still enforces 90 passthrough on its side)
                self._udp_servo({"cmd": "servo", "id": 8, "angle": new})
                time.sleep(interval)

        self._ramp_thread = threading.Thread(target=_ramp, daemon=True)
        self._ramp_thread.start()

    def _motor_grip(self):
        """Ramp toward 140 (grip). Passes through 90 handled by Arduino."""
        self._log(f"Motor GRIP ramp → {MOTOR_GRIP}")
        self._start_ramp(MOTOR_GRIP, "grip")

    def _motor_reverse(self):
        """Ramp toward 45 (reverse grip). Passes through 90 handled by Arduino."""
        self._log(f"Motor REVERSE ramp → {MOTOR_REVERSE}")
        self._start_ramp(MOTOR_REVERSE, "reverse")

    def _send_posture(self, name):
        # Stop gripper before any posture change
        self._motor_stop_instant()
        self._udp_servo({"cmd": "posture", "name": name})
        # Update local angle display to match known posture angles
        posture_angles = {
            "home":       [85, 115,   105, 180,   0,  90, 110, 90],
            "horizontal": [85, 115,   105, 180,   0,  90, 110, 90],
            "guard":      None,   # placeholder — unknown yet
            "giraff":     None,
            "stair":      None,
        }
        with self._mu:
            angles = posture_angles.get(name)
            if angles:
                self.angles = list(angles)
        self._log(f"Posture → {name}")

    # ══════════════════════════════════════════════════════════
    #  KEY EVENTS
    # ══════════════════════════════════════════════════════════
    def on_press(self, k):
        # ── Quit ──────────────────────────────────────────────
        if k in ('`', 'esc'):
            self.quit_flag = True
            return

        # ── Speed up / down (wheel motors) ───────────────────
        if k in ('>', '.'):
            # '.' conflicts with motor grip — only treat as speed-up
            # when NO servo key is selected as active AND motor is stopped
            # Actually: '.' is grip key, handled below — skip speed here
            # Speed up uses '>' only
            pass
        if k == '>':
            with self._mu:
                self.linear_speed  = min(1.5, round(self.linear_speed  * 1.15, 3))
                self.angular_speed = min(3.0, round(self.angular_speed * 1.15, 3))
                self.base_rpm      = min(200, int(self.base_rpm * 1.15))
            self._log(f"Speed UP  lin={self.linear_speed:.2f} ang={self.angular_speed:.2f} rpm={self.base_rpm}")
            return
        if k == '<':
            with self._mu:
                self.linear_speed  = max(0.05, round(self.linear_speed  * 0.85, 3))
                self.angular_speed = max(0.1,  round(self.angular_speed * 0.85, 3))
                self.base_rpm      = max(10,   int(self.base_rpm * 0.85))
            self._log(f"Speed DN  lin={self.linear_speed:.2f} ang={self.angular_speed:.2f} rpm={self.base_rpm}")
            return

        # ── Brake ─────────────────────────────────────────────
        if k == 'x':
            with self._mu:
                if self.is_locked:
                    self.do_unlock()
                else:
                    self.active_keys.clear()
                    self.do_lock()
            return

        # ── Wheel movement ─────────────────────────────────────
        if k in MOVE_KEYS:
            with self._mu:
                if self.is_locked:
                    self.do_unlock()
                self.active_keys.add(k)
            return

        # ── Step size: 1-5 sets how many degrees - / = moves ─
        if k in ('1','2','3','4','5'):
            with self._mu:
                self.servo_step = int(k)
            self._log(f"Step → {k}°  (affects - and =)")
            return

        # ── Select servo ──────────────────────────────────────
        if k in SERVO_KEYS:
            with self._mu:
                self.selected = SERVO_KEYS[k]
            self._log(f"Selected {SERVO_NAMES[self.selected]}")
            return

        # ── SPACEBAR — gripper motor instant stop ─────────────
        if k == ' ':
            self._motor_stop_instant()
            return

        # ── = / + — increase angle OR start grip ramp ─────────
        if k in ('=', '+'):
            with self._mu:
                idx  = self.selected
                step = self.servo_step
            if idx == 7:
                self._motor_grip()
            else:
                with self._mu:
                    new = min(SERVO_MAXS[idx], self.angles[idx] + step)
                    self.angles[idx] = new
                self._send_servo(idx, new)
                self._log(f"{SERVO_NAMES[idx]} → {new}°")
            return

        # ── - — decrease angle OR start reverse ramp ──────────
        if k == '-':
            with self._mu:
                idx  = self.selected
                step = self.servo_step
            if idx == 7:
                self._motor_reverse()
            else:
                with self._mu:
                    new = max(SERVO_MINS[idx], self.angles[idx] - step)
                    self.angles[idx] = new
                self._send_servo(idx, new)
                self._log(f"{SERVO_NAMES[idx]} → {new}°")
            return

    def on_press_special(self, key):
        if key in POSTURE_KEYS:
            self._send_posture(POSTURE_KEYS[key])

    def on_release(self, k):
        if k in MOVE_KEYS:
            with self._mu:
                self.active_keys.discard(k)
                if not self.active_keys and not self.is_locked:
                    self._udp_motion(0.0, 0.0)
                    if self.ser:
                        threading.Thread(target=self._serial_stop_all, daemon=True).start()

    # ══════════════════════════════════════════════════════════
    #  SNAPSHOT  (UI reads this — no blocking)
    # ══════════════════════════════════════════════════════════
    def snapshot(self):
        with self._mu:
            lv, av, lr, rr = self._calc_vel()
            return dict(
                # wheel
                locked    = self.is_locked,
                keys      = set(self.active_keys),
                lv=lv, av=av, lr=lr, rr=rr,
                lin_spd   = self.linear_speed,
                ang_spd   = self.angular_speed,
                rpm_spd   = self.base_rpm,
                serial    = self.serial_ok,
                # servo
                angles      = list(self.angles),
                selected    = self.selected,
                motor_state = self.motor_state,
                servo_step  = self.servo_step,
                # shared
                log = list(self.log_lines),
            )

    def cleanup(self):
        self.running = False
        self._ramp_target = MOTOR_STOP   # stop any ramp
        if self.is_locked:
            self.do_unlock()
        self._udp_motion(0.0, 0.0)
        if self.ser:
            self._serial_stop_all()
            self.ser.close()
        self.sock.close()


# ══════════════════════════════════════════════════════════════
#  RENDER
# ══════════════════════════════════════════════════════════════
def angle_color(idx, angle):
    lo = SERVO_MINS[idx]; hi = SERVO_MAXS[idx]
    pct = (angle - lo) / max(1, hi - lo)
    if pct <= 0.05 or pct >= 0.95: return P(C_DANGER, bold=True)
    if pct <= 0.12 or pct >= 0.88: return P(C_WARN,   bold=True)
    return P(C_GOOD, bold=True)

def angle_bar(angle, lo, hi, width=14):
    filled = int((angle - lo) / max(1, hi - lo) * width)
    filled = max(0, min(width, filled))
    return "█"*filled + "░"*(width-filled)

def motor_color(state):
    if state == "grip":    return P(C_GOOD,   bold=True)
    if state == "reverse": return P(C_WARN,   bold=True)
    return P(C_DIM)


def render(stdscr, ctrl):
    s = ctrl.snapshot()
    H, W = stdscr.getmaxyx()
    stdscr.erase()

    if H < 28 or W < 80:
        put(stdscr, 0, 0,
            f"Terminal too small {W}x{H}. Need at least 80x28.",
            P(C_LOCK, bold=True))
        stdscr.refresh(); return

    MID    = W // 2
    LW     = MID
    RW     = W - MID
    PANEL_H = H - 7

    # ── Header ────────────────────────────────────────────────
    put(stdscr, 0, 0, " "*W, P(C_BDR, rev=True))
    put(stdscr, 0, 1, " ROBOT CONTROL ", P(C_TITLE, bold=True, rev=True))
    mode = "SERIAL + UDP" if s['serial'] else "UDP ONLY"
    mc   = P(C_GOOD, bold=True) if s['serial'] else P(C_WARN, bold=True)
    put(stdscr, 0, 17, f" {mode} ", mc | curses.A_REVERSE)
    pi_str = f" Pi: {PI_IP}  ESP32:{ESP32_PORT}  Mega:{MEGA_PORT_UDP} "
    put(stdscr, 0, W-len(pi_str)-1, pi_str, P(C_DIM))

    # Vertical divider
    for r in range(1, H-6):
        put(stdscr, r, MID, "│", P(C_BDR, bold=True))

    # ══════════════════════════════════════════════════════════
    #  LEFT PANEL — Wheel Motor Control
    # ══════════════════════════════════════════════════════════
    draw_box(stdscr, 1, 0, PANEL_H, LW, "Control Motor")
    row = 3
    for k, lbl in MOTOR_LABELS.items():
        key_row(stdscr, row, 2, k, lbl, active=(k in s['keys'] and not s['locked']))
        row += 1

    row += 1
    put(stdscr, row, 2, "─"*(LW-4), P(C_BDR)); row += 1
    key_row(stdscr, row,   2, '<', 'Decrease Speed')
    key_row(stdscr, row+1, 2, '>', 'Increase Speed')
    row += 3
    put(stdscr, row, 2, "─"*(LW-4), P(C_BDR)); row += 1

    # Brake row
    if s['locked']:
        put(stdscr, row, 2, " X  ", P(C_LOCK, bold=True, rev=True))
        put(stdscr, row, 6, "● Brake / Lock Motor  [ACTIVE]", P(C_LOCK, bold=True))
    else:
        put(stdscr, row, 2, " X  ", P(C_KEY, bold=True))
        put(stdscr, row, 6, "○ Brake / Lock Motor", P(C_DIM))

    # ══════════════════════════════════════════════════════════
    #  RIGHT PANEL — Servo Arm
    # ══════════════════════════════════════════════════════════
    draw_box(stdscr, 1, MID, PANEL_H, RW, "Arm / Gripper")

    row = 2
    # Column header
    put(stdscr, row, MID+2, "Key  Servo", P(C_KEY, bold=True))
    bar_w = max(8, RW - 30)
    put(stdscr, row, MID+14, "Angle", P(C_KEY, bold=True))
    put(stdscr, row, MID+14+bar_w+4, "Val", P(C_KEY, bold=True))
    row += 1

    for i in range(NUM_SERVOS):
        angle  = s['angles'][i]
        lo     = SERVO_MINS[i]
        hi     = SERVO_MAXS[i]
        is_sel = (s['selected'] == i)
        k_lbl  = KEY_SERVO.get(i, ' ').upper()
        name   = SERVO_NAMES[i]

        # Key badge
        if is_sel:
            put(stdscr, row, MID+2, f" {k_lbl} ", P(C_ACT, bold=True))
        else:
            put(stdscr, row, MID+2, f" {k_lbl} ", P(C_KEY, bold=True))

        # Servo name — always plain white, never covered by highlight
        put(stdscr, row, MID+6, f"{name:<13s}", P(C_KEY))

        if i == 7:
            # Gripper motor — show ramp bar + state
            ms   = s['motor_state']
            bar  = angle_bar(angle, lo, hi, bar_w)
            mcol = motor_color(ms)
            put(stdscr, row, MID+14, f"[{bar}]", mcol)
            put(stdscr, row, MID+14+bar_w+2, f"{angle:3d}°", mcol)
            put(stdscr, row, MID+14+bar_w+7, f"[{ms.upper():7s}]", mcol)
        else:
            # Regular servo — show angle bar
            bar  = angle_bar(angle, lo, hi, bar_w)
            acol = angle_color(i, angle) if not is_sel else P(C_GOOD, bold=True)
            bcol = P(C_VAL)
            put(stdscr, row, MID+14, f"[{bar}]", bcol)
            put(stdscr, row, MID+14+bar_w+2, f"{angle:3d}°", acol)
            put(stdscr, row, MID+14+bar_w+7, f"{lo}–{hi}", P(C_DIM))

        row += 1

    # Separator + control hints
    row += 1
    put(stdscr, row, MID+2, "─"*(RW-4), P(C_BDR)); row += 1

    step_now = s['servo_step']
    put(stdscr, row,   MID+2, " -  ", P(C_KEY, bold=True))
    put(stdscr, row,   MID+6, f"○ Decrease {step_now}° / REVERSE ramp", P(C_DIM))
    put(stdscr, row+1, MID+2, " =  ", P(C_KEY, bold=True))
    put(stdscr, row+1, MID+6, f"○ Increase {step_now}° / GRIP ramp", P(C_DIM))

    # Step selector — highlight active
    put(stdscr, row+2, MID+2, "1-5 ", P(C_KEY, bold=True))
    put(stdscr, row+2, MID+6, "Step: ", P(C_DIM))
    offset = MID + 12
    for n in range(1, 6):
        col = P(C_ACT, bold=True) if n == step_now else P(C_KEY)
        put(stdscr, row+2, offset, f"[{n}]", col)
        offset += 4

    row += 4
    put(stdscr, row,   MID+2, "SPC ", P(C_KEY, bold=True))
    put(stdscr, row,   MID+6, "○ Motor STOP instant (90)", P(C_DIM))
    row += 2
    put(stdscr, row, MID+2, "─"*(RW-4), P(C_BDR)); row += 1

    # Posture keys
    put(stdscr, row, MID+2, "Postures:", P(C_KEY, bold=True)); row += 1
    for pk, pname in [("F1","home"),("F2","horizontal"),("F3","guard"),("F4","giraff"),("F5","stair")]:
        put(stdscr, row, MID+2, f" {pk} ", P(C_KEY, bold=True))
        put(stdscr, row, MID+7, f"○ {pname}", P(C_DIM))
        row += 1

    row += 1
    put(stdscr, row, MID+2, " `  ", P(C_KEY, bold=True))
    put(stdscr, row, MID+6, "○ Quit", P(C_DIM))

    # ══════════════════════════════════════════════════════════
    #  STATUS BAR
    # ══════════════════════════════════════════════════════════
    sy = H - 6
    put(stdscr, sy, 0, "═"*W, P(C_BDR, bold=True))

    # Wheel status
    if s['locked']:
        state_str = "BRAKE"; state_col = P(C_LOCK, bold=True)
    elif s['keys']:
        state_str = "MOVING"; state_col = P(C_GOOD, bold=True)
    else:
        state_str = "IDLE";  state_col = P(C_DIM)

    put(stdscr, sy+1, 2, "Wheels : ", P(C_KEY, bold=True))
    put(stdscr, sy+1, 11, state_str, state_col)
    spd = (f"Lin={s['lin_spd']:.2f} m/s   "
           f"Ang={s['ang_spd']:.2f} rad/s   "
           f"RPM={s['rpm_spd']}")
    put(stdscr, sy+1, 11+len(state_str)+3, spd, P(C_VAL))

    # RPM bar
    max_rpm = max(1, BASE_RPM)
    bar_w16 = 16
    lf = min(bar_w16, abs(s['lr'])*bar_w16 // max_rpm)
    rf = min(bar_w16, abs(s['rr'])*bar_w16 // max_rpm)
    bc = P(C_VAL) if s['keys'] else P(C_DIM)
    vel_ln = (f"L:[{'█'*lf}{'░'*(bar_w16-lf)}]{s['lr']:+4d}  "
              f"R:[{'█'*rf}{'░'*(bar_w16-rf)}]{s['rr']:+4d} RPM")
    put(stdscr, sy+2, 2, vel_ln[:W//2-2], bc)

    # Servo status (right side of status bar)
    sel = s['selected']
    put(stdscr, sy+2, W//2+2, "Arm : ", P(C_KEY, bold=True))
    if sel == 7:
        ms = s['motor_state']
        put(stdscr, sy+2, W//2+8,
            f"{SERVO_NAMES[sel]}  [{ms.upper()}]  stop=90 grip=140 rev=45",
            motor_color(ms))
    else:
        put(stdscr, sy+2, W//2+8,
            f"{SERVO_NAMES[sel]}  {s['angles'][sel]}°  "
            f"range {SERVO_MINS[sel]}–{SERVO_MAXS[sel]}°",
            angle_color(sel, s['angles'][sel]))

    # Latest log
    if s['log']:
        put(stdscr, sy+3, 2, s['log'][-1][:W-4], P(C_LOG))

    put(stdscr, H-1, 0, "═"*W, P(C_BDR, bold=True))
    stdscr.refresh()


# ══════════════════════════════════════════════════════════════
#  PYNPUT BRIDGE
# ══════════════════════════════════════════════════════════════
def make_handlers(ctrl):
    def on_press(key):
        if hasattr(key, 'char') and key.char is not None:
            c = key.char.lower()
            if c:
                ctrl.on_press(c)
        else:
            if key == pynput_kb.Key.space:
                ctrl.on_press(' ')
            elif key == pynput_kb.Key.esc:
                ctrl.on_press('esc')
            else:
                ctrl.on_press_special(key)

    def on_release(key):
        if hasattr(key, 'char') and key.char is not None:
            c = key.char.lower()
            if c:
                ctrl.on_release(c)

    return on_press, on_release


# ══════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════
def tui_main(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(False)
    init_colors()

    ctrl = RobotController()
    on_press, on_release = make_handlers(ctrl)
    listener = pynput_kb.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        while not ctrl.quit_flag:
            render(stdscr, ctrl)
            try:
                ch = stdscr.getch()
                if ch == 27:
                    break
            except curses.error:
                pass
            time.sleep(0.033)
    finally:
        listener.stop()
        ctrl.cleanup()


def main():
    if sys.platform == 'win32':
        os.system('chcp 65001 > nul 2>&1')
    try:
        curses.wrapper(tui_main)
    except KeyboardInterrupt:
        pass
    print("\nRobot control stopped.")


if __name__ == '__main__':
    main()
