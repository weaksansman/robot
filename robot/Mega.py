#!/usr/bin/env python3
import socket
import struct
import threading
import subprocess
import serial
import glob
import sys
import os
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8


# ═══════════════════════════════════════════════════════════════
#  ★ SET THESE MANUALLY ★
#
#  Not sure which port?  Run: ls -l /dev/ttyA* or ls -l /dev/ttyU*
#  IGNORE ttyAMA0 — that is the Pi's own UART, not USB.
#
#  Common ports:
#    /dev/ttyUSB0   ← CP2102 / CH340 chips  (most ESP32 devkit boards)
#    /dev/ttyACM0   ← ESP32-S3 native USB, or Arduino-style boards
#
# ═══════════════════════════════════════════════════════════════
MEGA_PORT  = "/dev/ttyUSB0"   # ← Arduino Mega 2560  (servo JSON forwarded here)
BAUD_RATE  = 115200           # Do not change — must match both boards

# ── micro-ROS agent ───────────────────────────────────────────
LAUNCH_MICROROS_AGENT = True

# ── UDP ───────────────────────────────────────────────────────
LISTEN_IP   = "0.0.0.0"
LISTEN_PORT = 3390

# ── ROS2 topics ───────────────────────────────────────────────
CMD_VEL_TOPIC = "cmd_vel"
LOCK_TOPIC    = "/lock"

# ── Packet constants ──────────────────────────────────────────
MOTION_PACKET_SIZE = 8
LOCK_PACKET_SIZE   = 2
LOCK_MARKER        = 0xFF
SERVO_MARKER       = 0xAA


# ═══════════════════════════════════════════════════════════════
#  Arduino Mega serial connection
# ═══════════════════════════════════════════════════════════════
class MegaSerial:
    def __init__(self):
        self._ser  = None
        self._lock = threading.Lock()
        self._connect()

    def _connect(self):
        if not os.path.exists(MEGA_PORT):
            print(f"[MEGA]  WARN: {MEGA_PORT} not found — servo packets will be dropped")
            return
        try:
            self._ser = serial.Serial(MEGA_PORT, BAUD_RATE, timeout=1)
            time.sleep(1.5)
            print(f"[MEGA]  OK: Arduino Mega connected on {MEGA_PORT} @ {BAUD_RATE}")
        except serial.SerialException as e:
            print(f"[MEGA]  ERROR: {e}")

    def send(self, json_bytes: bytes):
        with self._lock:
            if self._ser is None:
                print("[MEGA]  WARN: not connected — servo packet dropped")
                return
            try:
                self._ser.write(json_bytes + b'\n')
            except serial.SerialException as e:
                print(f"[MEGA]  ERROR write: {e}")
                self._ser = None

    def close(self):
        with self._lock:
            if self._ser:
                self._ser.close()


# ═══════════════════════════════════════════════════════════════
#  --list helper
# ═══════════════════════════════════════════════════════════════
def _get_port_description(port):
    try:
        result = subprocess.run(
            ['udevadm', 'info', '--name=' + port, '--query=property'],
            capture_output=True, text=True, timeout=2
        )
        props = {}
        for line in result.stdout.splitlines():
            if '=' in line:
                k, v = line.split('=', 1)
                props[k] = v
        parts = []
        if 'ID_VENDOR'       in props: parts.append(props['ID_VENDOR'])
        if 'ID_MODEL'        in props: parts.append(props['ID_MODEL'])
        if 'ID_SERIAL_SHORT' in props: parts.append(f"S/N:{props['ID_SERIAL_SHORT']}")
        if parts:
            return ' | '.join(parts)
    except Exception:
        pass
    dev_name = port.split('/')[-1]
    try:
        base = f'/sys/class/tty/{dev_name}/device/..'
        vid  = open(f'{base}/idVendor').read().strip()
        pid  = open(f'{base}/idProduct').read().strip()
        return f"VID:0x{vid}  PID:0x{pid}"
    except Exception:
        pass
    return "(no description — try: udevadm info --name=" + port + ")"


def list_serial_ports():
    ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
    print()
    print("Serial ports currently available on this Pi:")
    print("─" * 62)
    if not ports:
        print("  (none found — are the boards plugged in?)")
    else:
        for port in ports:
            desc = _get_port_description(port)
            tag  = "  ← MEGA_PORT (servo/arm)" if port == MEGA_PORT else ""
            print(f"  {port:<20}  {desc}{tag}")
    print("─" * 62)
    print()
    print("Common chips:")
    print("  CP2102 / CP2104  →  usually /dev/ttyUSB*   (most ESP32 devkits)")
    print("  CH340 / CH341    →  usually /dev/ttyUSB*")
    print("  ESP32-S3 native  →  usually /dev/ttyACM*")
    print("  Arduino Mega     →  usually /dev/ttyUSB* or /dev/ttyACM*")
    print()
    print("Set MEGA_PORT at the top of this file.")
    print()


# ═══════════════════════════════════════════════════════════════
#  micro-ROS agent — now uses ACM0 hardcoded since that's ESP32
# ═══════════════════════════════════════════════════════════════
class MicroROSAgent:
    AGENT_PORT = "/dev/ttyACM0"

    def __init__(self):
        self._proc = None

    def start(self):
        cmd = f"micro-ros-agent serial --dev {self.AGENT_PORT} -b {BAUD_RATE}"
        print(f"[AGENT] Starting: {cmd}")
        try:
            self._proc = subprocess.Popen(
                cmd.split(),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True
            )
            threading.Thread(target=self._log_output, daemon=True).start()
            print(f"[AGENT] PID {self._proc.pid}  — waiting for ESP32...")
            time.sleep(2)
        except FileNotFoundError:
            print("[AGENT] ERROR: 'micro-ros-agent' command not found.")
            print("        Install:  sudo snap install micro-ros-agent")
            print("        Or set LAUNCH_MICROROS_AGENT = False and run agent manually.")

    def _log_output(self):
        for line in self._proc.stdout:
            line = line.rstrip()
            if line:
                print(f"[AGENT] {line}")

    def stop(self):
        if self._proc and self._proc.poll() is None:
            print("\n[AGENT] Stopping...")
            self._proc.terminate()
            try:
                self._proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._proc.kill()


# ═══════════════════════════════════════════════════════════════
#  ROS2 UDP bridge node
# ═══════════════════════════════════════════════════════════════
class UDPBridgeNode(Node):
    def __init__(self, mega: MegaSerial):
        super().__init__('udp_bridge')

        self._mega = mega

        self.cmd_pub  = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.lock_pub = self.create_publisher(Int8,  LOCK_TOPIC,    10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((LISTEN_IP, LISTEN_PORT))
        self.sock.settimeout(1.0)

        self.get_logger().info(f"ESP32 (micro-ROS) : /dev/ttyACM0 @ {BAUD_RATE} baud")
        self.get_logger().info(f"Mega port         : {MEGA_PORT}  @ {BAUD_RATE} baud")
        self.get_logger().info(f"UDP listening     : {LISTEN_IP}:{LISTEN_PORT}")
        self.get_logger().info(f"Publishing        : '{CMD_VEL_TOPIC}'  |  '{LOCK_TOPIC}'")

        self._last_recv = self.get_clock().now()
        self._is_locked = False

        self.create_timer(0.1, self._watchdog_cb)

        self._running = True
        threading.Thread(target=self._recv_loop, daemon=True).start()

    def _watchdog_cb(self):
        if self._is_locked:
            return
        elapsed = (self.get_clock().now() - self._last_recv).nanoseconds / 1e9
        if elapsed > 0.5:
            self.cmd_pub.publish(Twist())

    def _recv_loop(self):
        while self._running:
            try:
                data, addr = self.sock.recvfrom(512)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"UDP recv error: {e}")
                continue

            self._last_recv = self.get_clock().now()
            first = data[0] if data else None

            if first == SERVO_MARKER and len(data) > 1:
                json_bytes = data[1:]
                self._mega.send(json_bytes)
                self.get_logger().debug(f"[SERVO] → Mega: {json_bytes.decode(errors='replace')}")

            elif len(data) == LOCK_PACKET_SIZE and first == LOCK_MARKER:
                self._handle_lock(data)

            elif len(data) == MOTION_PACKET_SIZE:
                self._handle_motion(data)

            else:
                self.get_logger().warn(
                    f"Unknown packet: {len(data)} bytes  "
                    f"first=0x{first:02X}  from {addr[0]}:{addr[1]}"
                )

    def _handle_motion(self, data):
        linear, angular = struct.unpack('ff', data)
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def _handle_lock(self, data):
        marker, state = struct.unpack('Bb', data)
        if marker != LOCK_MARKER:
            self.get_logger().warn("Lock packet: wrong marker byte — ignored")
            return
        self._is_locked = bool(state)
        msg = Int8()
        msg.data = int(state)
        self.lock_pub.publish(msg)
        self.get_logger().info(
            f"/lock → {'1  (ENGAGED)' if state else '0  (RELEASED)'}"
        )

    def destroy_node(self):
        self._running = False
        self.sock.close()
        super().destroy_node()


# ═══════════════════════════════════════════════════════════════
def main():
    if '--list' in sys.argv:
        list_serial_ports()
        return

    if not os.path.exists(MEGA_PORT):
        print(f"\nWARN: Mega port '{MEGA_PORT}' not found — servo packets will be dropped.")
        print("Run with --list to see what's available.\n")

    mega  = MegaSerial()
    agent = MicroROSAgent()

    if LAUNCH_MICROROS_AGENT:
        agent.start()

    rclpy.init()
    node = UDPBridgeNode(mega)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if LAUNCH_MICROROS_AGENT:
            agent.stop()
        mega.close()
        print("Bridge stopped.")


if __name__ == '__main__':
    main()