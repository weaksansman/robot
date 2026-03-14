#!/usr/bin/env python3
import socket
import struct
import threading
import subprocess
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
#  Not sure which port?  Run: ls -l /dev/ttyA* or ls - l /dev/ttyU* #check if there is something PI connect IGNORE AMA0
#  
#  Common ports:
#    /dev/ttyUSB0   ← CP2102 / CH340 chips  (most ESP32 devkit boards)
#    /dev/ttyACM0   ← ESP32-S3 native USB, or Arduino-style boards
#
# ═══════════════════════════════════════════════════════════════
ESP32_PORT = "/dev/ttyACM0" 
BAUD_RATE  = 115200	#Do not change or else PI can't communicate ESP32

# ── micro-ROS agent ──────────────────────────────────────────────────────────
# DO not change 
LAUNCH_MICROROS_AGENT = True #ALways true

# ── UDP ──────────────────────────────────────────────────────────────────────
LISTEN_IP   = "0.0.0.0"   # listen on all network interfaces
LISTEN_PORT = 3390         # must match PI_PORT 

# ── ROS2 topics ──────────────────────────────────────────────────────────────
CMD_VEL_TOPIC = "cmd_vel"
LOCK_TOPIC    = "/lock"

# ── Internal packet constants ─────────────────────────────────────────────────
MOTION_PACKET_SIZE = 8      # struct.pack('ff', linear, angular)
LOCK_PACKET_SIZE   = 2      # struct.pack('Bb', 0xFF, state)
LOCK_MARKER        = 0xFF


# ═══════════════════════════════════════════════════════════════════════════════
#  --list helper:  show all serial ports with USB descriptions
# ═══════════════════════════════════════════════════════════════════════════════
def _get_port_description(port):
    """Read USB device info from udevadm (most reliable on Pi)."""
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

    # Fallback: sysfs VID/PID
    dev_name = port.split('/')[-1]
    try:
        base = f'/sys/class/tty/{dev_name}/device/..'
        vid = open(f'{base}/idVendor').read().strip()
        pid = open(f'{base}/idProduct').read().strip()
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
        print("  (none found — is the ESP32 plugged in?)")
    else:
        for port in ports:
            desc = _get_port_description(port)
            print(f"  {port:<20}  {desc}")

    print("─" * 62)
    print()
    print("Common chips:")
    print("  CP2102 / CP2104  →  usually /dev/ttyUSB*   (most ESP32 devkits)")
    print("  CH340 / CH341    →  usually /dev/ttyUSB*")
    print("  ESP32-S3 native  →  usually /dev/ttyACM*")
    print()
    print("Once you know the port, set ESP32_PORT at the top of this file.")
    print()


# ═══════════════════════════════════════════════════════════════════════════════
#  micro-ROS agent process manager
# ═══════════════════════════════════════════════════════════════════════════════
class MicroROSAgent:
    def __init__(self):
        self._proc = None

    def start(self):
        cmd = f"micro-ros-agent serial --dev {ESP32_PORT} -b {BAUD_RATE}"
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
            print()

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


# ═══════════════════════════════════════════════════════════════════════════════
#  ROS2 UDP bridge node
# ═══════════════════════════════════════════════════════════════════════════════
class UDPBridgeNode(Node):
    def __init__(self):
        super().__init__('udp_bridge')

        self.cmd_pub  = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.lock_pub = self.create_publisher(Int8,  LOCK_TOPIC,    10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((LISTEN_IP, LISTEN_PORT))
        self.sock.settimeout(1.0)

        self.get_logger().info(f"ESP32 port    : {ESP32_PORT} @ {BAUD_RATE} baud")
        self.get_logger().info(f"UDP listening : {LISTEN_IP}:{LISTEN_PORT}")
        self.get_logger().info(f"Publishing    : '{CMD_VEL_TOPIC}'  |  '{LOCK_TOPIC}'")

        self._last_recv = self.get_clock().now()
        self._is_locked = False

        # Watchdog: send stop if laptop goes silent (but not while locked)
        self.create_timer(0.1, self._watchdog_cb)

        self._running = True
        threading.Thread(target=self._recv_loop, daemon=True).start()

    # ── Watchdog ───────────────────────────────────────────────────────────────
    def _watchdog_cb(self):
        if self._is_locked:
            return  # never override position-hold with a watchdog stop
        elapsed = (self.get_clock().now() - self._last_recv).nanoseconds / 1e9
        if elapsed > 0.5:
            self.cmd_pub.publish(Twist())  # all-zero Twist = full stop

    # ── UDP receive ────────────────────────────────────────────────────────────
    def _recv_loop(self):
        while self._running:
            try:
                data, addr = self.sock.recvfrom(64)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"UDP recv error: {e}")
                continue

            self._last_recv = self.get_clock().now()

            if len(data) == MOTION_PACKET_SIZE:
                self._handle_motion(data)
            elif len(data) == LOCK_PACKET_SIZE:
                self._handle_lock(data)
            else:
                self.get_logger().warn(
                    f"Unknown packet: {len(data)} bytes from {addr[0]}:{addr[1]}"
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
        self.get_logger().info(f"/lock published → {'1  (ENGAGED)' if state else '0  (RELEASED)'}")

    def destroy_node(self):
        self._running = False
        self.sock.close()
        super().destroy_node()


# ═══════════════════════════════════════════════════════════════════════════════
def main():

    # ── --list mode ────────────────────────────────────────────────────────────
    if '--list' in sys.argv:
        list_serial_ports()
        return

    # ── Validate port ──────────────────────────────────────────────────────────
    if not os.path.exists(ESP32_PORT):
        print(f"\nERROR: Port '{ESP32_PORT}' does not exist.")
        print("Run with --list to see what's available:\n")
        list_serial_ports()
        sys.exit(1)

    # ── micro-ROS agent ────────────────────────────────────────────────────────
    agent = MicroROSAgent()
    if LAUNCH_MICROROS_AGENT:
        agent.start()

    # ── ROS2 bridge ────────────────────────────────────────────────────────────
    rclpy.init()
    node = UDPBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if LAUNCH_MICROROS_AGENT:
            agent.stop()
        print("Bridge stopped.")


if __name__ == '__main__':
    main()
