#!/usr/bin/env python3
"""
Truck teleop — plain CLI, no curses.

  ↑ / ↓    throttle  +/- 0.5 m/s
  ← / →    steer     +/- 0.04 rad
  SPACE     stop + center
  q / ^C   quit
"""

import os, sys, tty, termios, select, threading, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

WHEEL_RADIUS = 0.5    # m  — converts m/s → rad/s for the wheel joints
SPEED_STEP   = 0.5    # m/s per keypress
STEER_STEP   = 0.04   # rad per keypress (~2.3°)
MAX_SPEED    = 5.0    # m/s
MAX_STEER    = 0.5236 # rad (30°)

# Raw escape codes for arrow keys
KEY_UP    = b'\x1b[A'
KEY_DOWN  = b'\x1b[B'
KEY_RIGHT = b'\x1b[C'
KEY_LEFT  = b'\x1b[D'


class TruckTeleop(Node):
    def __init__(self):
        super().__init__('truck_teleop')
        # /truck/cmd_delta → ackermann_controller_node (was /truck/steer)
        self.pub_steer = self.create_publisher(Float64, '/truck/cmd_delta', 10)
        self.pub_drive = self.create_publisher(Float64, '/truck/drive',     10)
        self.speed = 0.0   # m/s
        self.angle = 0.0   # rad

    def publish(self):
        s = Float64()
        s.data = self.angle
        self.pub_steer.publish(s)

        d = Float64()
        d.data = self.speed / WHEEL_RADIUS   # m/s → rad/s
        self.pub_drive.publish(d)

    def stop(self):
        self.speed = 0.0
        self.angle = 0.0
        self.publish()

    def wait_for_connection(self, timeout=30.0):
        """Block until both publishers have at least one subscriber.

        steer  → steering_controller_node
        drive  → parameter_bridge → Gazebo JointController
        """
        deadline = time.monotonic() + timeout
        spinner  = '|/-\\'
        i = 0
        sys.stdout.write("waiting for truck controller + drive bridge ")
        sys.stdout.flush()
        while time.monotonic() < deadline:
            steer_subs = self.pub_steer.get_subscription_count()
            drive_subs = self.pub_drive.get_subscription_count()
            if steer_subs >= 1 and drive_subs >= 1:
                sys.stdout.write(
                    f"\r  connected: cmd_delta←{steer_subs} sub, drive←{drive_subs} sub          \n"
                )
                sys.stdout.flush()
                return True
            sys.stdout.write(
                f"\rwaiting for truck controller + drive bridge {spinner[i % 4]}  "
                f"(steer={steer_subs}, drive={drive_subs})"
            )
            sys.stdout.flush()
            i += 1
            time.sleep(0.25)
        sys.stdout.write("\n  TIMEOUT — is sim_truck.launch.py running?\n")
        return False


def read_key(timeout=0.05):
    """Return raw bytes of next keypress, or None on timeout."""
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        return os.read(sys.stdin.fileno(), 10)
    return None


def status(node):
    direction = "FWD" if node.speed > 0 else ("REV" if node.speed < 0 else "---")
    steer_dir = "L" if node.angle > 0 else ("R" if node.angle < 0 else "-")
    sys.stdout.write(
        f"\r  speed {node.speed:+.1f} m/s [{direction}]   "
        f"steer {node.angle*57.3:+.1f}° [{steer_dir}]   "
    )
    sys.stdout.flush()


def main():
    rclpy.init()
    node = TruckTeleop()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    print("=== Truck Teleop ===")

    if not node.wait_for_connection():
        node.destroy_node()
        rclpy.shutdown()
        return

    print("  ↑/↓  throttle    ←/→  steer    SPACE  stop    q  quit")
    print()

    try:
        tty.setraw(fd)
        while True:
            key = read_key()
            if key is None:
                continue

            if key in (b'q', b'Q', b'\x03'):   # q or Ctrl-C
                break
            elif key == b' ':
                node.stop()
            elif key == KEY_UP:
                node.speed = min(node.speed + SPEED_STEP, MAX_SPEED)
            elif key == KEY_DOWN:
                node.speed = max(node.speed - SPEED_STEP, -MAX_SPEED)
            elif key == KEY_LEFT:
                node.angle = min(node.angle + STEER_STEP, MAX_STEER)
            elif key == KEY_RIGHT:
                node.angle = max(node.angle - STEER_STEP, -MAX_STEER)
            else:
                continue

            node.publish()
            status(node)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.stop()
        print("\n  stopped.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
