#!/usr/bin/env python3
"""Arrow-key steering teleop for the smartport truck.

  ←  /  →  — steer left / right  (hold for continuous sweep)
  ↑         — re-center wheels
  SPACE      — re-center wheels
  Q          — quit (re-centers on exit)
"""

import curses
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

MAX_ANGLE  = 0.5236   # ±30° hard limit
STEER_STEP = 0.04     # rad per tick (~2.3°)
RATE_HZ    = 20       # publish rate


class KeyboardSteerNode(Node):
    def __init__(self):
        super().__init__('keyboard_steer')
        self.pub   = self.create_publisher(Float64, '/truck/steer', 10)
        self.angle = 0.0

    def set_angle(self, angle: float):
        self.angle = max(-MAX_ANGLE, min(MAX_ANGLE, angle))
        msg = Float64()
        msg.data = self.angle
        self.pub.publish(msg)

    def center(self):
        self.set_angle(0.0)


def _spin(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    node = KeyboardSteerNode()
    threading.Thread(target=_spin, args=(node,), daemon=True).start()

    def run(stdscr):
        curses.cbreak()
        curses.noecho()
        stdscr.keypad(True)
        stdscr.timeout(50)   # 50 ms block — ERR on timeout means key released
        curses.curs_set(0)

        def redraw():
            bar_len = 40
            center  = bar_len // 2
            pos     = center + int(node.angle / MAX_ANGLE * center)
            pos     = max(0, min(bar_len - 1, pos))
            bar     = ['-'] * bar_len
            bar[center] = '|'
            bar[pos]    = '#'
            stdscr.erase()
            stdscr.addstr(0, 0, "── Truck Steering Teleop ─────────────────────")
            stdscr.addstr(1, 0, "  ← / →  : steer left / right  (hold to sweep)")
            stdscr.addstr(2, 0, "  ↑ / SPACE : re-center")
            stdscr.addstr(3, 0, "  Q        : quit")
            stdscr.addstr(5, 0, f"  Angle : {node.angle:+.3f} rad  ({node.angle * 57.3:+.1f}°)")
            stdscr.addstr(6, 0, f"  [{''.join(bar)}]")
            stdscr.refresh()

        redraw()

        while True:
            key = stdscr.getch()   # blocks up to 50 ms; ERR = no key / released

            if key == ord('q') or key == ord('Q'):
                break
            elif key in (ord(' '), curses.KEY_UP):
                node.center()
                redraw()
            elif key == curses.KEY_LEFT:
                node.set_angle(node.angle + STEER_STEP)
                redraw()
            elif key == curses.KEY_RIGHT:
                node.set_angle(node.angle - STEER_STEP)
                redraw()
            # ERR (timeout) = nothing held — angle stays where it is

    try:
        curses.wrapper(run)
    finally:
        node.center()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
