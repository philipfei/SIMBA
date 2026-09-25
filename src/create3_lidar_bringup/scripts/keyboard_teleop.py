#!/usr/bin/env python3

import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """
Create 3 safe keyboard teleoperation (hold a movement key)
  W: forward       S: reverse
  A: rotate left   D: rotate right
  X/Space: stop    Q: stop and quit

The robot stops automatically about 0.35 seconds after key release.
Speed limits: 0.25 m/s forward, 0.15 m/s reverse, 0.50 rad/s angular.
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('create3_keyboard_teleop')
        self.publisher = self.create_publisher(Twist, '/cmd_vel_remote', 10)
        self.command = Twist()
        self.last_key_time = 0.0

    def set_key(self, key):
        self.command = Twist()
        if key == 'w':
            self.command.linear.x = 0.25
        elif key == 's':
            self.command.linear.x = -0.15
        elif key == 'a':
            self.command.angular.z = 0.50
        elif key == 'd':
            self.command.angular.z = -0.50
        self.last_key_time = time.monotonic()

    def tick(self):
        if time.monotonic() - self.last_key_time > 0.35:
            self.command = Twist()
        self.publisher.publish(self.command)

    def stop(self):
        self.command = Twist()
        for _ in range(5):
            self.publisher.publish(self.command)
            time.sleep(0.02)


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = KeyboardTeleop()
    print(HELP, flush=True)
    tty.setcbreak(sys.stdin.fileno())
    try:
        running = True
        while rclpy.ok() and running:
            readable, _, _ = select.select([sys.stdin], [], [], 0.05)
            if readable:
                key = sys.stdin.read(1).lower()
                if key == 'q':
                    running = False
                elif key in ('w', 'a', 's', 'd', 'x', ' '):
                    node.set_key(key)
            node.tick()
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
