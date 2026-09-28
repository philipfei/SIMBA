"""Lease-based keyboard teleoperation with an offline key-repeat measurement mode."""
import argparse
import select
import sys
import termios
import time
import tty
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Empty
from std_srvs.srv import SetBool
from simba_coverage.params import load_settings, default_config_dir


class KeyboardTeleop(Node):
    def __init__(self, config_dir):
        super().__init__('simba_keyboard_teleop')
        self.settings = load_settings(config_dir)
        motion = self.settings['motion']
        self.forward = motion['manual_forward_m_s']
        self.reverse = motion['manual_reverse_m_s']
        self.angular = motion['manual_angular_rad_s']
        self.event_timeout = 0.35
        self.publisher = self.create_publisher(Twist, '/cmd_vel_remote', 10)
        self.heartbeat = self.create_publisher(Empty, '/control/manual_heartbeat', 10)
        self.manual = self.create_client(SetBool, '/control/manual')
        self.command = None
        self.last_key_time = 0.0
        self.zero_sent = True
        self.last_heartbeat = 0.0

    def request_manual(self, enabled):
        if not self.manual.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('/control/manual service is unavailable')
        request = SetBool.Request(); request.data = enabled
        future = self.manual.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(response.message if response else 'Manual ownership request timed out')

    def set_key(self, key):
        command = Twist()
        if key == 'w': command.linear.x = self.forward
        elif key == 's': command.linear.x = -self.reverse
        elif key == 'a': command.angular.z = self.angular
        elif key == 'd': command.angular.z = -self.angular
        self.command = command
        self.last_key_time = time.monotonic()
        self.zero_sent = False

    def tick(self):
        now = time.monotonic()
        if now - self.last_heartbeat >= 0.1:
            self.heartbeat.publish(Empty()); self.last_heartbeat = now
        active = self.command is not None and now - self.last_key_time <= self.event_timeout
        if active:
            self.publisher.publish(self.command)
        elif not self.zero_sent:
            self.publisher.publish(Twist()); self.zero_sent = True; self.command = None

    def stop(self):
        for _ in range(3):
            self.publisher.publish(Twist()); time.sleep(0.05)


def measure_repeat():
    previous = termios.tcgetattr(sys.stdin)
    print('Hold one movement key for at least two seconds. No ROS messages will be published.', flush=True)
    tty.setcbreak(sys.stdin.fileno())
    stamps = []
    try:
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline and len(stamps) < 12:
            readable, _, _ = select.select([sys.stdin], [], [], 0.1)
            if readable:
                sys.stdin.read(1); stamps.append(time.monotonic())
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, previous)
    if len(stamps) < 3:
        raise RuntimeError('Not enough repeated key events were observed')
    gaps = [b-a for a,b in zip(stamps, stamps[1:])]
    print(f'Observed first repeat delay: {gaps[0]:.3f} s')
    print(f'Observed steady repeat interval: {sum(gaps[1:])/len(gaps[1:]):.3f} s')
    print('Keep the 0.5 s heartbeat and remote Twist timeouts unchanged.')


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--config-dir', default=str(default_config_dir()))
    parser.add_argument('--measure-key-repeat', action='store_true')
    parsed, ros_args = parser.parse_known_args(args)
    if parsed.measure_key_repeat:
        measure_repeat(); return
    terminal = termios.tcgetattr(sys.stdin)
    rclpy.init(args=ros_args)
    node = KeyboardTeleop(parsed.config_dir)
    print('W/S/A/D move, X or Space stops, Q stops and exits.', flush=True)
    tty.setcbreak(sys.stdin.fileno())
    granted = False
    try:
        node.request_manual(True); granted = True
        running = True
        while rclpy.ok() and running:
            readable, _, _ = select.select([sys.stdin], [], [], 0.05)
            if readable:
                key = sys.stdin.read(1).lower()
                if key == 'q': running = False
                elif key in ('w','a','s','d','x',' '): node.set_key(key)
            node.tick(); rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        if rclpy.ok():
            node.stop()
            if granted:
                try: node.request_manual(False)
                except RuntimeError as error: node.get_logger().error(str(error))
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
