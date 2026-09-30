"""Kinematic Create 3 + RPLIDAR stand-in for closed-loop tests of the SIMBA coverage stack.

Integrates /cmd_vel (unicycle, acceleration limited, 0.5 s command timeout like the Create 3) and
publishes /odom, TF odom->base_footprint, a ray-cast /scan in the 'laser' frame (yaw pi, as in
lidar_params.yaml), battery/hazard/wheel/dock status, and serves /e_stop. Walls come from the map
image; extra round obstacles (--obstacle x,y,r) exist only in the simulated world, not in the map
that Nav2 loads. A body that would hit a wall or obstacle is stopped and reports a BUMP hazard.
The true pose is logged to a CSV file.

Not simulated: IR proximity (hazard OBJECT_PROXIMITY), odometry drift, wheel slip, LiDAR motion blur.
"""
import argparse
import csv
import math
import time
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt
import rclpy
import rclpy.duration
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, BatteryState
from tf2_msgs.msg import TFMessage
from irobot_create_msgs.msg import HazardDetectionVector, HazardDetection, WheelStatus, DockStatus
from irobot_create_msgs.srv import EStop

BODY = 0.171  # Create 3 radius; SIMBA plans with 0.18 m + 0.02 m padding.
BEAMS = 300   # Fits one DDS fragment; see env.sh.


class World:
    """Occupancy at 1 cm with a distance field for collision checks and ray casting."""
    def __init__(self, map_yaml, obstacles, res=0.01):
        m = yaml.safe_load(open(map_yaml))
        img = np.array(Image.open(Path(map_yaml).parent / m['image']))
        self.res = res
        self.ox, self.oy = m['origin'][:2]
        k = int(round(m['resolution'] / res))
        occ = np.kron((img == 0).astype(np.uint8), np.ones((k, k), np.uint8)).astype(bool)[::-1]  # row 0 = origin y
        yy, xx = np.mgrid[0:occ.shape[0], 0:occ.shape[1]]
        cx, cy = self.ox + (xx + .5) * res, self.oy + (yy + .5) * res
        for x, y, r in obstacles:
            occ |= (cx - x) ** 2 + (cy - y) ** 2 <= r * r
        self.occ = occ
        self.dist = distance_transform_edt(~occ) * res

    def clearance(self, x, y):
        r, c = int((y - self.oy) / self.res), int((x - self.ox) / self.res)
        if not (0 <= r < self.occ.shape[0] and 0 <= c < self.occ.shape[1]):
            return 0.
        return float(self.dist[r, c])

    def scan(self, x, y, angles, max_range=12.):
        # Sphere tracing on the distance field: exact to the grid resolution and fast in numpy.
        t = np.full(len(angles), .05)
        c, s = np.cos(angles), np.sin(angles)
        for _ in range(80):
            px, py = x + t * c, y + t * s
            r = ((py - self.oy) / self.res).astype(int)
            k = ((px - self.ox) / self.res).astype(int)
            inside = (r >= 0) & (r < self.occ.shape[0]) & (k >= 0) & (k < self.occ.shape[1])
            d = np.zeros_like(t)
            d[inside] = self.dist[r[inside], k[inside]]
            d[~inside] = max_range
            t = np.minimum(t + np.maximum(d, self.res * .5) * (d > self.res * .5), max_range)
        return t


class FakeCreate3(Node):
    def __init__(self, a):
        super().__init__('fake_create3')
        self.world = World(a.map, a.obstacle)
        self.x0, self.y0, self.yaw0 = a.start
        self.x, self.y, self.yaw = a.start
        self.v = self.w = 0.
        self.cmd = (0., 0.)
        self.cmd_time = -1.
        self.history = [(time.monotonic(), *a.start)]
        self.estop = False
        self.bump_until = 0.
        self.bumps = 0
        self.rng = np.random.default_rng(1)
        self.log_file = open(a.log, 'w', newline='', buffering=1)  # Line-buffered: complete even if killed.
        self.log = csv.writer(self.log_file)
        self.log.writerow(['t', 'x', 'y', 'yaw', 'v', 'w', 'clearance', 'bump'])
        self.t0 = time.monotonic()
        self.create_subscription(Twist, '/cmd_vel', self.command, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)  # Reliable: Nav2 subscribes with SystemDefaultsQoS.
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.bat_pub = self.create_publisher(BatteryState, '/battery_state', qos_profile_sensor_data)
        self.haz_pub = self.create_publisher(HazardDetectionVector, '/hazard_detection', qos_profile_sensor_data)
        self.wheel_pub = self.create_publisher(WheelStatus, '/wheel_status', qos_profile_sensor_data)
        self.dock_pub = self.create_publisher(DockStatus, '/dock_status', qos_profile_sensor_data)
        self.create_service(EStop, '/e_stop', self.e_stop)
        self.dt = .02
        self.create_timer(self.dt, self.step)
        self.create_timer(.05, self.publish_odom)
        self.create_timer(.1, self.publish_scan)
        self.create_timer(.1, self.publish_status)
        self.create_timer(1., lambda: self.bat_pub.publish(BatteryState(header=self.header(), percentage=.9)))

    def header(self, frame=''):
        h = Odometry().header
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = frame
        return h

    def command(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.cmd_time = time.monotonic()

    def e_stop(self, req, res):
        self.estop = bool(req.e_stop_on)
        res.success = True
        return res

    def step(self):
        now = time.monotonic()
        tv, tw = self.cmd if now - self.cmd_time < .5 and not self.estop else (0., 0.)
        self.v += float(np.clip(tv - self.v, -1. * self.dt, 1. * self.dt))
        self.w += float(np.clip(tw - self.w, -4. * self.dt, 4. * self.dt))
        yaw = self.yaw + self.w * self.dt
        x = self.x + self.v * math.cos(yaw) * self.dt
        y = self.y + self.v * math.sin(yaw) * self.dt
        if self.world.clearance(x, y) < BODY and self.world.clearance(x, y) <= self.world.clearance(self.x, self.y):
            if now > self.bump_until:
                self.bumps += 1
                self.get_logger().warn(f'BUMP at ({self.x:.2f}, {self.y:.2f})')
            self.bump_until = now + .3
            self.v = 0.
            x, y = self.x, self.y
        self.x, self.y, self.yaw = x, y, math.atan2(math.sin(yaw), math.cos(yaw))
        self.history.append((now, self.x, self.y, self.yaw))
        del self.history[:-20]
        if int(now / .1) != int((now - self.dt) / .1):
            self.log.writerow([f'{now - self.t0:.2f}', f'{self.x:.4f}', f'{self.y:.4f}', f'{self.yaw:.4f}',
                               f'{self.v:.3f}', f'{self.w:.3f}', f'{self.world.clearance(self.x, self.y):.3f}',
                               int(now < self.bump_until)])

    def odom_pose(self):
        # Odometry starts at (0, 0, 0) where the robot booted, like the real Create 3.
        dx, dy = self.x - self.x0, self.y - self.y0
        c, s = math.cos(-self.yaw0), math.sin(-self.yaw0)
        return c * dx - s * dy, s * dx + c * dy, self.yaw - self.yaw0

    def publish_odom(self):
        x, y, yaw = self.odom_pose()
        od = Odometry()
        od.header = self.header('odom')
        od.child_frame_id = 'base_footprint'
        od.pose.pose.position.x, od.pose.pose.position.y = x, y
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        od.twist.twist.linear.x, od.twist.twist.angular.z = self.v, self.w
        self.odom_pub.publish(od)
        t = TransformStamped()
        t.header = od.header
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation = od.pose.pose.orientation
        self.tf_pub.publish(TFMessage(transforms=[t]))

    def publish_scan(self):
        scan = LaserScan()
        scan.header = self.header('laser')
        # rplidar_ros stamps a scan with its start time, one scan period before publishing.
        scan.header.stamp = (self.get_clock().now() - rclpy.duration.Duration(seconds=.1)).to_msg()
        scan.angle_min, scan.angle_increment = -math.pi, 2 * math.pi / BEAMS
        scan.angle_max = scan.angle_min + (BEAMS - 1) * scan.angle_increment
        scan.range_min, scan.range_max, scan.scan_time = .15, 12., .1
        # Ray-cast from the pose at the stamp time, so scan and odometry agree while turning.
        _, x, y, yaw = min(self.history, key=lambda h: abs(h[0] - (time.monotonic() - .1)))
        angles = scan.angle_min + np.arange(BEAMS) * scan.angle_increment + yaw + math.pi
        ranges = self.world.scan(x, y, angles) + self.rng.normal(0, .01, BEAMS)
        ranges[ranges >= 11.9] = np.inf
        scan.ranges = ranges.astype(float).tolist()
        self.scan_pub.publish(scan)

    def publish_status(self):
        hz = HazardDetectionVector()
        hz.header = self.header('base_link')
        if time.monotonic() < self.bump_until:
            hz.detections = [HazardDetection(type=HazardDetection.BUMP)]
        self.haz_pub.publish(hz)
        self.wheel_pub.publish(WheelStatus(header=self.header(), wheels_enabled=not self.estop))
        self.dock_pub.publish(DockStatus(header=self.header(), is_docked=False, dock_visible=False))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True)
    ap.add_argument('--start', type=float, nargs=3, required=True, metavar=('X', 'Y', 'YAW'))
    ap.add_argument('--obstacle', type=lambda s: tuple(map(float, s.split(','))), action='append', default=[],
                    metavar='X,Y,R')
    ap.add_argument('--log', required=True)
    a = ap.parse_args()
    rclpy.init()
    node = FakeCreate3(a)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.get_logger().info(f'bumps={node.bumps}')
        node.log_file.close()


if __name__ == '__main__':
    main()
