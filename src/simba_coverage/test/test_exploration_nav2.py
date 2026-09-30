"""Actual RPP goal-heading rotation with simulated odometry, loopback domain 91."""
import os
os.environ['ROS_DOMAIN_ID'] = '91'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
os.environ['CYCLONEDDS_URI'] = '<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="lo" multicast="false"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
import math
from pathlib import Path
import signal
import subprocess
import time
import pytest
pytest.importorskip('rclpy')
pytest.importorskip('nav2_msgs')
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_prefix, PackageNotFoundError
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import LaserScan
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from nav2_msgs.action import FollowPath
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from simba_coverage.geometry import wrap
from simba_coverage.params import load_settings


def controller_executable():
    override = os.environ.get('SIMBA_RPP_EXECUTABLE')
    if override:
        return Path(override)
    try:
        return Path(get_package_prefix('nav2_controller')) / 'lib/nav2_controller/controller_server'
    except PackageNotFoundError:
        pytest.skip('Actual Nav2 controller is not installed')


@pytest.mark.parametrize('heading', [.3, -.3, .8])
def test_actual_rpp_final_heading_without_translation(tmp_path, heading):
    executable = controller_executable()
    config = Path(__file__).resolve().parents[2] / 'simba_bringup/config'
    params = load_settings(config).render_nav2(directory=tmp_path, live_map=True)
    log_path = tmp_path / 'controller.log'
    with log_path.open('w') as log:
        process = subprocess.Popen([str(executable), '--ros-args', '--params-file', str(params),
            '-r', 'cmd_vel:=/cmd_vel_nav'], stdout=log, stderr=subprocess.STDOUT)
    rclpy.init(args=[])
    node = Node('fake_rpp_odometry')
    broadcaster = TransformBroadcaster(node)
    static = StaticTransformBroadcaster(node)
    laser_tf = TransformStamped()
    laser_tf.header.frame_id = 'base_footprint'
    laser_tf.child_frame_id = 'laser'
    laser_tf.transform.rotation.w = 1.
    static.sendTransform(laser_tf)
    odom_pub = node.create_publisher(Odometry, '/odom', 10)
    scan_pub = node.create_publisher(LaserScan, '/scan', 10)
    state = {'yaw': 0., 'angular': 0., 'last': time.monotonic()}
    commands = []
    def receive_velocity(message):
        commands.append((message.linear.x, message.angular.z))
        state['angular'] = message.angular.z
    node.create_subscription(Twist, '/cmd_vel_nav', receive_velocity, 10)
    def publish():
        now = time.monotonic()
        state['yaw'] = wrap(state['yaw'] + state['angular'] * (now - state['last']))
        state['last'] = now
        stamp = node.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.rotation.z = math.sin(state['yaw']/2)
        transform.transform.rotation.w = math.cos(state['yaw']/2)
        broadcaster.sendTransform(transform)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.orientation = transform.transform.rotation
        odom.twist.twist.angular.z = state['angular']
        odom_pub.publish(odom)
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'laser'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2 * math.pi / 360
        scan.range_min = .2
        scan.range_max = 12.
        scan.ranges = [2.] * 360
        scan_pub.publish(scan)
    node.create_timer(.02, publish)
    def until(condition, seconds=8.):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and process.poll() is None:
            if condition():
                return
            rclpy.spin_once(node, timeout_sec=.02)
        assert condition(), log_path.read_text()[-4000:]
    lifecycle = node.create_client(ChangeState, '/controller_server/change_state')
    action = ActionClient(node, FollowPath, '/follow_path')
    try:
        until(lifecycle.service_is_ready)
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            request = ChangeState.Request()
            request.transition.id = transition
            future = lifecycle.call_async(request)
            until(future.done)
            assert future.result().success, log_path.read_text()[-4000:]
        until(action.server_is_ready)
        goal = FollowPath.Goal()
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'goal_checker'
        goal.progress_checker_id = 'progress_checker'
        path = RosPath()
        path.header.frame_id = 'odom'
        path.header.stamp = node.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.orientation.z = math.sin(heading/2)
        pose.pose.orientation.w = math.cos(heading/2)
        path.poses = [pose]
        goal.path = path
        accepted = action.send_goal_async(goal)
        until(accepted.done)
        assert accepted.result().accepted
        result = accepted.result().get_result_async()
        until(result.done, 12.)
        assert result.result().status == 4 and result.result().result.error_code == 0, log_path.read_text()[-4000:]
        assert abs(wrap(state['yaw'] - heading)) <= .05
        assert commands and max(abs(v) for v, _ in commands) <= 1e-8
        assert any(abs(w) > .01 for _, w in commands)
    finally:
        action.destroy()
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.)


def test_actual_global_costmap_follows_growing_slam_map(tmp_path):
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.msg import Costmap
    from nav2_msgs.action import ComputePathToPose
    from simba_coverage.ros_common import LATCHED
    try:
        executable = Path(get_package_prefix('nav2_planner')) / 'lib/nav2_planner/planner_server'
    except PackageNotFoundError:
        pytest.skip('Actual Nav2 planner is not installed')
    config = Path(__file__).resolve().parents[2] / 'simba_bringup/config'
    params = load_settings(config).render_nav2(directory=tmp_path, live_map=True)
    log_path = tmp_path / 'planner.log'
    with log_path.open('w') as log:
        process = subprocess.Popen([str(executable), '--ros-args', '--params-file', str(params)],
                                   stdout=log, stderr=subprocess.STDOUT)
    rclpy.init(args=[])
    node = Node('fake_growing_slam_map')
    static = StaticTransformBroadcaster(node)
    broadcaster = TransformBroadcaster(node)
    transforms = []
    for parent, child in [('map', 'odom'), ('base_footprint', 'laser')]:
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = parent, child
        transform.transform.rotation.w = 1.
        transforms.append(transform)
    static.sendTransform(transforms)
    maps = node.create_publisher(OccupancyGrid, '/map', LATCHED)
    scans = node.create_publisher(LaserScan, '/scan', 10)
    received = []
    node.create_subscription(Costmap, '/global_costmap/costmap_raw', received.append, LATCHED)
    def publish_sensors():
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id, transform.child_frame_id = 'odom', 'base_footprint'
        transform.transform.rotation.w = 1.
        broadcaster.sendTransform(transform)
        scan = LaserScan()
        scan.header = transform.header
        scan.header.frame_id = 'laser'
        scan.angle_min, scan.angle_max = -math.pi, math.pi
        scan.angle_increment = 2 * math.pi / 360
        scan.range_min, scan.range_max = .2, 12.
        scan.ranges = [float('inf')] * 360
        scans.publish(scan)
    node.create_timer(.02, publish_sensors)
    def publish_map(width, height, x, y):
        message = OccupancyGrid()
        message.header.frame_id = 'map'
        message.header.stamp = node.get_clock().now().to_msg()
        message.info.width, message.info.height = width, height
        message.info.resolution = .05
        message.info.origin.position.x, message.info.origin.position.y = x, y
        message.info.origin.orientation.w = 1.
        message.data = [0] * (width * height)
        maps.publish(message)
    def until(condition, seconds=8.):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and process.poll() is None:
            if condition():
                return
            rclpy.spin_once(node, timeout_sec=.02)
        assert condition(), log_path.read_text()[-4000:]
    lifecycle = node.create_client(ChangeState, '/planner_server/change_state')
    action = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    try:
        publish_map(40, 40, -1., -1.)
        until(lifecycle.service_is_ready)
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            request = ChangeState.Request()
            request.transition.id = transition
            future = lifecycle.call_async(request)
            until(future.done)
            assert future.result().success, log_path.read_text()[-4000:]
        until(lambda: received and received[-1].metadata.size_x == 40)
        publish_map(80, 60, -2., -1.5)
        until(lambda: received and received[-1].metadata.size_x == 80 and received[-1].metadata.size_y == 60, 2.)
        assert received[-1].metadata.origin.position.x == -2.
        assert received[-1].metadata.origin.position.y == -1.5
        until(action.server_is_ready)
        goal = ComputePathToPose.Goal()
        goal.planner_id = 'GridBased'
        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = node.get_clock().now().to_msg()
        goal.goal.pose.position.x = 1.5  # Outside the original map extent.
        goal.goal.pose.orientation.w = 1.
        accepted = action.send_goal_async(goal)
        until(accepted.done)
        assert accepted.result().accepted
        result = accepted.result().get_result_async()
        until(result.done)
        assert result.result().status == 4 and result.result().result.error_code == 0
        assert result.result().result.path.poses[-1].pose.position.x > 1.
    finally:
        action.destroy()
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.)
