"""Send an exported coverage plan to a running Nav2 stack (ROS 2).

For a plain Nav2 stack with bt_navigator. On the SIMBA robot use coverage.launch.py plan:=FILE
instead (see the repository README).

Run on the robot / ROS 2 machine (needs nav2_simple_commander, not the other deps):
    python3 send_to_nav2.py office_coverage.yaml               # NavigateThroughPoses (drives through, no stops)
    python3 send_to_nav2.py office_coverage.yaml --waypoints   # FollowWaypoints (stops at every pose)
    python3 send_to_nav2.py office_coverage.yaml --set-initial-pose   # also tell AMCL the robot is at the plan start

The plan contains a pose at every vertex of the planned path (and at least every
0.5 m), so Nav2 follows the planned shape. With FollowWaypoints the robot would
stop at each of them, so NavigateThroughPoses is the default.

The map loaded in Nav2 must be the same map the plan was made on.
"""
import argparse
import json

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def load(path):
    with open(path) as f:
        return json.load(f) if path.endswith('.json') else yaml.safe_load(f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('plan')
    ap.add_argument('--waypoints', action='store_true', help='use FollowWaypoints (stops at every pose)')
    ap.add_argument('--set-initial-pose', action='store_true', help='set the AMCL initial pose to the plan start')
    a = ap.parse_args()
    plan = load(a.plan)

    rclpy.init()
    nav = BasicNavigator()
    if a.set_initial_pose:
        ip = PoseStamped()
        ip.header.frame_id = plan.get('frame_id', 'map')
        ip.header.stamp = nav.get_clock().now().to_msg()
        ip.pose.position.x, ip.pose.position.y = map(float, plan['start'])
        o = plan['poses'][0]['orientation'] if plan['poses'] else {'z': 0.0, 'w': 1.0}
        ip.pose.orientation.z, ip.pose.orientation.w = float(o['z']), float(o['w'])
        nav.setInitialPose(ip)
    nav.waitUntilNav2Active()
    poses = []
    for p in plan['poses']:
        ps = PoseStamped()
        ps.header.frame_id = plan.get('frame_id', 'map')
        ps.header.stamp = nav.get_clock().now().to_msg()
        ps.pose.position.x = float(p['x'])
        ps.pose.position.y = float(p['y'])
        ps.pose.orientation.z = float(p['orientation']['z'])
        ps.pose.orientation.w = float(p['orientation']['w'])
        poses.append(ps)
    print(f'Sending {len(poses)} poses...')
    if a.waypoints:
        nav.followWaypoints(poses)
    else:
        nav.goThroughPoses(poses)
    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        if fb is not None and hasattr(fb, 'current_waypoint'):
            print(f'\rwaypoint {fb.current_waypoint + 1}/{len(poses)}', end='')
        elif fb is not None and hasattr(fb, 'number_of_poses_remaining'):
            print(f'\rpose {len(poses) - fb.number_of_poses_remaining}/{len(poses)}', end='')
    print()
    result = nav.getResult()
    print({TaskResult.SUCCEEDED: 'Done', TaskResult.CANCELED: 'Canceled', TaskResult.FAILED: 'Failed'}.get(result, result))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
