"""Pi ROS adapter for the exploration state machine. Never publishes velocity."""
import math
import signal
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.msg import Costmap
from nav2_msgs.action import ComputePathToPose, FollowPath
from irobot_create_msgs.action import Dock, Undock
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger, SetBool
from simba_coverage.geometry import Grid, wrap
from simba_coverage.params import node_settings
from simba_coverage.state import Mission
from simba_coverage.ros_common import (
    Inputs, ActionSlot, LATCHED, LATEST, decode, json_msg, parameter,
    pose_msg, pose3, transform3, ros_now, stamp_seconds, yaw)
from .exploration import Exploration, Observation, SlamRecovery, load_profile


def occupancy_grid(message):
    info = message.info
    if info.width <= 0 or info.height <= 0 or info.resolution <= 0 or len(message.data) != info.width * info.height:
        raise ValueError('Invalid map metadata')
    cells = np.asarray(message.data, dtype=np.int16).reshape(info.height, info.width)
    if not np.isin(cells, [-1, 0, 100]).all():
        raise ValueError('Exploration requires a trinary SLAM map')
    return Grid.from_cells(cells, info.resolution,
        (info.origin.position.x, info.origin.position.y, yaw(info.origin.orientation)))


def costmap_grid(message):
    meta = message.metadata
    if meta.size_x <= 0 or meta.size_y <= 0 or meta.resolution <= 0 or len(message.data) != meta.size_x * meta.size_y:
        raise ValueError('Invalid costmap metadata')
    raw = np.asarray(message.data, dtype=np.uint8).reshape(meta.size_y, meta.size_x)
    # Inflation costs below inscribed collision remain traversable. The static
    # map is checked separately with the full footprint; avoid double inflation.
    cells = np.where(raw == 255, -1, np.where(raw >= 253, 100, 0))
    return Grid.from_cells(cells, meta.resolution,
        (meta.origin.position.x, meta.origin.position.y, yaw(meta.origin.orientation)))


class CandidateChecks:
    """Evaluate clearance on demand, so first-valid selection short-circuits."""
    def __init__(self, candidates, validate):
        self.candidates, self.validate = candidates, validate
        self.checked = {}

    def __len__(self):
        return len(self.candidates)

    def __getitem__(self, index):
        if index not in self.checked:
            self.checked[index] = self.validate(self.candidates[index])
        return self.checked[index]


class ExplorationCoordinator(Node):
    def __init__(self):
        super().__init__('exploration_coordinator')
        self.settings = node_settings(self)
        self.profile = load_profile(self.settings.config_dir)
        # Launch also validates this; direct node invocation must not accept a saved graph.
        import yaml
        slam = yaml.safe_load((self.settings.config_dir / 'slam_toolbox.yaml').read_text())['slam_toolbox']['ros__parameters']
        if slam.get('mode') != 'mapping' or slam.get('map_file_name', ''):
            raise ValueError('Fresh SLAM mapping required; map_file_name must be empty')
        self.policy = Exploration(self.profile, self.settings, time.monotonic(),
                                  parameter(self, 'auto_undock', True))
        self.recovery = SlamRecovery(self.profile)
        self.inputs = Inputs(self)
        self.mission = Mission(generation=self.policy.session_id)
        self.slot = ActionSlot(self, self.mission)
        for typ, name in ((ComputePathToPose, '/compute_path_to_pose'), (FollowPath, '/follow_path'),
                          (Dock, '/dock'), (Undock, '/undock')):
            self.slot.client(typ, name)
        self.gate = {}
        self.gate_received = 0.
        self.grid = None
        self.map_seq = 0
        self.map_stamp = 0.
        self.map_received = 0.
        self.map_received_ros = 0.
        self.global_grid = self.local_grid = None
        self.global_received = self.local_received = 0.
        self.global_stamp = 0.
        self.pose = self.odom_pose = None
        self.localization = 'TF_WAIT'
        self.visible_samples = 0
        self.last_dock_stamp = None
        self.sequence = 0
        self.last_state = 0.
        self.last_state_key = None
        self.last_home = None
        self.estop_pending = None
        self.graph_ready_since = None
        self.last_motion_pose = None
        self.last_motion_at = time.monotonic()
        self.lease_pub = self.create_publisher(String, '/coverage/lease', LATEST)
        self.trust_pub = self.create_publisher(String, '/coverage/trust', LATEST)
        self.state_pub = self.create_publisher(String, '/exploration/state', LATCHED)
        self.home_pub = self.create_publisher(PoseStamped, '/exploration/home', LATCHED)
        self.path_pub = self.create_publisher(Path, '/exploration/path', LATCHED)
        self.stop_client = self.create_client(Trigger, '/safety/estop')
        self.create_subscription(OccupancyGrid, '/map', self.receive_map, LATCHED)
        self.create_subscription(Costmap, '/global_costmap/costmap_raw', self.receive_global, LATCHED)
        self.create_subscription(Costmap, '/local_costmap/costmap_raw', self.receive_local, LATCHED)
        self.create_subscription(String, '/safety/state', self.receive_gate, LATEST)
        self.create_subscription(Empty, '/control/manual_heartbeat', self.manual_heartbeat, 10)
        self.create_service(SetBool, '/control/manual', self.manual_service)
        self.create_service(Trigger, '/exploration/return_home', self.return_service)
        self.create_service(Trigger, '/exploration/cancel', self.cancel_service)
        self.create_timer(1 / self.settings['localization']['pose_rate_hz'], self.tick)
        self.clear_path()

    def receive_map(self, message):
        try:
            if message.header.frame_id != self.settings['runtime']['map_frame']:
                raise ValueError('Unexpected map frame')
            stamp = stamp_seconds(message.header.stamp)
            if stamp < self.map_stamp:
                raise ValueError('Map timestamp reversed')
            grid = occupancy_grid(message)
            self.grid = grid
            self.map_seq += 1
            self.map_stamp = stamp
            self.map_received = time.monotonic()
            self.map_received_ros = ros_now(self)
        except (ValueError, TypeError) as error:
            self.get_logger().error(str(error))

    def receive_global(self, message):
        try:
            if message.header.frame_id != self.settings['runtime']['map_frame']:
                raise ValueError('Unexpected global costmap frame')
            self.global_grid = costmap_grid(message)
            self.global_stamp = stamp_seconds(message.header.stamp)
            self.global_received = time.monotonic()
        except (ValueError, TypeError) as error:
            self.get_logger().error(str(error))

    def receive_local(self, message):
        try:
            if message.header.frame_id != self.settings['runtime']['odom_frame']:
                raise ValueError('Unexpected local costmap frame')
            self.local_grid = costmap_grid(message)
            self.local_received = time.monotonic()
        except (ValueError, TypeError) as error:
            self.get_logger().error(str(error))

    def receive_gate(self, message):
        try:
            self.gate = decode(message)
            self.gate_received = time.monotonic()
        except (ValueError, TypeError):
            self.gate = {}

    def manual_heartbeat(self, _message):
        if self.policy.manual:
            self.policy.manual_heartbeat = time.monotonic()

    def update_pose(self, now):
        try:
            pose = self.inputs.lookup(self.settings['runtime']['map_frame'], self.settings['runtime']['base_frame'])
            odom = self.inputs.lookup(self.settings['runtime']['odom_frame'], self.settings['runtime']['base_frame'])
            for transform in (pose, odom):
                age = ros_now(self) - stamp_seconds(transform.header.stamp)
                if not -self.settings['health']['future_stamp_tolerance_s'] <= age <= self.profile['tf_age_s']:
                    raise ValueError('TF stale')
            self.pose, self.odom_pose = transform3(pose), transform3(odom)
            if not np.isfinite([*self.pose, *self.odom_pose]).all():
                raise ValueError('TF invalid')
        except Exception:
            self.pose = self.odom_pose = None
        self.localization = self.recovery.observe(now, self.pose, self.odom_pose)

    def costmap_ready(self, now):
        grid, cost = self.grid, self.global_grid
        return bool(grid is not None and cost is not None and
            now - self.global_received <= self.profile['costmap_wait_s'] and
            self.global_stamp >= self.map_received_ros + 1 / 2.0 and
            grid.cells.shape == cost.cells.shape and abs(grid.resolution - cost.resolution) < 1e-8 and
            np.allclose(grid.origin, cost.origin, atol=1e-6) and
            np.all(cost.cells[grid.cells == 100] == 100))

    def candidate_valid(self, point):
        if self.grid is None or self.global_grid is None:
            return False
        xy = point[:2]
        return (self.grid.segment_safe(xy, xy, self.settings.collision + self.profile['approach_margin_m']) and
                self.global_grid.segment_safe(xy, xy, self.profile['approach_margin_m']))

    def local_clear(self, now):
        if self.pose is None or self.odom_pose is None or self.local_grid is None or self.policy.home is None:
            return False
        if now - self.local_received > self.profile['costmap_wait_s']:
            return False
        if not self.candidate_valid(self.policy.home):
            return False
        heading = wrap(self.odom_pose[2] - self.pose[2])
        dx, dy = self.policy.home[0] - self.pose[0], self.policy.home[1] - self.pose[1]
        point = (self.odom_pose[0] + math.cos(heading)*dx - math.sin(heading)*dy,
                 self.odom_pose[1] + math.sin(heading)*dx + math.cos(heading)*dy)
        return self.local_grid.segment_safe(point, point, self.profile['approach_margin_m'])

    def observation(self):
        now = time.monotonic()
        docked = None
        if self.inputs.fresh('dock_status', self.settings['health']['dock_age_s']):
            dock = self.inputs.messages['dock_status']
            docked = dock.is_docked
            stamp = stamp_seconds(dock.header.stamp)
            if stamp != self.last_dock_stamp:
                self.visible_samples = self.visible_samples + 1 if dock.dock_visible else 0
                self.last_dock_stamp = stamp
        else:
            self.visible_samples = 0
        reason = self.gate.get('fault', '')
        if now - self.gate_received > self.settings['health']['gate_age_s']:
            reason = reason or 'SAFETY_GATE_STALE'
        elif not self.gate.get('graph_ok'):
            reason = reason or 'VELOCITY_GRAPH_CONFLICT'
        elif self.gate.get('config_hash') != self.settings.hash:
            reason = reason or 'CONFIG_HASH_MISMATCH'
        if not reason:
            if self.graph_ready_since is None:
                self.graph_ready_since = now
        else:
            self.graph_ready_since = None
        sensor_reason = self.inputs.reason()
        if self.policy.phase == 'WAIT_READY' and self.policy.auto:
            if not all(client.server_is_ready() for client in self.slot.clients.values()):
                sensor_reason = sensor_reason or 'ACTION_SERVERS_NOT_READY'
            elif self.graph_ready_since is None or now - self.graph_ready_since < self.profile['settle_s']:
                sensor_reason = sensor_reason or 'GRAPH_SETTLING'
        if self.policy.return_started is not None and self.policy.owner != 'NATIVE' and now - self.map_received > self.profile['map_wait_s']:
            sensor_reason = sensor_reason or 'MAP_STALE'
        return Observation(now=now, stamp=ros_now(self), stopped=self.inputs.stopped(),
            slot_idle=self.slot.idle, sensor_reason=sensor_reason, gate_reason=reason,
            gate_owner=self.gate.get('owner', 'NONE'), gate_epoch=self.gate.get('epoch', ''),
            gate_healthy=bool(self.gate.get('healthy')), docked=docked,
            visible_samples=self.visible_samples, pose=self.pose,
            localization=self.localization, map_seq=self.map_seq, map_stamp=self.map_stamp,
            map_identity=(self.grid.identity + ':' + self.global_grid.identity) if self.grid is not None and self.global_grid is not None else '',
            costmap_ready=self.costmap_ready(now),
            candidate_valid=CandidateChecks(self.policy.candidates, self.candidate_valid),
            local_clear=self.local_clear(now))

    def manual_service(self, request, response):
        response.success, response.message = self.policy.manual_request(request.data, self.observation())
        self.publish_control()
        self.publish_state(force=True)
        return response

    def return_service(self, _request, response):
        self.policy.last_jump_total = self.recovery.total
        response.success, response.message = self.policy.request_return(self.observation())
        self.publish_state(force=True)
        return response

    def cancel_service(self, _request, response):
        self.policy.cancel(self.observation())
        self.process_effects()
        self.publish_control()
        self.publish_state(force=True)
        response.success = True
        response.message = 'Cancel accepted; wait for stopped MANUAL/FAILED state before moving'
        return response

    def publish_control(self):
        self.lease_pub.publish(json_msg({'config_hash': self.settings.hash,
            'owner': self.policy.owner, 'epoch': self.policy.epoch}))
        trusted = not self.localization and self.grid is not None and time.monotonic() - self.map_received <= self.profile['map_wait_s']
        self.trust_pub.publish(json_msg({'trusted': trusted, 'reason': self.localization or ('MAP_STALE' if not trusted else '')}))

    def clear_path(self):
        message = Path()
        message.header.frame_id = self.settings['runtime']['map_frame']
        message.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(message)

    def request_estop(self, reason):
        self.policy.reason = reason
        if self.estop_pending is not None or not self.stop_client.service_is_ready():
            return
        self.estop_pending = self.stop_client.call_async(Trigger.Request())
        self.estop_pending.add_done_callback(lambda _f: setattr(self, 'estop_pending', None))

    def valid_path(self, path):
        if path.header.frame_id != self.settings['runtime']['map_frame'] or not path.poses or self.pose is None:
            return False
        points = [pose3(p.pose) for p in path.poses]
        if not np.isfinite(points).all() or math.dist(points[0][:2], self.pose[:2]) > self.settings['recovery']['planner_start_tolerance_m']:
            return False
        if math.dist(points[-1][:2], self.policy.home[:2]) > self.profile['navigation']['planner_tolerance_m'] + 1e-8:
            return False
        if self.grid is None or self.global_grid is None:
            return False
        segments = list(zip(points, points[1:])) or [(points[0], points[0])]
        return all(self.grid.segment_safe(a[:2], b[:2], self.settings.collision) and
                   self.global_grid.segment_safe(a[:2], b[:2], 1e-9) for a, b in segments)

    def process_results(self, obs):
        while self.slot.events:
            name, status, result, error = self.slot.events.pop(0)
            success = status == GoalStatus.STATUS_SUCCEEDED and not error
            if name == '/compute_path_to_pose':
                success = success and getattr(result, 'error_code', 0) == 0 and self.valid_path(result.path)
                path = result.path if success else None
                if success:
                    path.poses[-1].pose.orientation = pose_msg(self, self.policy.home[:2], self.policy.home[2]).pose.orientation
                self.policy.action_result('plan', success, path, obs)
            elif name == '/follow_path':
                self.policy.action_result('follow', success and getattr(result, 'error_code', 0) == 0, None, obs)
            else:
                self.policy.action_result(name[1:], success, getattr(result, 'is_docked', None), obs)

    def process_effects(self):
        effects, self.policy.effects = self.policy.effects, []
        for kind, payload in effects:
            try:
                if kind == 'cancel':
                    self.slot.cancel()
                elif kind == 'clear_path':
                    self.clear_path()
                elif kind == 'estop':
                    self.request_estop(payload)
                elif kind == 'show_path':
                    self.path_pub.publish(payload)
                elif kind == 'plan':
                    goal = ComputePathToPose.Goal()
                    goal.goal = pose_msg(self, payload[:2], payload[2])
                    goal.planner_id = 'GridBased'
                    goal.use_start = False
                    self.slot.send(ComputePathToPose, '/compute_path_to_pose', goal)
                elif kind == 'follow':
                    goal = FollowPath.Goal()
                    goal.path = payload
                    goal.controller_id = 'FollowPath'
                    goal.goal_checker_id = 'goal_checker'
                    goal.progress_checker_id = 'progress_checker'
                    self.last_motion_pose = self.pose
                    self.last_motion_at = time.monotonic()
                    self.slot.send(FollowPath, '/follow_path', goal)
                elif kind == 'undock':
                    # Re-read immediately before dispatch, including auto startup.
                    obs = self.observation()
                    if obs.docked is not True:
                        self.policy.fail(obs, 'DOCK_STATUS_UNCONFIRMED' if obs.docked is None else 'NOT_DOCKED')
                    else:
                        self.slot.send(Undock, '/undock', Undock.Goal())
                elif kind == 'dock':
                    self.slot.send(Dock, '/dock', Dock.Goal())
            except (RuntimeError, ValueError) as error:
                self.policy.fail(self.observation(), 'ACTION_UNAVAILABLE: ' + str(error))

    def publish_state(self, force=False):
        now = time.monotonic()
        key = (self.policy.phase, self.policy.subphase, self.policy.reason, self.policy.owner,
               self.gate.get('owner'), self.gate.get('epoch'))
        if not force and key == self.last_state_key and now - self.last_state < self.profile['state_period_s']:
            return
        self.last_state, self.last_state_key = now, key
        self.sequence += 1
        stamp = self.get_clock().now().to_msg()
        self.state_pub.publish(json_msg({'session_id': self.policy.session_id,
            'sequence': self.sequence, 'stamp': {'sec': stamp.sec, 'nanosec': stamp.nanosec},
            'phase': self.policy.phase, 'subphase': self.policy.subphase,
            'reason': self.policy.reason, 'owner': self.gate.get('owner', 'NONE'),
            'home_valid': self.policy.home_valid, 'retry_count': self.policy.retry_count,
            'jump_count': self.policy.jump_count, 'docked_pose': self.policy.docked_pose,
            'exit_pose': self.policy.exit_pose, 'home': self.policy.home,
            'correction_m_rad': self.recovery.last_correction,
            'stopped': self.inputs.stopped(), 'action_idle': self.slot.idle}))
        if self.policy.home is not None and self.policy.home != self.last_home:
            self.home_pub.publish(pose_msg(self, self.policy.home[:2], self.policy.home[2]))
            self.last_home = self.policy.home
        if key != getattr(self, '_logged_key', None):
            self.get_logger().info(f'{key[0]} {key[1]}: {key[2]}')
            self._logged_key = key

    def tick(self):
        now = time.monotonic()
        self.update_pose(now)
        obs = self.observation()
        # Policy reacts to current TF/safety before consuming an action success.
        self.policy.tick(obs, self.recovery.total)
        if self.policy.phase == 'FOLLOW' and self.pose is not None:
            recovery = self.settings['recovery']
            if (self.last_motion_pose is None or
                math.dist(self.pose[:2], self.last_motion_pose[:2]) >= recovery['movement_translation_m'] or
                abs(wrap(self.pose[2] - self.last_motion_pose[2])) >= recovery['movement_rotation_rad']):
                self.last_motion_pose = tuple(self.pose)
                self.last_motion_at = now
            if now - self.last_motion_at >= self.settings['deadlines']['no_motion_s']:
                self.policy.retry_navigation(obs, 'NO_MOVEMENT')
        if self.slot.acceptance_overdue(now) and self.policy.subphase != 'CANCEL':
            self.policy.fail(obs, 'ACTION_ACCEPT_TIMEOUT')
        native_rejected = any(r['cancel_rejected'] and r['name'] in ('/dock', '/undock') for r in self.slot.records)
        if native_rejected or self.slot.cancel_overdue(now):
            self.request_estop('CANCEL_STOP_UNCONFIRMED')
            self.policy.pending = 'FAILED'
        self.process_results(obs)
        self.process_effects()
        self.publish_control()
        self.publish_state()

    def stop_before_shutdown(self):
        self.policy.cancel(self.observation(), 'PI_SHUTDOWN')
        self.process_effects()
        deadline = time.monotonic() + self.settings['deadlines']['action_cancel_s'] + 1.0
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_control()
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.slot.idle and self.inputs.stopped():
                self.policy.native_hold = False
                self.policy.change('FAILED', time.monotonic(), reason='PI_SHUTDOWN')
                self.publish_control()
                return
        self.request_estop('CANCEL_STOP_UNCONFIRMED')
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ExplorationCoordinator()
    previous = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    stopping = False
    def interrupted(_signum, _frame):
        nonlocal stopping
        stopping = True
    for sig in previous:
        signal.signal(sig, interrupted)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.stop_before_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    main()
