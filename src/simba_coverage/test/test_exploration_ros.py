"""Fake-robot exploration integration on loopback only, never physical hardware."""
import os
os.environ['ROS_DOMAIN_ID'] = '91'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
os.environ['CYCLONEDDS_URI'] = '<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="lo" multicast="false"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
import time
import numpy as np
import pytest
pytest.importorskip('rclpy')
pytest.importorskip('nav2_msgs')
pytest.importorskip('irobot_create_msgs')
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.msg import Costmap
from nav2_msgs.action import ComputePathToPose, FollowPath
from irobot_create_msgs.action import Dock, Undock
from simba_bringup.exploration_coordinator import ExplorationCoordinator, occupancy_grid, costmap_grid
from simba_coverage.gate_node import SafetyGate
from simba_coverage.ros_common import LATCHED, path_msg, decode
from test_ros import FakeInputs


@pytest.fixture
def exploration_ros():
    rclpy.init(args=['--ros-args', '-p', 'mapping_mode:=true', '-p', 'supervised_mapping:=true', '-p', 'auto_undock:=false'])
    executor = MultiThreadedExecutor(num_threads=4)
    nodes = []
    servers = []
    def add(node):
        nodes.append(node)
        executor.add_node(node)
        return node
    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
    def until(condition, timeout=8.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if condition():
                return
            pump(.02)
        assert condition(), 'Timed out waiting for fake-robot state'
    gate = add(SafetyGate())
    coordinator = add(ExplorationCoordinator())
    fake = add(FakeInputs())
    fake.request_owner = None
    fake.publish_trust = False
    fake.docked = True
    for name in ('planner_server', 'controller_server', 'slam_toolbox'):
        add(Node(name))
    publisher = add(Node('fake_costmap_publisher'))
    map_pub = publisher.create_publisher(OccupancyGrid, '/map', LATCHED)
    global_pub = publisher.create_publisher(Costmap, '/global_costmap/costmap_raw', LATCHED)
    local_pub = publisher.create_publisher(Costmap, '/local_costmap/costmap_raw', LATCHED)
    cells = np.zeros((80, 80), dtype=np.int8)
    cells[[0, -1], :] = 100
    cells[:, [0, -1]] = 100
    def publish_map():
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.header.stamp = publisher.get_clock().now().to_msg()
        m.info.resolution = .05
        m.info.width = m.info.height = 80
        m.info.origin.orientation.w = 1.
        m.data = cells.ravel().tolist()
        map_pub.publish(m)
    def publish_costmaps():
        for pub, frame in ((global_pub, 'map'), (local_pub, 'odom')):
            m = Costmap()
            m.header.frame_id = frame
            m.header.stamp = publisher.get_clock().now().to_msg()
            m.metadata.resolution = .05
            m.metadata.size_x = m.metadata.size_y = 80
            m.metadata.origin.orientation.w = 1.
            m.data = np.where(cells == 100, 254, 0).astype(np.uint8).ravel().tolist()
            pub.publish(m)
    publisher.create_timer(2., publish_map)
    publisher.create_timer(.2, publish_costmaps)
    publish_map()
    native_owners = []
    def undock(handle):
        native_owners.append(gate.policy.owner)
        fake.docked = False
        fake.tick()
        handle.succeed()
        return Undock.Result(is_docked=False)
    def plan(handle):
        goal = handle.request.goal.pose
        result = ComputePathToPose.Result()
        result.path = path_msg(publisher, [[fake.x, fake.y], [goal.position.x, goal.position.y]], 0.)
        handle.succeed()
        return result
    def follow(handle):
        goal = handle.request.path.poses[-1].pose
        fake.x, fake.y = goal.position.x, goal.position.y
        fake.tick()
        time.sleep(.15)
        handle.succeed()
        return FollowPath.Result()
    def dock(handle):
        native_owners.append(gate.policy.owner)
        fake.docked = True
        fake.tick()
        handle.succeed()
        return Dock.Result(is_docked=True)
    for typ, name, callback in ((Undock, '/undock', undock), (Dock, '/dock', dock),
                                (ComputePathToPose, '/compute_path_to_pose', plan), (FollowPath, '/follow_path', follow)):
        servers.append(ActionServer(publisher, typ, name, execute_callback=callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=ReentrantCallbackGroup()))
    try:
        yield add, pump, until, gate, coordinator, fake, native_owners
    finally:
        executor.shutdown()
        for server in servers:
            server.destroy()
        for action_client in coordinator.slot.clients.values():
            action_client.destroy()
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


def test_fake_undock_manual_release_return_dock_and_late_attachment(exploration_ros):
    add, pump, until, gate, coordinator, fake, owners = exploration_ros
    coordinator.policy.auto = True
    until(lambda: coordinator.policy.phase == 'MANUAL', 10.)
    assert coordinator.policy.home_valid and owners == ['NATIVE']
    client = add(Node('late_exploration_client'))
    states, homes, paths = [], [], []
    client.create_subscription(String, '/exploration/state', lambda m: states.append(decode(m)), LATCHED)
    client.create_subscription(PoseStamped, '/exploration/home', homes.append, LATCHED)
    client.create_subscription(Path, '/exploration/path', paths.append, LATCHED)
    until(lambda: states and homes and paths)
    assert states[-1]['home_valid'] and homes[-1].pose.position.x == pytest.approx(1.35)
    manual = client.create_client(SetBool, '/control/manual')
    returned = client.create_client(Trigger, '/exploration/return_home')
    def call(service, request):
        until(service.service_is_ready)
        future = service.call_async(request)
        until(future.done)
        return future.result()
    assert call(manual, SetBool.Request(data=True)).success
    # An H service call alone must not steal MANUAL from the UI.
    until(lambda: gate.policy.owner == 'MANUAL')
    assert not call(returned, Trigger.Request()).success
    assert call(manual, SetBool.Request(data=False)).success
    until(lambda: gate.policy.owner == 'NONE' and coordinator.gate.get('owner') == 'NONE')
    assert call(returned, Trigger.Request()).success
    until(lambda: coordinator.policy.phase == 'DOCKED', 15.)
    assert owners == ['NATIVE', 'NATIVE']
    assert call(returned, Trigger.Request()).success
    assert not call(manual, SetBool.Request(data=True)).success
    assert any(path.poses for path in paths)
    until(lambda: paths and not paths[-1].poses)
    assert states[-1]['sequence'] > states[0]['sequence']


def test_live_costmap_growth_and_native_independence_from_slam_trust(exploration_ros):
    _, pump, until, gate, coordinator, fake, _ = exploration_ros
    pump(2.5)
    assert gate.supervised_mapping and gate.graph_ok
    # Native supervision relies on leases/sensors, not map trust.
    coordinator.policy.change('DOCK', time.monotonic())
    coordinator.localization = 'TF_STALE'
    coordinator.publish_control()
    pump(.2)
    assert gate.policy.owner == 'NATIVE' and not fake.estops
    # Full-grid messages retain new size/origin instead of a fixed initial extent.
    m = OccupancyGrid()
    m.info.width, m.info.height, m.info.resolution = 100, 80, .05
    m.info.origin.position.x = -1.
    m.info.origin.orientation.w = 1.
    m.data = [0] * 8000
    grid = occupancy_grid(m)
    assert grid.cells.shape == (80, 100) and grid.origin[0] == -1.
    c = Costmap()
    c.metadata.size_x, c.metadata.size_y, c.metadata.resolution = 100, 80, .05
    c.metadata.origin = m.info.origin
    c.data = [0] * 8000
    assert costmap_grid(c).cells.shape == grid.cells.shape


def test_candidate_clearance_is_lazy_and_cached():
    from simba_bringup.exploration_coordinator import CandidateChecks
    visited = []
    checks = CandidateChecks([.65, .75, .55], lambda value: visited.append(value) or True)
    assert len(checks) == 3
    assert checks[0] and checks[0]
    assert visited == [.65]


def test_closed_terminal_output_does_not_interrupt_cleanup(monkeypatch):
    from simba_bringup.exploration_teleop import display
    def closed(*args, **kwargs):
        raise OSError('Terminal closed')
    monkeypatch.setattr('builtins.print', closed)
    display('Cleanup still proceeds', flush=True)


def test_launch_preflight_with_only_private_context():
    """Launch has no default rclpy context; preflight must own its executor."""
    import subprocess
    import sys
    result = subprocess.run([sys.executable, '-c',
        'from simba_bringup.mode_guard import preflight; preflight(None)'],
        capture_output=True, text=True, timeout=10.)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'Exception ignored' not in result.stderr


@pytest.mark.parametrize('module_name,node_name', [
    ('simba_bringup.exploration_coordinator', 'ExplorationCoordinator'),
    ('simba_coverage.gate_node', 'SafetyGate')])
def test_shutdown_signal_does_not_interrupt_executor_callback(monkeypatch, module_name, node_name):
    import importlib
    import signal
    module = importlib.import_module(module_name)
    handlers, events = {}, []
    class FakeNode:
        def stop_before_shutdown(self): events.append('stop')
        def destroy_node(self): events.append('destroy')
    monkeypatch.setattr(module, node_name, FakeNode)
    monkeypatch.setattr(module.rclpy, 'init', lambda **_: None)
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(module.rclpy, 'shutdown', lambda: events.append('shutdown'))
    monkeypatch.setattr(module.signal, 'signal', lambda sig, handler: handlers.update({sig: handler}))
    def spin(*args, **kwargs):
        handlers[signal.SIGINT](signal.SIGINT, None)
        events.append('callback completed')
    monkeypatch.setattr(module.rclpy, 'spin_once', spin)
    if node_name == 'SafetyGate':
        monkeypatch.setattr(module, '_stop_before_shutdown', lambda node: node.stop_before_shutdown())
    module.main()
    assert events == ['callback completed', 'stop', 'destroy', 'shutdown']


def test_startup_reports_gate_fault_instead_of_graph_settling(exploration_ros):
    _, pump, _, gate, coordinator, fake, _ = exploration_ros
    pump(2.5)
    fake.wheels = False
    pump(.3)
    coordinator.policy.auto = True
    observation = coordinator.observation()
    assert observation.gate_reason == 'WHEELS_DISABLED'
    assert observation.sensor_reason != 'GRAPH_SETTLING'
    coordinator.policy.tick(observation)
    assert coordinator.policy.reason == 'WHEELS_DISABLED'


def test_native_terminal_status_reconciles_only_matching_goal(exploration_ros):
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from unique_identifier_msgs.msg import UUID
    from types import SimpleNamespace
    from simba_bringup.exploration import approaches
    _, pump, _, _, coordinator, fake, _ = exploration_ros
    pump(2.5)
    coordinator.policy.docked_pose = tuple(coordinator.pose)
    coordinator.policy.candidates = approaches(coordinator.pose, coordinator.profile['approach_offsets_m'])
    coordinator.policy.home = coordinator.policy.candidates[0]
    coordinator.policy.change('UNDOCK', time.monotonic())
    goal_id = UUID(uuid=[1]*16)
    record = {'name': '/undock', 'handle': SimpleNamespace(goal_id=goal_id),
              'key': coordinator.mission.action_id(), 'canceled': False,
              'sent': time.monotonic(), 'cancel_at': None, 'cancel_rejected': False}
    coordinator.slot.records.append(record)
    status = GoalStatus()
    status.goal_info.goal_id = UUID(uuid=[2]*16)
    status.status = GoalStatus.STATUS_SUCCEEDED
    coordinator.receive_native_status('/undock', GoalStatusArray(status_list=[status]))
    assert 'native_terminal' not in record
    status.goal_info.goal_id = goal_id
    coordinator.receive_native_status('/undock', GoalStatusArray(status_list=[status]))
    observation = coordinator.observation()
    observation.docked = None
    coordinator.reconcile_native_terminal(observation)
    assert not coordinator.slot.idle  # Missing dock status cannot authorize handoff.
    observation.docked = False
    observation.stopped = False
    coordinator.reconcile_native_terminal(observation)
    assert not coordinator.slot.idle
    observation.stopped = True
    coordinator.reconcile_native_terminal(observation)
    assert coordinator.slot.idle
    coordinator.process_results(observation)
    assert coordinator.policy.subphase == 'UNDOCK_CONFIRM'
    coordinator.policy.tick(observation)
    assert coordinator.policy.phase == 'MANUAL' and coordinator.policy.home_valid
    coordinator.slot.finish(record, 4, Undock.Result(is_docked=False))
    assert not coordinator.slot.events  # A late GetResult cannot complete twice.


def test_cancel_request_is_sent_once_even_with_late_acceptance():
    from types import SimpleNamespace
    from rclpy.task import Future
    from simba_coverage.ros_common import ActionSlot
    from simba_coverage.state import Mission
    mission = Mission()
    slot = ActionSlot(SimpleNamespace(), mission)
    sent = []
    def cancel():
        sent.append(True)
        return Future()
    record = {'key':mission.action_id(), 'name':'/undock', 'handle':None,
              'canceled':False, 'cancel_at':None, 'cancel_sent':False, 'cancel_rejected':False}
    slot.records.append(record)
    slot.cancel_record(record)
    deadline = record['cancel_at']
    record['handle'] = SimpleNamespace(cancel_goal_async=cancel)
    slot.cancel_record(record)
    slot.cancel_record(record)
    assert len(sent) == 1 and record['cancel_at'] == deadline


def test_manual_heartbeat_continues_while_service_reply_is_delayed():
    import threading
    from std_msgs.msg import Empty
    from simba_bringup.exploration_teleop import ExplorationTeleop
    from simba_coverage.ros_common import json_msg
    rclpy.init(args=[])
    server = Node('slow_manual_grant_server')
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server)
    heartbeats = []
    server.create_subscription(Empty, '/control/manual_heartbeat', lambda _: heartbeats.append(time.monotonic()), 10)
    def grant(request, response):
        time.sleep(.7)
        response.success = True
        return response
    server.create_service(SetBool, '/test/manual_grant', grant, callback_group=ReentrantCallbackGroup())
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    ui = ExplorationTeleop(None)
    ui.manual = ui.create_client(SetBool, '/test/manual_grant')
    stamp = ui.get_clock().now().to_msg()
    ui.receive_state(json_msg({'session_id':'test', 'sequence':1, 'stamp':{'sec':stamp.sec,'nanosec':stamp.nanosec},
                              'phase':'MANUAL', 'subphase':'', 'reason':'', 'owner':'NONE'}))
    try:
        ui.request_manual(True)
        assert len(heartbeats) >= 4 and ui.manual_requested and not ui.granted
        assert max(b-a for a,b in zip(heartbeats,heartbeats[1:])) < .3
    finally:
        ui.destroy_node()
        executor.shutdown()
        thread.join(timeout=2.)
        server.destroy_node()
        rclpy.shutdown()
