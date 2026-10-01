"""State-table-driven exploration tests; no ROS graph or robot connection."""
import os
from pathlib import Path
import yaml
import pytest
from simba_bringup.exploration import Exploration, Observation, SlamRecovery, approaches, at_home, key_action, load_profile
from simba_bringup.mode_guard import acquire_lock, conflicts
from simba_coverage.params import load_settings

CONFIG = Path(__file__).resolve().parents[2] / 'simba_bringup' / 'config'
PHASES = ('WAIT_READY', 'UNDOCK', 'MANUAL', 'HANDOVER', 'PLAN', 'FOLLOW', 'DOCK', 'DOCKED', 'FAILED')


def session(auto=True):
    settings = load_settings(CONFIG)
    return Exploration(load_profile(CONFIG), settings, 0., auto)


def observe(policy, now=3., **values):
    defaults = dict(now=now, stamp=now, stopped=True, gate_owner=policy.owner,
                    gate_epoch=policy.epoch, gate_healthy=True, docked=False,
                    pose=(2., 2., 0.), localization='', map_seq=1, map_stamp=now,
                    map_identity='map-a', costmap_ready=True, candidate_valid=(True, True, True))
    defaults.update(values)
    return Observation(**defaults)


def ready_manual():
    policy = session()
    policy.tick(observe(policy, 2.1, docked=True))
    assert policy.subphase == 'UNDOCK_ARM'
    policy.tick(observe(policy, 2.2, docked=True))
    assert policy.phase == 'UNDOCK'
    assert policy.effects == [('undock', None)]
    policy.effects.clear()
    policy.action_result('undock', True, False, observe(policy, 2.3))
    policy.tick(observe(policy, 2.4))
    assert policy.phase == 'MANUAL' and policy.home_valid
    return policy


@pytest.mark.parametrize('phase', PHASES)
@pytest.mark.parametrize('key', ['w', 's', 'a', 'd', 'h', 'x', ' ', 'q', 'D', 'R', 'r'])
def test_state_table_keys(phase, key):
    action = key_action(phase, key)
    expected = ('detach' if key == 'D' else 'quit' if key == 'q' else
                ('reset' if phase in ('MANUAL', 'FAILED') else 'reset_blocked') if key == 'R' else
                ('stop' if phase == 'MANUAL' else 'cancel') if key in ('x', ' ') else
                ('return' if phase in ('MANUAL', 'FAILED', 'DOCKED') else 'busy') if key == 'h' else
                'move' if key in ('w', 's', 'a', 'd') and phase == 'MANUAL' else 'ignore')
    assert action == expected


def test_manual_release_then_return_and_late_release_cannot_revoke_navigation():
    policy = ready_manual()
    assert policy.manual_request(True, observe(policy))[0]
    assert policy.owner == 'MANUAL'
    assert not policy.request_return(observe(policy))[0]
    policy.manual_request(False, observe(policy))
    assert not policy.request_return(observe(policy, gate_owner='MANUAL'))[0]
    assert policy.request_return(observe(policy, gate_owner='NONE'))[0]
    policy.change('FOLLOW', 3.)
    assert policy.owner == 'NAV'
    policy.manual_request(False, observe(policy))
    assert policy.owner == 'NAV'
    assert policy.request_return(observe(policy))[0] and not policy.effects


@pytest.mark.parametrize('docked, reason', [(None, 'DOCK_STATUS_UNCONFIRMED'), (False, 'NOT_DOCKED')])
def test_dispatch_rechecks_dock_status(docked, reason):
    policy = session()
    policy.tick(observe(policy, 2., docked=True))
    policy.tick(observe(policy, 2.1, docked=docked))
    assert policy.subphase == 'CANCEL' and policy.reason == reason
    assert not any(kind == 'undock' for kind, _ in policy.effects)


def test_diagnostic_startup_and_native_cancel_do_not_restart():
    policy = session(False)
    policy.tick(observe(policy, 100., docked=True))
    assert policy.phase == 'WAIT_READY' and not policy.effects
    policy = session()
    policy.change('DOCK', 1.)
    policy.cancel(observe(policy, 2., slot_idle=False, stopped=False))
    assert policy.owner == 'NATIVE'
    policy.tick(observe(policy, 4.1, slot_idle=False, stopped=False))
    assert ('estop', 'CANCEL_STOP_UNCONFIRMED') in policy.effects
    policy.action_result('dock', True, True, observe(policy, 4.2))
    assert policy.subphase == 'CANCEL'
    policy.tick(observe(policy, 4.3))
    assert policy.phase == 'FAILED' and policy.owner == 'NONE'


def test_manual_heartbeat_expiry_and_terminal_docked():
    policy = ready_manual()
    policy.manual_request(True, observe(policy, 3.))
    policy.tick(observe(policy, 3.6, gate_owner='NONE'))
    assert not policy.manual and policy.owner == 'NONE'
    policy.change('DOCKED', 4.)
    policy.cancel(observe(policy, 4.1))
    assert policy.phase == 'DOCKED'
    assert policy.request_return(observe(policy))[0]
    assert not policy.manual_request(True, observe(policy))[0]


@pytest.mark.parametrize('scenario', ['isolated', 'repeated', 'tf_loss', 'long_burst', 'ordinary_motion'])
def test_recovery_escalation(scenario):
    recovery = SlamRecovery(load_profile(CONFIG))
    pose = (0., 0., 0.)
    assert recovery.observe(0., pose, pose) == 'SLAM_SETTLING'
    assert recovery.observe(2., pose, pose) == ''
    if scenario == 'tf_loss':
        assert recovery.observe(3., None, None) == 'TF_WAIT'
        assert recovery.observe(5., None, None) == 'TF_STALE'
    elif scenario == 'ordinary_motion':
        assert recovery.observe(3., (1., 0., .2), (1., 0., .2)) == ''
    elif scenario == 'long_burst':
        for t in range(3, 14):
            result = recovery.observe(float(t), (.2 * t, 0., 0.), pose)
        assert result == 'SLAM_UNSTABLE' and recovery.total == 1
    else:
        assert recovery.observe(3., (.2, 0., 0.), pose) == 'SLAM_SETTLING'
        assert recovery.observe(5., (.2, 0., 0.), pose) == ''
        if scenario == 'repeated':
            assert recovery.observe(6., (.4, 0., 0.), pose) == 'SLAM_SETTLING'
            assert recovery.observe(8., (.4, 0., 0.), pose) == ''
            assert recovery.observe(9., (.6, 0., 0.), pose) == 'SLAM_UNSTABLE'


def test_correction_stops_waits_fresh_map_then_replans():
    policy = ready_manual()
    policy.request_return(observe(policy, 3., gate_owner='NONE'))
    policy.change('FOLLOW', 3.)
    policy.tick(observe(policy, 4., localization='SLAM_SETTLING', slot_idle=False), 1)
    assert policy.owner == 'NONE' and policy.subphase == 'SLAM_SETTLING'
    assert ('cancel', None) in policy.effects
    policy.effects.clear()
    policy.tick(observe(policy, 6., map_seq=2), 1)
    assert policy.subphase == 'MAP_WAIT'
    policy.tick(observe(policy, 6.1, map_seq=2, map_stamp=6.), 1)
    assert policy.subphase == 'MAP_WAIT'
    policy.tick(observe(policy, 7., map_seq=3), 1)
    assert policy.phase == 'PLAN' and policy.jump_count == 1


@pytest.mark.parametrize('valid, selected', [((True, True, True), 0), ((False, True, True), 1),
                                          ((False, False, True), 2), ((False, False, False), None)])
def test_approach_candidate_selection(valid, selected):
    policy = ready_manual()
    assert policy.candidates == approaches((2., 2., 0.), [.65, .75, .55])
    obs = observe(policy, candidate_valid=valid)
    policy.plan_next(obs)
    if selected is None:
        assert policy.subphase == 'STOP' and policy.retry_count == 1
    else:
        assert policy.effects == [('plan', policy.candidates[selected])]
        policy.effects.clear()
        policy.action_result('plan', True, 'path', obs)
        assert policy.subphase == 'NAV_ARM'
        assert not any(kind == 'plan' for kind, _ in policy.effects)


def test_identical_no_path_is_not_repeated_and_handoff_checked():
    policy = ready_manual()
    obs = observe(policy)
    policy.plan_next(obs)
    for _ in range(3):
        policy.action_result('plan', False, None, obs)
    assert policy.retry_count == 1
    policy.effects.clear()
    policy.tick(observe(policy, 4., gate_owner='NONE'))
    policy.tick(observe(policy, 5., map_seq=2))
    assert policy.subphase == 'CANCEL' and policy.reason == 'NO_PATH'
    assert not any(kind == 'plan' for kind, _ in policy.effects)
    profile = policy.cfg
    assert at_home((1.35, 2., .05), (1.35, 2., 0.), profile)
    assert not at_home((1.41, 2., 0.), (1.35, 2., 0.), profile)
    assert not at_home((1.35, 2., .06), (1.35, 2., 0.), profile)


@pytest.mark.parametrize('visible, clear, result', [(2, True, 'DOCK_ARM'), (0, True, 'DOCK_READY'), (2, False, 'CANCEL')])
def test_dock_handoff_checks(visible, clear, result):
    policy = ready_manual()
    policy.change('HANDOVER', 3., 'DOCK_READY')
    policy.tick(observe(policy, 3.1, pose=policy.home, visible_samples=visible, local_clear=clear))
    assert policy.subphase == result


def test_map_and_costmap_barriers_are_bounded():
    for maps, reason in [(False, 'MAP_STALE'), (True, 'COSTMAP_STALE')]:
        policy = ready_manual()
        policy.wait_map(observe(policy, 3.))
        if maps:
            policy.tick(observe(policy, 4., map_seq=2, costmap_ready=False))
            policy.tick(observe(policy, 6.1, map_seq=2, costmap_ready=False))
        else:
            policy.tick(observe(policy, 9.1))
        assert policy.reason == reason and policy.subphase == 'CANCEL'


def test_mode_lock_and_graph_conflicts(tmp_path):
    descriptor = acquire_lock(tmp_path)
    try:
        with pytest.raises(RuntimeError, match='MODE_CONFLICT'):
            acquire_lock(tmp_path)
    finally:
        os.close(descriptor)
    assert conflicts(['planner_server', 'unrelated'], ['rogue']) == ['planner_server', '/cmd_vel publisher: rogue']


def test_live_map_render_and_fixed_map_compatibility(tmp_path):
    settings = load_settings(CONFIG)
    old_hash = settings.hash
    fixed = yaml.safe_load(settings.render_nav2('/tmp/map.yaml', tmp_path / 'fixed').read_text())
    live = yaml.safe_load(settings.render_nav2(directory=tmp_path / 'live', live_map=True).read_text())
    assert 'amcl' in fixed and 'map_server' in fixed
    assert 'amcl' not in live and 'map_server' not in live
    assert fixed['controller_server']['ros__parameters']['goal_checker']['xy_goal_tolerance'] == .05
    assert live['controller_server']['ros__parameters']['goal_checker']['xy_goal_tolerance'] == .03
    assert live['controller_server']['ros__parameters']['goal_checker']['yaw_goal_tolerance'] == .05
    glob = live['global_costmap']['global_costmap']['ros__parameters']
    assert glob['rolling_window'] is False and 'width' not in glob and 'height' not in glob
    assert glob['static_layer']['map_topic'] == '/map' and not glob['static_layer']['subscribe_to_updates']
    assert glob['update_frequency'] == 2. and settings.hash == old_hash


def test_exploration_installed_data_contract():
    root = CONFIG.parent
    for relative in ('launch/explore.launch.py', 'config/exploration_params.yaml', 'rviz/exploration.rviz'):
        assert (root / relative).is_file()
    rviz = yaml.safe_load((root / 'rviz/exploration.rviz').read_text())
    topics = {d.get('Topic', {}).get('Value'): d.get('Topic') for d in rviz['Visualization Manager']['Displays']}
    for name in ('/exploration/home', '/exploration/path'):
        assert topics[name]['Durability Policy'] == 'Transient Local'


def test_installed_launch_generates_live_profile(tmp_path, monkeypatch):
    prefix = os.environ.get('SIMBA_TEST_INSTALL_PREFIX')
    if not prefix:
        pytest.skip('Set SIMBA_TEST_INSTALL_PREFIX after a temporary-prefix colcon build')
    from launch import LaunchContext
    import runpy
    root = Path(prefix) / 'simba_bringup'
    share = root / 'share/simba_bringup'
    for resource in ('launch/explore.launch.py', 'rviz/exploration.rviz', 'config/exploration_params.yaml'):
        assert (share / resource).is_file()
    assert os.access(root / 'lib/simba_bringup/exploration_coordinator', os.X_OK)
    module = runpy.run_path(str(share / 'launch/explore.launch.py'))
    monkeypatch.setattr(module['tempfile'], 'mkdtemp', lambda **_: str(tmp_path))
    context = LaunchContext()
    context.launch_configurations.update(config_dir=str(share / 'config'), auto_undock='false', lidar_driver='false')
    actions = module['_setup'](context)  # Construct only: do not execute ROS nodes or preflight.
    assert actions
    nav = yaml.safe_load(next(tmp_path.glob('nav2-*.yaml')).read_text())
    assert 'amcl' not in nav and 'map_server' not in nav
    assert nav['global_costmap']['global_costmap']['ros__parameters']['rolling_window'] is False
    assert nav['controller_server']['ros__parameters']['goal_checker']['yaw_goal_tolerance'] == .05


def test_pc_teleop_does_not_import_robot_action_messages():
    pytest.importorskip('rclpy')
    import subprocess
    import sys
    script = '''
import importlib.abc
import sys
class NoRobotMessages(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('irobot_create_msgs', 'nav2_msgs'):
            raise ModuleNotFoundError(fullname)
sys.meta_path.insert(0, NoRobotMessages())
from simba_bringup.exploration_teleop import ExplorationTeleop
'''
    result = subprocess.run([sys.executable, '-c', script], capture_output=True, text=True, timeout=10.)
    assert result.returncode == 0, result.stderr


def test_repeated_cancel_preserves_cause_and_deadline():
    policy = ready_manual()
    policy.change('UNDOCK', 3.)
    policy.home_valid = False
    policy.cancel(observe(policy, 4.))
    cause, deadline, effects = policy.reason, policy.since, list(policy.effects)
    policy.cancel(observe(policy, 5.5))
    assert (policy.reason, policy.since, policy.effects) == (cause, deadline, effects)
    policy.tick(observe(policy, 6.1, slot_idle=False))
    assert policy.reason == 'CANCEL_STOP_UNCONFIRMED'
    policy.cancel(observe(policy, 7.))
    assert policy.reason == 'CANCEL_STOP_UNCONFIRMED' and policy.since == deadline
