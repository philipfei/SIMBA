"""Static launch-contract tests that run without ROS hardware or Nav2 imports."""
import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
LAUNCH = ROOT / 'simba_bringup' / 'launch'


def source(name):
    path = LAUNCH / name
    text = path.read_text(encoding='utf-8')
    ast.parse(text, filename=str(path))
    return text


def test_coverage_launch_has_exact_nav2_process_contract_and_remap():
    text = source('coverage.launch.py')
    expected = {
        "_node('nav2_map_server', 'map_server'",
        "_node('nav2_amcl', 'amcl'",
        "_node('nav2_planner', 'planner_server'",
        "_node('nav2_controller', 'controller_server'",
        "_node('nav2_lifecycle_manager', 'lifecycle_manager'",
    }
    assert all(token in text for token in expected)
    assert "CONTROLLER_REMAPS = [('cmd_vel', '/cmd_vel_nav'), ('/cmd_vel', '/cmd_vel_nav')]" in text
    assert "'plan_file'" in text and text.count("'plan_file'")==1
    # An exported plan adds only bt_navigator (NavigateThroughPoses), under the supervisor's lease.
    assert "if plan:\n        nodes.append(_node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', parameters=[str(nav_file)]))" in text
    assert "(['bt_navigator'] if plan else [])" in text
    for forbidden in ('waypoint_follower', 'smoother_server', 'velocity_smoother',
                      'behavior_server', 'nav2_behaviors', 'collision_monitor'):
        assert forbidden not in text


def test_navigate_through_poses_tree_has_no_motion_behaviors():
    tree = (ROOT / 'simba_bringup' / 'config' / 'navigate_through_poses.xml').read_text(encoding='utf-8')
    # Spin/BackUp/DriveOnHeading/Wait need behavior_server, whose motion would bypass the gate lease.
    for forbidden in ('<Spin', '<BackUp', '<DriveOnHeading', '<Wait ', '<AssistedTeleop',
                      'ControllerSelector', 'PlannerSelector'):
        assert forbidden not in tree
    assert '<RemovePassedGoals input_goals="{goals}" output_goals="{goals}" radius="0.25"/>' in tree


def test_each_motion_mode_launches_one_velocity_gate_and_no_direct_cmd_vel_remap():
    for name in ('manual.launch.py', 'slam.launch.py', 'coverage.launch.py'):
        text = source(name)
        assert text.count("'velocity_gate'") == 1
        assert "'/cmd_vel_remote'" not in text
        if name != 'coverage.launch.py':
            assert "'/cmd_vel_nav'" not in text


def test_launches_have_bounded_shutdown_and_no_respawn():
    for name in ('lidar.launch.py', 'manual.launch.py', 'slam.launch.py',
                 'coverage.launch.py', 'rviz.launch.py'):
        text = source(name)
        assert "sigterm_timeout='6'" in text
        assert "sigkill_timeout='2'" in text
        assert 'respawn=True' not in text
