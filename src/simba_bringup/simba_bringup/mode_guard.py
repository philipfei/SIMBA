"""Launch-lifetime local mode lock and bounded, read-only DDS preflight."""
import fcntl
import os
from pathlib import Path
import time


CONFLICT_NODES = {
    'velocity_safety_gate', 'coverage_supervisor', 'coverage_meter',
    'exploration_coordinator', 'slam_toolbox', 'amcl', 'map_server',
    'planner_server', 'controller_server', 'simba_tf_relay',
}
_locks = []  # Retain descriptors for the launch process lifetime.


def conflicts(nodes, publishers):
    found = sorted(set(nodes).intersection(CONFLICT_NODES))
    return found + [f'/cmd_vel publisher: {name}' for name in publishers]


def acquire_lock(directory=None):
    root = Path(directory or '/tmp')
    path = root / f'simba-mode-{os.getuid()}-{os.environ.get("ROS_DOMAIN_ID", "0")}.lock'
    descriptor = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        os.close(descriptor)
        raise RuntimeError('SIMBA_MODE_CONFLICT: another SIMBA launch holds the mode lock') from None
    return descriptor


def preflight(_launch_context, *args, **kwargs):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    descriptor = acquire_lock()
    context = Context()
    node = None
    executor = None
    try:
        rclpy.init(args=[], context=context)
        node = Node('simba_launch_preflight', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
            found = conflicts(
                [name for name, _ in node.get_node_names_and_namespaces()],
                [p.node_name for p in node.get_publishers_info_by_topic('/cmd_vel')])
            if found:
                raise RuntimeError('SIMBA_MODE_CONFLICT: ' + ', '.join(found))
        _locks.append(descriptor)
    except BaseException:
        os.close(descriptor)
        raise
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()
    return []
