"""Run coverage.launch.py plan:=... (real Nav2 + SIMBA nodes) against fake_create3.py and record the run.

Acts like an operator: sets the AMCL initial pose, waits until localization is trusted and converged,
calls /coverage/preview and /coverage/start, resumes after pauses listed in --resume-on, and stops when
the task finishes, fails, or stays paused. Everything is written to tools/sim/runs/<name>/.
Source tools/sim/env.sh first.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from nav2_msgs.action import NavigateThroughPoses
from simba_coverage.ros_common import LATEST

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
EXAMPLES = REPO / 'tools/coverage_tool/examples'
T0 = time.monotonic()


def since():
    return round(time.monotonic() - T0, 2)


class Monitor(Node):
    def __init__(self):
        super().__init__('sim_monitor')
        self.state = {}
        self.timeline = []
        self.feedback = []
        self.amcl = []
        self.map_odom = []  # [arrival s, stamp - arrival s] for every map->odom transform from AMCL
        self.trust_reasons = []
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.create_subscription(String, '/coverage/state', self.on_state, LATEST)
        self.create_subscription(NavigateThroughPoses.Impl.FeedbackMessage,
                                 '/navigate_through_poses/_action/feedback', self.on_feedback, 10)
        # Not self.services/self.clients: those are read-only Node properties.
        self.triggers = {n: self.create_client(Trigger, '/coverage/' + n) for n in ('preview', 'start', 'resume')}

    def on_amcl(self, msg):
        c = msg.pose.covariance
        p = msg.pose.pose
        self.amcl.append([since(), p.position.x, p.position.y,
                          math.atan2(2 * p.orientation.w * p.orientation.z, 1 - 2 * p.orientation.z ** 2),
                          math.sqrt(max(c[0], c[7])), math.degrees(math.sqrt(c[35]))])

    def on_tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                now = self.get_clock().now().nanoseconds * 1e-9
                self.map_odom.append([since(), round(t.header.stamp.sec + t.header.stamp.nanosec * 1e-9 - now, 3)])

    def on_state(self, msg):
        d = json.loads(msg.data)
        if d.get('trust_reason') and (not self.trust_reasons or self.trust_reasons[-1][1] != d['trust_reason']):
            self.trust_reasons.append([since(), d['trust_reason']])
        if not self.timeline or self.timeline[-1][1:3] != [d['state'], d['reason']]:
            self.timeline.append([round(since(), 1), d['state'], d['reason'], round(d['fraction'], 4)])
            print(f"[{self.timeline[-1][0]:7.1f}s] {d['state']:12s} {d['reason']}  fraction={d['fraction']:.3f}", flush=True)
        self.state = d

    def on_feedback(self, msg):
        f = msg.feedback
        self.feedback.append([round(since(), 1), f.number_of_poses_remaining, f.number_of_recoveries])

    def initial_pose(self, x, y, yaw):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = .05 ** 2
        m.pose.covariance[35] = math.radians(5) ** 2
        self.init_pub.publish(m)

    def call(self, name):
        c = self.triggers[name]
        if not c.wait_for_service(timeout_sec=2.):
            return False, 'service unavailable'
        f = c.call_async(Trigger.Request())
        end = time.monotonic() + 10
        while not f.done() and time.monotonic() < end:
            time.sleep(.05)
        return (f.result().success, f.result().message) if f.done() else (False, 'timeout')


def wait(cond, timeout, step=.2):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(step)
    return False


def other_runs_in_domain():
    """PIDs of coverage launches or fake robots already running in this ROS_DOMAIN_ID."""
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    found = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            cmd = (proc / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            env = (proc / 'environ').read_bytes().split(b'\0')
        except OSError:
            continue
        if ('coverage.launch.py' in cmd or 'fake_create3.py' in cmd or 'run_sim.py' in cmd) and \
                f'ROS_DOMAIN_ID={domain}'.encode() in env:
            found.append(int(proc.name))
    return found


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('name', help='run name; results go to tools/sim/runs/<name>/')
    ap.add_argument('--start', type=float, nargs=3, default=[.9, -3., 0.], metavar=('X', 'Y', 'YAW'),
                    help='true robot start pose in the map frame (default: 0.9 -3.0 0.0)')
    ap.add_argument('--obstacle', action='append', default=[], metavar='X,Y,R',
                    help='round obstacle that is in the world but not in the map; repeatable')
    ap.add_argument('--plan', default=str(EXAMPLES / 'map_ME_room1v4_coveragev3.yaml'))
    ap.add_argument('--map', default=str(EXAMPLES / 'map_ME_room1v4.yaml'))
    ap.add_argument('--config', default=str(REPO / 'src/simba_bringup/config'),
                    help='config directory passed to coverage.launch.py (copy and edit it to experiment)')
    ap.add_argument('--timeout', type=float, default=1500., help='max mission time in seconds')
    ap.add_argument('--settle-sigma', type=float, default=.03,
                    help='wait until the AMCL position sigma is below this before starting (m)')
    ap.add_argument('--resume-on', nargs='*', default=['TF_UNAVAILABLE', 'AMCL_STALE', 'SCAN_STALE'],
                    help='pause reasons after which to call /coverage/resume (at most 5 times)')
    a = ap.parse_args()
    if 'install' not in os.environ.get('AMENT_PREFIX_PATH', '') or 'ROS_DOMAIN_ID' not in os.environ:
        sys.exit('Source tools/sim/env.sh first (after colcon build).')
    busy = other_runs_in_domain()
    if busy:
        sys.exit(f'Another simulation is running in ROS_DOMAIN_ID={os.environ["ROS_DOMAIN_ID"]} (PIDs {busy}). '
                 'Stop it or use another domain.')
    if os.environ.get('RMW_IMPLEMENTATION') == 'rmw_fastrtps_cpp':
        # Segments left by killed processes break discovery of the next run.
        subprocess.run(['fastdds', 'shm', 'clean'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    out = HERE / 'runs' / a.name
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    mon = Monitor()
    threading.Thread(target=rclpy.spin, args=(mon,), daemon=True).start()
    result = {'name': a.name, 'start': a.start, 'obstacles': a.obstacle, 'plan': a.plan, 'map': a.map,
              'config': a.config, 'resumes': []}
    launch = robot = None
    try:
        launch = subprocess.Popen(
            ['ros2', 'launch', 'simba_bringup', 'coverage.launch.py', f'config_dir:={a.config}',
             f'map:={a.map}', f'plan:={a.plan}', f'output_dir:={out}/output', 'lidar_driver:=false'],
            stdout=open(out / 'launch.log', 'w'), stderr=subprocess.STDOUT, start_new_session=True)
        robot = subprocess.Popen(
            [sys.executable, str(HERE / 'fake_create3.py'), '--map', a.map, '--start', *map(str, a.start),
             '--log', str(out / 'truth.csv')] + [x for o in a.obstacle for x in ('--obstacle', o)],
            stdout=open(out / 'robot.log', 'w'), stderr=subprocess.STDOUT, start_new_session=True)
        result['robot_started_s'] = since()
        # Like an operator in RViz: set the initial pose until AMCL has taken it, then leave AMCL alone.
        if not wait(lambda: mon.amcl or mon.initial_pose(*a.start), 90, 2.):
            raise RuntimeError('AMCL never published a pose')
        if not wait(lambda: mon.state.get('trusted'), 60):
            raise RuntimeError('never trusted: ' + str(mon.state.get('trust_reason')))
        # A careful operator waits until AMCL has converged before starting.
        wait(lambda: mon.amcl and mon.amcl[-1][4] < a.settle_sigma, 120, .5)
        result['sigma_at_start'] = round(mon.amcl[-1][4], 4)
        print('AMCL sigma at start', result['sigma_at_start'], flush=True)
        print('preview:', *mon.call('preview'), flush=True)
        if not wait(lambda: mon.state.get('preview_ready') or 'PREVIEW_FAILED' in mon.state.get('reason', ''), 120):
            raise RuntimeError('preview timeout')
        if not mon.state.get('preview_ready'):
            raise RuntimeError(mon.state.get('reason'))
        started, message = False, ''
        for _ in range(30):
            started, message = mon.call('start')
            print('start:', started, message, flush=True)
            if started:
                break
            time.sleep(1.)
        if not started:
            raise RuntimeError('start refused: ' + message)
        t_start = time.monotonic()
        paused_since = [None]

        def done():
            s = mon.state.get('state')
            paused_since[0] = (paused_since[0] or time.monotonic()) if s == 'PAUSED' else None
            if paused_since[0] and time.monotonic() - paused_since[0] > 3 and \
                    mon.state.get('reason') in a.resume_on and len(result['resumes']) < 5:
                ok, msg = mon.call('resume')
                if ok:
                    print('resume:', msg, flush=True)
                    result['resumes'].append([round(since(), 1), mon.state.get('reason')])
                    paused_since[0] = None
            return s in ('FINISHED', 'FAILED', 'CANCELED') or (paused_since[0] and time.monotonic() - paused_since[0] > 15)
        wait(done, a.timeout, .5)
        result['mission_s'] = round(time.monotonic() - t_start, 1)
    except Exception as e:
        result['error'] = str(e)
        print('ERROR', e, flush=True)
    finally:
        result.update(final=mon.state, timeline=mon.timeline)
        json.dump(mon.feedback, open(out / 'feedback.json', 'w'))
        json.dump(mon.amcl, open(out / 'amcl.json', 'w'))
        json.dump({'map_odom': mon.map_odom, 'trust': mon.trust_reasons}, open(out / 'tf.json', 'w'))
        procs = [p for p in (launch, robot) if p]
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for p in procs:
            try:
                p.wait(20)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
        reports = sorted((out / 'output').glob('task_*.json'), key=os.path.getmtime)
        if reports:
            result['report'] = reports[-1].name
        json.dump(result, open(out / 'result.json', 'w'), indent=2)
        print(json.dumps({k: result.get(k) for k in ('error', 'mission_s')} |
                         {'state': mon.state.get('state'), 'reason': mon.state.get('reason')}), flush=True)
        # A hung spin thread must never leave a stale monitor calling services in the DDS domain.
        os._exit(0)


if __name__ == '__main__':
    main()
