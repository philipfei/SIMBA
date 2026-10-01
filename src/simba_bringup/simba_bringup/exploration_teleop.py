"""Phase-aware extension of terminal teleop; manual velocity stays on the PC."""
import json
import os
import select
import signal
import subprocess
import sys
import tempfile
import termios
import time
import tty
import rclpy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from .keyboard_teleop import KeyboardTeleop
from .exploration import key_action

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def display(*args, **kwargs):
    """Terminal disappearance must not interrupt lease release or cleanup."""
    try:
        print(*args, **kwargs)
    except OSError:
        pass


class ExplorationTeleop(KeyboardTeleop):
    def __init__(self, config_dir):
        super().__init__(config_dir)
        self.granted = False
        self.manual_requested = False
        self.state = {}
        self.received = 0.
        self.last_printed = None
        self.return_home = self.create_client(Trigger, '/exploration/return_home')
        self.cancel = self.create_client(Trigger, '/exploration/cancel')
        self.safety_reset = self.create_client(Trigger, '/safety/reset_all')
        self.create_subscription(String, '/exploration/state', self.receive_state, LATCHED)
        self.create_timer(0.1, self.lease_heartbeat)

    def lease_heartbeat(self):
        # ROS timers also run while waiting for a service/grant acknowledgement.
        if self.manual_requested and self.fresh() and self.state.get('phase') == 'MANUAL':
            self.heartbeat.publish(Empty())
            self.last_heartbeat = time.monotonic()

    def request_manual(self, enabled):
        self.manual_requested = enabled
        try:
            super().request_manual(enabled)
        except Exception:
            self.manual_requested = False
            raise

    def receive_state(self, message):
        try:
            state = json.loads(message.data)
            if self.state.get('session_id') == state['session_id'] and state['sequence'] <= self.state.get('sequence', -1):
                return
            self.state = state
            self.received = time.monotonic()
            if state['owner'] != 'MANUAL':
                self.granted = False
                self.command = None
            key = (state['phase'], state['subphase'], state['reason'])
            if key != self.last_printed:
                display(f'\r\n{key[0]} {key[1]}: {key[2]}', flush=True)
                self.last_printed = key
        except (ValueError, KeyError, TypeError):
            self.state = {}

    def fresh(self):
        try:
            stamp = self.state['stamp']
            age = self.get_clock().now().nanoseconds / 1e9 - stamp['sec'] - stamp['nanosec'] / 1e9
            return -0.1 <= age <= 2.0 and time.monotonic() - self.received <= 2.0
        except (KeyError, TypeError):
            return False

    def pump_until(self, condition, timeout=2.0):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            if condition():
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return bool(condition())

    def trigger(self, client, label='Coordinator command'):
        if not client.wait_for_service(timeout_sec=0.5):
            raise RuntimeError(label + ' service unavailable; use the physical robot stop if needed')
        future = client.call_async(Trigger.Request())
        if not self.pump_until(future.done):
            raise RuntimeError(label + ' acknowledgement timed out; physical stop may be required')
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(response.message if response else 'Command failed')
        display('\r\n' + response.message, flush=True)

    def release(self):
        self.command = None
        if self.granted or self.manual_requested:
            self.stop()
            self.request_manual(False)
            self.granted = False

    def handle_key(self, key):
        if not self.fresh():
            display('\r\nExploration state is stale; movement/return disabled.', flush=True)
            return
        action = key_action(self.state['phase'], key)
        if action == 'reset':
            self.reset_safety()
        elif action == 'reset_blocked':
            display('\r\nShift+R requires MANUAL or FAILED. Cancel with X and wait for '
                    'the robot to stop before resetting.', flush=True)
        elif action == 'return':
            self.release()
            if not self.pump_until(lambda: self.fresh() and (self.state.get('owner') == 'NONE' or self.state.get('phase') == 'DOCKED')):
                raise RuntimeError('Gate NONE not confirmed; return was not requested')
            self.trigger(self.return_home)
        elif action == 'cancel':
            self.release()
            self.trigger(self.cancel)
        elif action == 'stop':
            self.set_key('x')
        elif action == 'move':
            if not self.granted:
                self.request_manual(True)
                self.granted = True
                self.command = None
                if not self.pump_until(lambda: self.fresh() and self.state.get('owner') == 'MANUAL'):
                    self.request_manual(False)
                    self.granted = False
                    raise RuntimeError('Manual gate acknowledgement timed out; no movement sent')
                self.granted = True
            self.set_key(key.lower())
        elif action == 'ignore' and key.lower() in ('w', 'a', 's', 'd'):
            display('\r\nMovement requires MANUAL; current phase: ' + self.state['phase'] +
                    ' / ' + self.state.get('subphase', '') + ' / ' + self.state.get('reason', ''), flush=True)
        elif action == 'busy':
            display('\r\nReturn unavailable in this phase; H is not queued.', flush=True)

    def reset_safety(self):
        self.release()
        if not self.pump_until(lambda: self.fresh() and self.state.get('owner') == 'NONE'
                              and self.state.get('stopped') and self.state.get('action_idle')):
            raise RuntimeError('Reset refused: stopped robot, idle actions and gate NONE '
                               'were not confirmed. Use X to cancel and wait before Shift+R.')
        self.trigger(self.safety_reset, '/safety/reset_all')
        # Resetting a latch does not acknowledge the coordinator's FAILED phase.
        # Use its existing cancellation/acknowledgement path; never request motion.
        if self.state.get('phase') == 'FAILED':
            if not self.state.get('home_valid'):
                display('\r\nReset requested; HOME_INVALID remains. Recover to the dock '
                        'and relaunch Exploration.', flush=True)
                return
            self.trigger(self.cancel)
            if not self.pump_until(lambda: self.fresh()
                                  and self.state.get('phase') == 'MANUAL'
                                  and self.state.get('stopped') and self.state.get('action_idle')
                                  and self.state.get('owner') == 'NONE'):
                raise RuntimeError('Reset requested; MANUAL readiness not confirmed. '
                                   'Inspect the current state; no movement was requested.')
        display('\r\nReset requested; wait for clear safety state / enabled wheels. '
                'Press a fresh w/s/a/d key to drive or H to return; neither is automatic.',
                flush=True)

    def tick(self):
        if self.granted and self.fresh() and self.state.get('phase') == 'MANUAL':
            super().tick()
        elif self.granted:
            self.stop()
            self.granted = False
            self.command = None


def start_rviz():
    log = tempfile.NamedTemporaryFile(prefix='simba-exploration-rviz-', suffix='.log', delete=False)
    try:
        process = subprocess.Popen(['ros2', 'launch', 'simba_bringup', 'rviz.launch.py', 'mode:=exploration'],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    finally:
        log.close()
    display(f'RViz log: {log.name}', flush=True)
    return process


def close_rviz(process):
    if process is None or process.poll() is not None:
        return
    for sig, timeout in ((signal.SIGINT, 6), (signal.SIGTERM, 2), (signal.SIGKILL, 2)):
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue
        except ProcessLookupError:
            return


def run(parsed, ros_args):
    terminal = termios.tcgetattr(sys.stdin)
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = ExplorationTeleop(parsed.config_dir)
    rviz = None
    exit_kind = {'value': None}
    previous = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    def interrupted(sig, _frame):
        exit_kind['value'] = 'detach' if sig == signal.SIGHUP else 'quit'
    for sig in previous:
        signal.signal(sig, interrupted)
    display('w/s/a/d move; H return home; X/Space cancel/stop; Shift+R reset safety; '
          'Q stop and exit; Shift+D detach.\n'
          'Detach or PC loss lets an active return continue. Use physical stop when disconnected.', flush=True)
    try:
        if parsed.rviz:
            rviz = start_rviz()
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok() and exit_kind['value'] is None:
            readable, _, _ = select.select([sys.stdin], [], [], 0.05)
            if readable:
                key = sys.stdin.read(1)
                if key == '' or key == 'D':
                    exit_kind['value'] = 'detach'
                elif key.lower() == 'q':
                    exit_kind['value'] = 'quit'
                else:
                    try:
                        node.handle_key(key)
                    except RuntimeError as error:
                        display('\r\n' + str(error), flush=True)
            node.tick()
            rclpy.spin_once(node, timeout_sec=0.)
    except (EOFError, OSError):
        exit_kind['value'] = 'detach'
    finally:
        if rclpy.ok():
            try:
                node.release()
                if exit_kind['value'] != 'detach':
                    node.trigger(node.cancel)
                    if not node.pump_until(lambda: node.fresh() and node.state.get('stopped') and
                        node.state.get('action_idle') and node.state.get('phase') in ('MANUAL', 'FAILED', 'DOCKED')):
                        display('Stop not confirmed. Use the physical robot stop.', flush=True)
            except RuntimeError as error:
                display(str(error), flush=True)
        close_rviz(rviz)
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal)
        except termios.error:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
