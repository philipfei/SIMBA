"""Hardware-independent exploration policy; ROS adapters supply observations/actions."""
from collections import deque
from dataclasses import dataclass
import math
from pathlib import Path
import uuid
import yaml
from simba_coverage.geometry import wrap


def load_profile(config_dir):
    profile = yaml.safe_load((Path(config_dir) / 'exploration_params.yaml').read_text())
    expected = {'approach_offsets_m', 'approach_margin_m', 'handoff_xy_m', 'handoff_yaw_rad',
                'map_wait_s', 'costmap_wait_s', 'tf_age_s', 'tf_lost_s', 'settle_s',
                'settle_timeout_s', 'jump_translation_m', 'jump_rotation_deg', 'jump_window_s',
                'max_jump_recoveries', 'max_navigation_retries', 'state_period_s', 'navigation'}
    if not isinstance(profile, dict) or set(profile) != expected:
        raise ValueError('Invalid exploration profile keys')
    navigation = profile['navigation']
    if not isinstance(navigation, dict) or set(navigation) != {'planner_tolerance_m', 'goal_xy_m', 'goal_yaw_rad'}:
        raise ValueError('Invalid exploration navigation keys')
    numbers = [v for k, v in profile.items() if k not in ('navigation', 'approach_offsets_m')]
    offsets = profile['approach_offsets_m']
    if not isinstance(offsets, list) or not offsets:
        raise ValueError('At least one approach offset is required')
    for value in [*numbers, *offsets, *navigation.values()]:
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
            raise ValueError('Exploration settings must be finite and positive')
    for key in ('max_jump_recoveries', 'max_navigation_retries'):
        if not isinstance(profile[key], int):
            raise ValueError(key + ' must be an integer')
    return profile


def approaches(docked_pose, offsets):
    x, y, heading = docked_pose
    return [(x - d * math.cos(heading), y - d * math.sin(heading), heading) for d in offsets]


def at_home(pose, home, profile):
    return bool(pose is not None and home is not None and
                math.dist(pose[:2], home[:2]) <= profile['handoff_xy_m'] + 1e-10 and
                abs(wrap(pose[2] - home[2])) <= profile['handoff_yaw_rad'] + 1e-10)


def key_action(phase, key):
    """Uppercase D detaches and R resets; lowercase d retains right turn."""
    if key == 'D':
        return 'detach'
    if key == 'R':
        return 'reset' if phase in ('MANUAL', 'FAILED') else 'reset_blocked'
    key = key.lower()
    if key == 'q':
        return 'quit'
    if key in ('x', ' '):
        return 'stop' if phase == 'MANUAL' else 'cancel'
    if key == 'h':
        return 'return' if phase in ('MANUAL', 'FAILED', 'DOCKED') else 'busy'
    if key in ('w', 'a', 's', 'd'):
        return 'move' if phase == 'MANUAL' else 'ignore'
    return 'ignore'


class SlamRecovery:
    """Separate TF loss from odometry-compensated map corrections."""
    def __init__(self, profile):
        self.cfg = profile
        self.previous = None
        self.stable_since = None
        self.lost_since = None
        self.episode_since = None
        self.episodes = deque()
        self.total = 0
        self.last_correction = (0., 0.)

    def observe(self, now, pose, odom):
        if pose is None or odom is None:
            self.previous = None
            self.stable_since = None
            if self.lost_since is None:
                self.lost_since = now
            return 'TF_STALE' if now - self.lost_since >= self.cfg['tf_lost_s'] else 'TF_WAIT'
        self.lost_since = None
        jump = False
        if self.previous:
            old_pose, old_odom = self.previous
            angle = wrap(old_pose[2] - old_odom[2])
            dx, dy = odom[0] - old_odom[0], odom[1] - old_odom[1]
            predicted = (old_pose[0] + math.cos(angle)*dx - math.sin(angle)*dy,
                         old_pose[1] + math.sin(angle)*dx + math.cos(angle)*dy)
            translation = math.dist(predicted, pose[:2])
            rotation = abs(wrap(pose[2] - old_pose[2] - wrap(odom[2] - old_odom[2])))
            self.last_correction = (translation, rotation)
            jump = translation > self.cfg['jump_translation_m'] or rotation > math.radians(self.cfg['jump_rotation_deg'])
        self.previous = (tuple(pose), tuple(odom))
        while self.episodes and now - self.episodes[0] > self.cfg['jump_window_s']:
            self.episodes.popleft()
        if jump:
            self.stable_since = now
            if self.episode_since is None:
                self.episode_since = now
                self.episodes.append(now)
                self.total += 1
            if len(self.episodes) > self.cfg['max_jump_recoveries']:
                return 'SLAM_UNSTABLE'
            if now - self.episode_since >= self.cfg['settle_timeout_s']:
                return 'SLAM_UNSTABLE'
            return 'SLAM_SETTLING'
        if self.stable_since is None:
            self.stable_since = now
        if self.episode_since is not None and now - self.episode_since >= self.cfg['settle_timeout_s']:
            return 'SLAM_UNSTABLE'
        if now - self.stable_since < self.cfg['settle_s']:
            return 'SLAM_SETTLING'
        self.episode_since = None
        return ''


@dataclass
class Observation:
    now: float
    stamp: float
    stopped: bool = False
    slot_idle: bool = True
    sensor_reason: str = ''
    gate_reason: str = ''
    gate_owner: str = 'NONE'
    gate_epoch: str = ''
    gate_healthy: bool = False
    docked: object = None  # None means missing/stale, never assume undocked.
    visible_samples: int = 0
    pose: object = None
    localization: str = 'TF_WAIT'
    map_seq: int = 0
    map_stamp: float = 0.
    map_identity: str = ''
    costmap_ready: bool = False
    candidate_valid: tuple = ()
    local_clear: bool = False


class Exploration:
    """One session, explicit owner handoffs, and bounded autonomous effects."""
    def __init__(self, profile, settings, now, auto_undock=True):
        self.cfg = profile
        self.settings = settings
        self.session_id = uuid.uuid4().hex
        self.phase = 'WAIT_READY'
        self.subphase = ''
        self.reason = ''
        self.since = now
        self.auto = auto_undock
        self.manual = False
        self.manual_heartbeat = now
        self.docked_pose = None
        self.exit_pose = None
        self.home = None
        self.home_valid = False
        self.candidates = []
        self.candidate_index = 0
        self.retry_count = 0
        self.jump_count = 0
        self.return_started = None
        self.effects = []
        self._owner = 'NONE'
        self.epoch_number = 0
        self.stop_stamp = 0.
        self.barrier_seq = 0
        self.map_seen_at = None
        self.baseline_identity = ''
        self.failed_candidates = set()
        self.pending = 'MANUAL'
        self.native_hold = False
        self.native_result = None
        self.last_jump_total = 0

    @property
    def owner(self):
        if self.native_hold or self.phase in ('UNDOCK', 'DOCK') or self.subphase in ('UNDOCK_ARM', 'DOCK_ARM'):
            owner = 'NATIVE'
        elif self.manual:
            owner = 'MANUAL'
        elif self.phase == 'FOLLOW' or self.subphase == 'NAV_ARM':
            owner = 'NAV'
        else:
            owner = 'NONE'
        if owner != self._owner:
            self._owner = owner
            self.epoch_number += 1
        return owner

    @property
    def epoch(self):
        self.owner
        return f'{self.session_id}:{self.epoch_number}'

    def change(self, phase, now, subphase='', reason=''):
        self.phase, self.subphase, self.reason, self.since = phase, subphase, reason, now
        self.owner

    def manual_request(self, enabled, obs):
        if not enabled:
            self.manual = False  # Must never affect an autonomous grant.
            self.owner
            return True, 'Manual lease released; wait for gate NONE before returning'
        if self.phase != 'MANUAL' or not obs.slot_idle or self.native_hold:
            return False, 'Manual ownership denied: ' + self.phase
        if obs.sensor_reason or obs.gate_reason or not obs.gate_healthy or not obs.stopped:
            return False, obs.sensor_reason or obs.gate_reason or 'ROBOT_NOT_READY_OR_STOPPED'
        if obs.docked is not False:
            return False, 'DOCK_STATUS_UNCONFIRMED' if obs.docked is None else 'ALREADY_DOCKED'
        self.manual = True
        self.manual_heartbeat = obs.now
        self.owner
        return True, 'Manual lease granted; fresh commands required'

    def request_return(self, obs):
        if self.phase == 'DOCKED':
            return True, 'Already docked; relaunch to undock again'
        if self.phase in ('HANDOVER', 'PLAN', 'FOLLOW', 'DOCK'):
            return True, 'Return already active; no duplicate task created'
        if self.phase not in ('MANUAL', 'FAILED'):
            return False, 'Return unavailable: ' + self.phase
        if self.manual or obs.gate_owner != 'NONE':
            return False, 'Release MANUAL and wait for gate NONE first'
        if not self.home_valid:
            return False, 'HOME_INVALID: relaunch from the dock'
        if obs.docked is True:
            self.change('DOCKED', obs.now)
            return True, 'Already docked'
        if obs.docked is None or obs.sensor_reason or obs.gate_reason or obs.localization:
            return False, obs.sensor_reason or obs.gate_reason or obs.localization or 'DOCK_STATUS_UNCONFIRMED'
        self.return_started = obs.now
        self.retry_count = self.jump_count = 0
        self.failed_candidates.clear()
        self.pending = 'RETURN'
        self.change('HANDOVER', obs.now, 'STOP')
        return True, 'Return accepted; completion is reported on /exploration/state'

    def cancel(self, obs, failure=''):
        if self.phase == 'DOCKED':
            return
        if self.subphase == 'CANCEL':
            return  # Preserve the first cause and original cancellation deadline.
        self.auto = False
        self.manual = False
        if self.phase == 'UNDOCK' or self.subphase == 'UNDOCK_ARM':
            self.home_valid = False
        self.native_hold = self.owner == 'NATIVE'
        self.effects.extend([('cancel', None), ('clear_path', None)])
        self.pending = 'FAILED' if failure else ('MANUAL' if self.home_valid else 'FAILED')
        self.change('HANDOVER', obs.now, 'CANCEL', failure or ('HOME_INVALID' if not self.home_valid else 'OPERATOR_CANCEL'))

    def fail(self, obs, reason):
        self.cancel(obs, reason)

    def wait_map(self, obs, recovery=False):
        self.stop_stamp, self.barrier_seq = obs.stamp, obs.map_seq
        self.map_seen_at = None
        self.change('HANDOVER', obs.now, 'MAP_WAIT', 'SLAM_RECOVERED' if recovery else '')

    def retry_navigation(self, obs, reason):
        if self.retry_count >= self.cfg['max_navigation_retries']:
            self.fail(obs, reason)
            return
        self.retry_count += 1
        self.baseline_identity = obs.map_identity
        self.pending = 'RETRY'
        self.effects.extend([('cancel', None), ('clear_path', None)])
        self.change('HANDOVER', obs.now, 'STOP', reason)

    def plan_next(self, obs):
        while self.candidate_index < len(self.candidates):
            i = self.candidate_index
            if i < len(obs.candidate_valid) and obs.candidate_valid[i]:
                if self.retry_count and obs.map_identity == self.baseline_identity and i in self.failed_candidates:
                    self.candidate_index += 1
                    continue
                self.home = self.candidates[i]
                self.change('PLAN', obs.now)
                self.effects.append(('plan', self.home))
                return
            self.candidate_index += 1
        self.retry_navigation(obs, 'APPROACH_UNREACHABLE' if not self.failed_candidates else 'NO_PATH')

    def action_result(self, name, success, result, obs):
        if self.subphase == 'CANCEL' or self.phase == 'FAILED':
            return
        if name == 'undock' and self.phase == 'UNDOCK':
            if not success or result is not False:
                self.fail(obs, 'UNDOCK_FAILED')
            else:
                self.native_result = False
                self.change('HANDOVER', obs.now, 'UNDOCK_CONFIRM')
        elif name == 'plan' and self.phase == 'PLAN':
            if not success:
                self.failed_candidates.add(self.candidate_index)
                self.candidate_index += 1
                self.plan_next(obs)
            else:
                self.effects.append(('show_path', result))
                self.path = result
                self.change('HANDOVER', obs.now, 'NAV_ARM')
        elif name == 'follow' and self.phase == 'FOLLOW':
            if not success:
                self.failed_candidates.add(self.candidate_index)
                self.retry_navigation(obs, 'FOLLOW_FAILED')
            else:
                self.change('HANDOVER', obs.now, 'DOCK_READY')
        elif name == 'dock' and self.phase == 'DOCK':
            self.native_result = result
            self.change('HANDOVER', obs.now, 'DOCK_CONFIRM', '' if success and result else 'DOCK_FAILED')

    def tick(self, obs, jump_total=0):
        deadlines = self.settings['deadlines']
        if self.manual and obs.now - self.manual_heartbeat > self.settings['health']['lease_age_s']:
            self.manual = False
            self.change('HANDOVER', obs.now, 'STOP', 'MANUAL_HEARTBEAT_LOST')
            self.pending = 'MANUAL'
        if self.phase in ('DOCKED', 'FAILED'):
            return
        if self.subphase == 'CANCEL':
            if obs.slot_idle and obs.stopped:
                self.native_hold = False
                phase = self.pending
                self.return_started = None
                self.change(phase, obs.now, reason=self.reason if phase == 'FAILED' else '')
            elif obs.now - self.since >= deadlines['action_cancel_s']:
                self.effects.append(('estop', 'CANCEL_STOP_UNCONFIRMED'))
                self.reason = 'CANCEL_STOP_UNCONFIRMED'
                self.pending = 'FAILED'
            return
        active = self.phase in ('UNDOCK', 'PLAN', 'FOLLOW', 'DOCK') or self.phase == 'HANDOVER'
        if active and (obs.sensor_reason or obs.gate_reason):
            # Successful docking can disable wheels; consume fresh dock confirmation first.
            if obs.docked is True and self.subphase == 'DOCK_CONFIRM' and self.native_result:
                self.native_hold = False
                self.change('DOCKED', obs.now)
            else:
                self.fail(obs, obs.sensor_reason or obs.gate_reason)
            return
        if self.return_started is not None and obs.now - self.return_started >= deadlines['return_s']:
            self.fail(obs, 'RETURN_TIMEOUT')
            return
        navigating = self.return_started is not None and self.phase not in ('DOCK',) and self.subphase not in ('DOCK_ARM', 'DOCK_CONFIRM')
        if navigating and obs.localization:
            if obs.localization in ('TF_STALE', 'SLAM_UNSTABLE'):
                self.fail(obs, obs.localization)
                return
            if self.subphase not in ('TF_WAIT', 'SLAM_SETTLING'):
                self.effects.extend([('cancel', None), ('clear_path', None)])
                self.change('HANDOVER', obs.now, obs.localization)
            if jump_total > self.last_jump_total:
                self.jump_count += jump_total - self.last_jump_total
                self.last_jump_total = jump_total
            if self.jump_count > self.cfg['max_jump_recoveries']:
                self.fail(obs, 'SLAM_UNSTABLE')
            return
        if self.subphase in ('TF_WAIT', 'SLAM_SETTLING'):
            if obs.slot_idle and obs.stopped:
                self.wait_map(obs, recovery=True)
            return
        if self.phase == 'WAIT_READY':
            if not self.auto:
                return
            blocker = (obs.sensor_reason or obs.gate_reason or obs.localization or
                       ('DOCK_STATUS_UNCONFIRMED' if obs.docked is None else 'NOT_DOCKED' if not obs.docked else ''))
            if blocker:
                self.reason = blocker
                if obs.docked is False or obs.now - self.since >= deadlines['prepare_s']:
                    self.fail(obs, blocker)
            elif obs.gate_healthy and obs.stopped and obs.pose is not None and obs.map_seq:
                self.docked_pose = tuple(obs.pose)
                self.candidates = approaches(self.docked_pose, self.cfg['approach_offsets_m'])
                self.home = self.candidates[0]
                self.change('HANDOVER', obs.now, 'UNDOCK_ARM')
            return
        if self.subphase in ('UNDOCK_ARM', 'DOCK_ARM', 'NAV_ARM'):
            if obs.now - self.since >= deadlines['native_ownership_s']:
                self.fail(obs, 'OWNERSHIP_TIMEOUT')
            elif obs.gate_owner == self.owner and obs.gate_epoch == self.epoch and obs.gate_healthy and obs.stopped and obs.slot_idle:
                if self.subphase == 'UNDOCK_ARM':
                    if obs.docked is not True:
                        self.fail(obs, 'DOCK_STATUS_UNCONFIRMED' if obs.docked is None else 'NOT_DOCKED')
                        return
                    self.change('UNDOCK', obs.now)
                    self.effects.append(('undock', None))
                elif self.subphase == 'DOCK_ARM':
                    if obs.docked is True:
                        self.change('DOCKED', obs.now)
                    elif obs.docked is None or obs.visible_samples < 2 or not obs.local_clear:
                        self.fail(obs, 'DOCK_NOT_VISIBLE_OR_CLEAR')
                    else:
                        self.change('DOCK', obs.now)
                        self.effects.append(('dock', None))
                else:
                    self.change('FOLLOW', obs.now)
                    self.effects.append(('follow', self.path))
            return
        if self.phase in ('UNDOCK', 'DOCK'):
            limit = deadlines['undock_s'] if self.phase == 'UNDOCK' else deadlines['dock_s']
            if obs.now - self.since >= limit:
                self.fail(obs, 'NATIVE_ACTION_TIMEOUT')
            return
        if self.subphase == 'UNDOCK_CONFIRM':
            if obs.docked is False and obs.stopped and obs.pose is not None and not obs.localization:
                self.exit_pose = tuple(obs.pose)
                self.home_valid = True
                self.change('MANUAL', obs.now)
            elif obs.now - self.since >= deadlines['prepare_s']:
                self.fail(obs, 'UNDOCK_STATUS_DISAGREEMENT')
            return
        if self.subphase == 'DOCK_CONFIRM':
            if self.native_result and obs.docked is True:
                self.change('DOCKED', obs.now)
                self.effects.append(('clear_path', None))
            elif obs.now - self.since >= deadlines['dock_confirmation_s']:
                self.fail(obs, self.reason or 'DOCK_STATUS_DISAGREEMENT')
            return
        if self.subphase == 'STOP':
            if obs.slot_idle and obs.stopped and obs.gate_owner == 'NONE':
                if self.pending == 'MANUAL':
                    self.change('MANUAL', obs.now)
                elif self.pending != 'RETRY' and at_home(obs.pose, self.home, self.cfg) and obs.visible_samples >= 2 and obs.local_clear:
                    self.change('HANDOVER', obs.now, 'DOCK_ARM')
                else:
                    self.wait_map(obs)
            elif obs.now - self.since >= deadlines['action_cancel_s']:
                self.effects.append(('estop', 'CANCEL_STOP_UNCONFIRMED'))
                self.fail(obs, 'CANCEL_STOP_UNCONFIRMED')
            return
        if self.subphase == 'MAP_WAIT':
            fresh = obs.map_seq > self.barrier_seq and obs.map_stamp > self.stop_stamp
            if fresh and self.map_seen_at is None:
                self.map_seen_at = obs.now
            if self.map_seen_at is None:
                if obs.now - self.since >= self.cfg['map_wait_s']:
                    self.fail(obs, 'MAP_STALE')
            elif obs.costmap_ready:
                self.candidate_index = 0
                self.plan_next(obs)
            elif obs.now - self.map_seen_at >= self.cfg['costmap_wait_s']:
                self.fail(obs, 'COSTMAP_STALE')
            return
        if self.phase == 'PLAN' and obs.now - self.since >= deadlines['planning_s']:
            self.failed_candidates.add(self.candidate_index)
            self.retry_navigation(obs, 'PLAN_TIMEOUT')
            return
        if self.subphase == 'DOCK_READY':
            if not obs.stopped:
                if obs.now - self.since >= deadlines['action_cancel_s']:
                    self.fail(obs, 'STOP_NOT_CONFIRMED')
                return
            if not at_home(obs.pose, self.home, self.cfg):
                self.retry_navigation(obs, 'HANDOFF_POSE_ERROR')
            elif not obs.local_clear:
                self.fail(obs, 'APPROACH_BLOCKED')
            elif obs.docked is True:
                self.change('DOCKED', obs.now)
            elif obs.visible_samples >= 2:
                self.change('HANDOVER', obs.now, 'DOCK_ARM')
            elif obs.now - self.since >= deadlines['dock_visibility_s']:
                self.fail(obs, 'DOCK_NOT_VISIBLE')
