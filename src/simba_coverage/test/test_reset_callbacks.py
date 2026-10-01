"""Exercise reset callbacks with fake services, without a ROS installation."""
import ast
from pathlib import Path
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from simba_bringup.exploration import key_action


def callbacks(path, class_name, names, **namespace):
    # Compile the actual callback bodies; ROS Node construction is not needed.
    tree = ast.parse(path.read_text())
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
    node.bases = []
    node.body = [n for n in node.body if isinstance(n, ast.FunctionDef) and n.name in names]
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace[class_name]


SRC = Path(__file__).resolve().parents[2]


@pytest.fixture
def ui():
    cls = callbacks(SRC / 'simba_bringup/simba_bringup/exploration_teleop.py',
                    'ExplorationTeleop', ('handle_key', 'reset_safety'),
                    key_action=key_action, display=Mock())
    node = cls()
    node.state = dict(phase='FAILED', owner='NONE', stopped=True, action_idle=True, home_valid=True)
    node.fresh = lambda: True
    node.release = Mock()
    node.safety_reset, node.cancel, node.return_home = object(), object(), object()
    node.pump_until = lambda condition: bool(condition())

    def trigger(client, *args):
        if client is node.cancel:
            node.state['phase'] = 'MANUAL'

    node.trigger = Mock(side_effect=trigger)
    return node


def test_shift_r_resets_and_acknowledges_failure_without_requesting_motion(ui):
    ui.handle_key('R')
    ui.release.assert_called_once()
    assert [call.args[0] for call in ui.trigger.call_args_list] == [ui.safety_reset, ui.cancel]
    assert ui.state['phase'] == 'MANUAL'


def test_reset_rejection_does_not_acknowledge_failure(ui):
    ui.trigger.side_effect = RuntimeError('Hazard remains')
    with pytest.raises(RuntimeError, match='Hazard remains'):
        ui.handle_key('R')
    assert ui.trigger.call_count == 1 and ui.state['phase'] == 'FAILED'


@pytest.mark.parametrize('phase,key', [('FAILED', 'r'), ('FOLLOW', 'R'), ('DOCK', 'R')])
def test_other_keys_or_active_actions_do_not_reset(ui, phase, key):
    ui.state['phase'] = phase
    ui.handle_key(key)
    ui.trigger.assert_not_called()


def test_invalid_home_can_reset_wheels_but_does_not_enable_return(ui):
    ui.state['home_valid'] = False
    ui.handle_key('R')
    assert ui.trigger.call_count == 1 and ui.state['phase'] == 'FAILED'


@pytest.fixture
def gate():
    cls = callbacks(SRC / 'simba_coverage/simba_coverage/gate_node.py', 'SafetyGate',
                    ('reset', 'reset_all'), time=time, EStop=SimpleNamespace(Request=SimpleNamespace))
    node = cls()
    node.reset_pending = node.owns_estop = False
    node.inputs = SimpleNamespace(reason=lambda: '', hazards=lambda: set(), stopped=lambda: True,
                                  messages={'wheel_status': SimpleNamespace(wheels_enabled=False)})
    node.graph_ok = True
    node.policy = SimpleNamespace(owner='NONE', fault='WHEELS_DISABLED')
    node.settings = {'health': {'reset_grace_s': 1.0}}
    node.estop = Mock()
    node.estop.service_is_ready.return_value = True
    node.get_logger = lambda: Mock()
    return node


def test_reset_all_restores_externally_disabled_wheels_and_clears_latch(gate):
    response = gate.reset_all(None, SimpleNamespace())
    assert response.success and gate.reset_pending
    request = gate.estop.call_async.call_args.args[0]
    assert request.e_stop_on is False
    callback = gate.estop.call_async.return_value.add_done_callback.call_args.args[0]
    callback(SimpleNamespace(result=lambda: SimpleNamespace(success=True)))
    assert gate.policy.fault == '' and not gate.reset_pending and not gate.owns_estop


def test_native_release_failure_keeps_latch(gate):
    gate.reset_all(None, SimpleNamespace())
    callback = gate.estop.call_async.return_value.add_done_callback.call_args.args[0]
    callback(SimpleNamespace(result=lambda: SimpleNamespace(success=False)))
    assert gate.policy.fault == 'WHEELS_DISABLED' and not gate.reset_pending


def test_normal_reset_still_requires_external_wheel_restoration(gate):
    response = gate.reset(None, SimpleNamespace())
    assert not response.success
    gate.estop.call_async.assert_not_called()
