"""Behavioral tests for offline provenance and deployment failure isolation."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import types
import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/create3_lidar_bringup'))
from create3_coverage import offline
from create3_coverage.geometry import Grid
from create3_coverage.settings import load_settings, canonical_hash, offline_case


def module(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


remote = module('simba_remote', 'modules/deployment/remote.py')
deps = module('simba_deps', 'tools/check_dependencies.py')
sync = module('simba_sync', 'modules/deployment/sync.py')


@pytest.fixture
def registered(tmp_path, monkeypatch):
    grid = Grid.from_cells(np.zeros((32, 32), dtype=np.int8), .05, (0., 0., 0.))
    registry = tmp_path / 'registry.yaml'
    monkeypatch.setattr(offline, 'INDEX', tmp_path / 'index.json')
    monkeypatch.setattr(offline, 'AUDIT', tmp_path / 'audit.json')
    settings = load_settings()
    offline.register(grid, [.6, .6], settings, registry=registry)
    return grid, settings, registry


def test_register_requires_safe_explicit_start(registered):
    grid, settings, registry = registered
    before = registry.read_bytes()
    with pytest.raises(ValueError):
        offline.register(grid, [.01, .01], settings, registry=registry)
    assert registry.read_bytes() == before


def test_changed_map_requires_own_record(registered):
    grid, _, registry = registered
    other = Grid.from_cells(grid.cells, grid.resolution, (1., 0., 0.))
    with pytest.raises(ValueError, match='No versioned'):
        offline_case(other, registry)


def test_registration_same_is_noop_and_update_requires_confirmation(registered):
    grid, settings, registry = registered
    before = registry.read_bytes()
    assert offline.register(grid, [.6, .6], settings, registry=registry)['status'] == 'unchanged'
    with pytest.raises(ValueError, match='--update'):
        offline.register(grid, [.8, .8], settings, registry=registry)
    with pytest.raises(ValueError, match='cancelled'):
        offline.register(grid, [.8, .8], settings, update=True, confirm=lambda: 'no', registry=registry)
    assert registry.read_bytes() == before


def test_noninteractive_update_refused(registered, monkeypatch):
    grid, settings, registry = registered
    monkeypatch.setattr(sys, 'stdin', types.SimpleNamespace(isatty=lambda: False))
    before = registry.read_bytes()
    with pytest.raises(ValueError, match='interactive'):
        offline.register(grid, [.8, .8], settings, update=True, registry=registry)
    assert registry.read_bytes() == before


def test_update_audit_and_staleness(registered, tmp_path):
    grid, settings, registry = registered
    _, old = offline_case(grid, registry)
    summary = {'map_id': grid.identity, 'registration_hash': old, 'config_hash': settings.hash}
    baseline = tmp_path / 'baseline.json'
    offline.atomic(baseline, {'summary': summary, 'registration_hash': old, 'config_hash': settings.hash})
    report = tmp_path / 'summary.json'
    offline.atomic(report, summary)
    offline.atomic(offline.INDEX, [dict(summary, output=str(tmp_path / 'preview'), status='current')])
    offline.verify_baseline(baseline, grid, old, settings)
    offline.check_result(report, grid, settings, registry)
    offline.register(grid, [.8, .8], settings, update=True, confirm=lambda: 'UPDATE', registry=registry)
    _, new = offline_case(grid, registry)
    assert old != new
    assert json.loads(offline.INDEX.read_text())[0]['status'] == 'superseded'
    event = json.loads(offline.AUDIT.read_text())[-1]
    assert event['previous_start'] == [.6, .6] and event['new_start'] == [.8, .8]
    assert event['previous_hash'] == old and event['new_hash'] == new
    with pytest.raises(ValueError, match='stale'):
        offline.verify_baseline(baseline, grid, new, settings)
    with pytest.raises(ValueError, match='stale'):
        offline.check_result(report, grid, settings, registry)


def test_historical_baseline_not_repaired(registered, tmp_path):
    grid, settings, registry = registered
    _, identity = offline_case(grid, registry)
    path = tmp_path / 'legacy.json'
    path.write_text(json.dumps({'summary': {'map_id': grid.identity, 'start_map_xy': [.6, .6]}}))
    before = path.read_bytes()
    with pytest.raises(ValueError, match='historical_unverifiable: missing_registration_hash'):
        offline.verify_baseline(path, grid, identity, settings)
    assert path.read_bytes() == before


def test_single_preview_and_current_comparison(registered, tmp_path):
    grid, settings, registry = registered
    from PIL import Image
    image = np.full((32, 32), 254, dtype=np.uint8)
    Image.fromarray(image).save(tmp_path / 'map.pgm')
    (tmp_path / 'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    output = tmp_path / 'single'
    summary = offline.preview(tmp_path / 'map.yaml', output, registry=registry)
    assert (output / 'preview.png').is_file() and (output / 'preview.svg').is_file()
    assert summary['actual_coverage_ratio'] is None
    masks = np.load(output / 'masks.npz')
    assert np.array_equal(masks['covered'] | masks['remaining'], masks['denominator'])
    assert int(masks['remaining'].sum()) == summary['remaining_cells']
    with pytest.raises(ValueError, match='Output already exists'):
        offline.preview(tmp_path / 'map.yaml', output, registry=registry)
    comparison = offline.preview(tmp_path / 'map.yaml', tmp_path / 'comparison', baseline=output / 'route.json', registry=registry)
    assert len(comparison['panels']) == 4
    assert comparison['registration_hash'] == summary['registration_hash']
    assert (tmp_path / 'comparison/comparison.png').is_file()


@pytest.mark.parametrize('expected,actual,status', [('1','1','match'),('1','2','mismatch'),('1',None,'missing'),(None,'1','unverified')])
def test_version_classification(expected, actual, status):
    assert deps.compare(expected, actual) == status


def test_effective_overlay_mismatch_is_visible(monkeypatch):
    row = dict(package='python3-numpy', python_module='numpy', ros_package=None,
               version_sources={'pc_installed': {'version': '1.26.4-1', 'architecture': 'amd64'},
                                'pc_extracted_tested': {'version': '1.25.0-1', 'architecture': 'amd64'},
                                'pi_historical': {'version': '1.24.0-1', 'architecture': 'arm64'}})
    monkeypatch.setattr(deps, 'apt_version', lambda _: ('1.26.4-1', 'amd64'))
    monkeypatch.setattr(deps, 'loaded_module', lambda _: {'actual_version': '2.0.0', 'loaded_from': '/different/overlay/numpy'})
    result = deps.check(row, 'pc', 'amd64')
    assert result['status'] == 'match'
    assert result['effective_environment']['status'] == 'mismatch'
    assert len(result['recorded_source_differences']) == 3
    assert result['effective_environment']['loaded_from'] == '/different/overlay/numpy'


def test_live_apt_mismatch_not_hidden(monkeypatch):
    row = dict(package='example', version_sources={'pc_installed': {'version':'1','architecture':'amd64'}})
    monkeypatch.setattr(deps, 'apt_version', lambda _: ('2','amd64'))
    assert deps.check(row, 'pc', 'amd64')['status'] == 'mismatch'


def candidate(d, name, status='success', activated=True):
    path = d.candidates / name
    (path / 'install').mkdir(parents=True)
    (path / 'install/setup.bash').write_text('export TEST_RELEASE=' + name + '\n')
    (path / 'install/version').write_text(name)
    (path / 'state.json').write_text(json.dumps({'created_at': name, 'status': status, 'ever_activated': activated}))
    return path


@pytest.fixture
def deployed(tmp_path):
    d = remote.Deployment(tmp_path / 'workspace')
    d.ws.mkdir()
    d.source.mkdir(parents=True)
    (d.source / 'source.txt').write_text('old source')
    old = candidate(d, '000-old')
    d.entry.symlink_to(old / 'install')
    return d, old


def test_unmanaged_install_refused_before_source_change(tmp_path):
    d = remote.Deployment(tmp_path)
    d.entry.mkdir()
    with pytest.raises(ValueError, match='Unmanaged'):
        d.preflight()
    assert not d.source.exists()


def test_source_symlink_install_refused(deployed):
    d, old = deployed
    (old / 'install/code').symlink_to(d.source / 'source.txt')
    with pytest.raises(ValueError, match='mutable/external'):
        d.preflight()


def test_lock_excludes_concurrent_attempt(deployed):
    d, _ = deployed
    other = remote.Deployment(d.ws)
    with d.locked():
        with pytest.raises(BlockingIOError):
            with other.locked():
                pass


def fake_commands(d, failing=None):
    def run(args):
        if d.stage == failing:
            raise subprocess.CalledProcessError(7, args)
        if d.stage == 'build':
            install = d.attempt / 'install'
            (install / 'share/create3_lidar_bringup').mkdir(parents=True)
            (install / 'setup.bash').write_text('export TEST_RELEASE=new\n')
            (install / 'version').write_text('new')
            (install / 'share/create3_lidar_bringup/source_manifest.json').write_text(json.dumps(remote.manifest(d.source)))
        if d.stage == 'test':
            (d.attempt / 'build').mkdir(exist_ok=True)
            (d.attempt / 'build/test.xunit.xml').write_text('<testsuites><testsuite tests="1" errors="0" failures="0"/></testsuites>')
    return run


@pytest.mark.parametrize('stage', ['sync', 'hash audit', 'build', 'test', 'install check', 'activate'])
def test_stage_failure_keeps_previous_install_and_no_source_rollback(deployed, monkeypatch, stage):
    d, old = deployed
    d.preflight()
    d.begin()
    (d.source / 'source.txt').write_text('new source')
    expected = remote.manifest(d.source)
    monkeypatch.setattr(d, 'command', fake_commands(d, stage))
    if stage == 'install check':
        monkeypatch.setattr(d, 'installed_check', lambda _: (_ for _ in ()).throw(ValueError('install failed')))
    if stage == 'activate':
        monkeypatch.setattr(d, 'activate', lambda: (_ for _ in ()).throw(ValueError('activation failed')))
    try:
        if stage == 'sync':
            d.stage = 'sync'; d.state['source_sync'] = 'may_be_partial'
            raise ValueError('rsync failed')
        if stage == 'hash audit':
            expected = {'different': 'hash'}
        d.finish(expected)
        pytest.fail('Expected deployment failure')
    except (ValueError, subprocess.CalledProcessError) as error:
        result = d.failure(error)
    assert result['stage'] == stage
    assert d.entry.resolve() == old / 'install'
    assert (d.entry / 'version').read_text() == '000-old'
    assert (d.source / 'source.txt').read_text() == 'new source'
    assert result['status'] == 'failed'
    assert result['log_directory']


def test_success_atomic_switch_and_old_process_environment(deployed, monkeypatch):
    d, old = deployed
    d.preflight()
    d.begin()
    old_process = subprocess.Popen(['bash', '--noprofile', '--norc', '-c', f'source "{old}/install/setup.bash"; echo READY; read -r proceed; echo "$TEST_RELEASE"; cat "{old}/install/version"'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    assert old_process.stdout.readline().strip() == 'READY'
    monkeypatch.setattr(d, 'command', fake_commands(d))
    replace = remote.os.replace
    observed = []
    def checked_replace(source, target):
        if Path(target) == d.entry:
            observed.append(d.entry.resolve())
            assert Path(source).parent == d.entry.parent
            assert d.entry.is_symlink()
        return replace(source, target)
    monkeypatch.setattr(remote.os, 'replace', checked_replace)
    d.finish(remote.manifest(d.source))
    assert observed == [old / 'install']
    assert d.entry.resolve() == d.attempt / 'install'
    old_output, _ = old_process.communicate('continue\n', timeout=5)
    assert old_output.splitlines() == ['000-old', '000-old']
    new_environment = subprocess.check_output(['bash', '-c', f'source "{d.entry.resolve()}/setup.bash"; echo "$TEST_RELEASE"'], text=True).strip()
    assert new_environment == 'new'
    assert (old / 'install/version').read_text() == '000-old'


def test_cleanup_only_old_failed_candidates(deployed):
    d, old = deployed
    previous = candidate(d, '001-success')
    unknown = candidate(d, '002-unknown', 'unknown', False)
    running = candidate(d, '003-running', 'running', False)
    failed = [candidate(d, '100-failed-' + str(i), 'failed', False) for i in range(6)]
    outside = d.ws / 'outside'
    outside.mkdir()
    (d.candidates / 'outside-link').symlink_to(outside, target_is_directory=True)
    d.cleanup()
    assert all(x.exists() for x in (old, previous, unknown, running, outside))
    assert not any(x.exists() for x in failed[:3])
    assert all(x.exists() for x in failed[3:])
    assert len(list((d.root / 'history').glob('*.json'))) == 3


def test_referenced_failed_candidate_is_protected(deployed):
    d, old = deployed
    failed = [candidate(d, '100-failed-' + str(i), 'failed', False) for i in range(5)]
    (old / 'retained_reference').symlink_to(failed[0] / 'install')
    d.cleanup()
    assert failed[0].exists()


@pytest.mark.parametrize('apply', [False, True])
def test_driver_uses_fake_ssh_rsync_and_readonly_default(monkeypatch, apply):
    import io
    calls = []
    class FakeProcess:
        def __init__(self, args, **kwargs):
            calls.append(args)
            self.stdout = io.StringIO('{"ready": true}\n' if apply else '{"stage":"preflight"}\n')
            self.stdin = io.StringIO()
            self.returncode = 0
        def wait(self):
            return 0
    def run(args, **kwargs):
        calls.append(args)
        return types.SimpleNamespace(returncode=0)
    monkeypatch.setattr(sync.subprocess, 'Popen', FakeProcess)
    monkeypatch.setattr(sync.subprocess, 'run', run)
    monkeypatch.setattr(sys, 'argv', ['sync.py'] + (['--apply'] if apply else []))
    assert sync.main() == 0
    ssh, rsync = calls
    assert ssh[0] == 'ssh' and 'StrictHostKeyChecking=yes' in ssh
    assert rsync[0] == 'rsync'
    assert ('--dry-run' in rsync) == (not apply)
    assert rsync[-1].endswith('/src/create3_lidar_bringup/')
    assert '/maps/' not in rsync[-1]


def test_shell_entrypoints_parse_and_help_without_network():
    for path in (ROOT / 'modules').rglob('*.sh'):
        subprocess.run(['bash', '-n', str(path)], check=True)
        if path.name != 'env.sh':
            result = subprocess.run([str(path), '--help'], capture_output=True, text=True)
            assert result.returncode == 0, (str(path), result.stderr)


def test_no_original_workspace_runtime_reference():
    paths = list((ROOT / 'modules').rglob('*')) + list((ROOT / 'tools').rglob('*'))
    for path in paths:
        if path.is_file() and path.suffix in ('.py', '.sh', '.yaml'):
            assert '/home/philip/Documents/ROS_0920' not in path.read_text()


def test_retained_failed_candidate_reference_is_protected(deployed):
    d, _ = deployed
    failed = [candidate(d, '200-failed-' + str(i), 'failed', False) for i in range(5)]
    (failed[-1] / 'debug_reference').symlink_to(failed[0] / 'install')
    d.cleanup()
    assert failed[0].exists()


def test_dependency_module_lookup_does_not_import_target(monkeypatch):
    import importlib.util
    monkeypatch.setattr(importlib.util, 'find_spec', lambda name: types.SimpleNamespace(origin='/tmp/example_pkg/__init__.py', submodule_search_locations=['/tmp/example_pkg']))
    monkeypatch.setattr(deps.importlib.metadata, 'distributions', lambda **kwargs: [types.SimpleNamespace(metadata={'Name': 'example-pkg'}, version='2.0')])
    def forbidden(*args, **kwargs):
        raise AssertionError('Read-only checker must not import inspected packages')
    monkeypatch.setattr(deps.importlib, 'import_module', forbidden)
    assert deps.loaded_module('example_pkg')['actual_version'] == '2.0'
