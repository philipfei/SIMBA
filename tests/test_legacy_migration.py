"""First-install migration preserves legacy operation and never launches nodes."""
import importlib.util
import json
import os
from pathlib import Path
import stat
import sys
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.dont_write_bytecode = True


def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'modules/deployment' / filename)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


remote = load('migration_remote', 'remote.py')
migration = load('migration_helper', 'migrate_legacy.py')


@pytest.fixture
def old_workspace(tmp_path, monkeypatch):
    monkeypatch.setitem(sys.modules, 'remote', remote)
    d = remote.Deployment(tmp_path / 'ws')
    d.source.mkdir(parents=True)
    (d.source / 'scripts').mkdir()
    (d.source / 'scripts/teleop.py').write_text('#!/usr/bin/env python3\nprint("old")\n')
    (d.source / 'scripts/teleop.py').chmod(0o755)
    (d.source / 'package.xml').write_text('<package><name>create3_lidar_bringup</name></package>')
    (d.source / 'config').mkdir()
    (d.source / 'config/map.yaml').write_text('old: true\n')
    d.entry.mkdir()
    (d.entry / 'setup.bash').write_text('# Legacy setup\n')
    share = d.entry / 'create3_lidar_bringup/share/create3_lidar_bringup'
    share.mkdir(parents=True)
    (share / 'package.xml').symlink_to(d.source / 'package.xml')
    (share / 'config').mkdir()
    (share / 'config/map.yaml').symlink_to(d.source / 'config/map.yaml')
    exe = d.entry / 'create3_lidar_bringup/lib/create3_lidar_bringup'
    exe.mkdir(parents=True)
    (exe / 'teleop.py').symlink_to(d.source / 'scripts/teleop.py')
    (d.ws / 'build').mkdir()
    (d.ws / 'build/old').write_text('preserve build')
    (d.ws / 'maps').mkdir()
    (d.ws / 'maps/map.data').write_bytes(b'preserve map')
    d.source.chmod(0o555)
    expected = migration.legacy_payload(d.entry)
    def command(args):
        assert not any('ros2 launch' in item for item in args)
        if d.stage == 'build legacy':
            install = d.attempt / 'install'
            install.mkdir()
            (install / 'setup.bash').write_text('# Independent setup\n')
            for relative in expected:
                target = install / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                if relative.startswith('lib/'):
                    source = d.source / 'scripts' / target.name
                elif relative.endswith('package.xml'):
                    source = d.source / 'package.xml'
                else:
                    source = d.source / 'config/map.yaml'
                target.write_bytes(source.read_bytes())
                target.chmod(source.stat().st_mode)
    monkeypatch.setattr(d, 'command', command)
    yield d
    # Restore the deliberately read-only directory so pytest can clean its fixture.
    d.source.chmod(0o755)


def test_migration_preflight_is_readonly(old_workspace):
    d = old_workspace
    assert migration.legacy_preflight(d)['already_managed'] is False
    assert d.entry.is_dir() and not d.entry.is_symlink()
    assert not d.root.exists()
    assert stat.S_IMODE(d.source.stat().st_mode) == 0o555


def test_migration_builds_before_atomic_exchange_and_preserves_files(old_workspace):
    d = old_workspace
    original_source = remote.manifest(d.source)
    original_payload = migration.legacy_payload(d.entry)
    result = migration.migrate(d)
    assert result['status'] == 'success'
    assert d.entry.is_symlink()
    assert migration.legacy_payload(Path(result['original_install_backup'])) == original_payload
    assert remote.manifest(d.source) == original_source
    assert (d.entry / 'lib/create3_lidar_bringup/teleop.py').read_text().endswith('print("old")\n')
    assert not (d.entry / 'lib/create3_lidar_bringup/teleop.py').is_symlink()
    assert stat.S_IMODE(d.source.stat().st_mode) & stat.S_IWUSR
    assert (d.ws / 'maps/map.data').read_bytes() == b'preserve map'
    assert (d.ws / 'build/old').read_text() == 'preserve build'
    assert d.preflight()['prior_version'] is True
    assert migration.migrate(d)['already_managed'] is True


@pytest.mark.parametrize('failure', ['build', 'content', 'exchange'])
def test_failure_does_not_replace_legacy_install(old_workspace, monkeypatch, failure):
    d = old_workspace
    before = migration.legacy_payload(d.entry)
    command = d.command
    def failed(args):
        if failure == 'build':
            raise ValueError('Injected build failure')
        command(args)
        if failure == 'content' and d.stage == 'build legacy':
            (d.attempt / 'install/lib/create3_lidar_bringup/teleop.py').write_text('wrong version')
    monkeypatch.setattr(d, 'command', failed)
    if failure == 'exchange':
        monkeypatch.setattr(migration, 'exchange', lambda *a: (_ for _ in ()).throw(OSError('Injected exchange failure')))
    with pytest.raises((ValueError, OSError)):
        migration.migrate(d)
    assert not d.entry.is_symlink()
    assert migration.legacy_payload(d.entry) == before
    assert (d.ws / 'maps/map.data').read_bytes() == b'preserve map'
    assert stat.S_IMODE(d.source.stat().st_mode) == 0o555
