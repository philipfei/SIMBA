#!/usr/bin/env python3
"""Explicit first-time migration of a stopped, legacy Pi workspace; never launches nodes."""
import argparse
import ast
import base64
import ctypes
import errno
import json
import os
from pathlib import Path
import shlex
import shutil
import stat
import subprocess
import sys
sys.dont_write_bytecode = True


def exchange(left, right):
    """Atomically exchange a directory and a link on Linux, with no missing-entry window."""
    libc = ctypes.CDLL(None, use_errno=True)
    call = libc.renameat2
    call.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    call.restype = ctypes.c_int
    if call(-100, os.fsencode(left), -100, os.fsencode(right), 2):
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))


def legacy_preflight(deployment):
    d = deployment
    for path in (d.ws, d.ws / 'src', d.source, d.root, d.candidates):
        d.safe_path(path)
    if not d.source.is_dir():
        raise ValueError('Legacy source package is missing')
    if d.entry.is_symlink():
        return dict(d.preflight(), already_managed=True)
    if not d.entry.is_dir() or not (d.entry / 'setup.bash').is_file():
        raise ValueError('Expected an existing legacy install directory with setup.bash')
    from remote import manifest
    source_hashes = manifest(d.source)
    if not source_hashes:
        raise ValueError('Empty legacy source')
    for path in [d.source, *d.source.rglob('*')]:
        if path.is_symlink():
            raise ValueError('Legacy source symlink rejected: ' + str(path))
        if path.stat().st_uid != os.getuid():
            raise ValueError('Source is not owned by the deployment user: ' + str(path))
    return {'already_managed': False, 'source_files': len(source_hashes), 'active_install': str(d.entry),
            'source_directory_writable': os.access(d.source, os.W_OK),
            'action': 'Snapshot current Pi source, build independent legacy release, check payload, exchange install entry, enable owner write on mirror directories. Maps are untouched.'}


def legacy_payload(install):
    """Capture deployable resources from the old package, resolving existing symlinks."""
    import hashlib
    package = 'create3_lidar_bringup'
    share = install / package / 'share' / package
    executables = install / package / 'lib' / package
    if not share.is_dir():
        share = install / 'share' / package
        executables = install / 'lib' / package
    result = {}
    for directory in ('config', 'launch', 'rviz'):
        for path in (share / directory).rglob('*'):
            if path.is_file() and path.suffix != '.pyc':
                result['share/' + package + '/' + str(path.relative_to(share))] = hashlib.sha256(path.read_bytes()).hexdigest()
    for path in executables.glob('*'):
        if path.is_file() and path.suffix != '.pyc':
            result['lib/' + package + '/' + path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
    result['share/' + package + '/package.xml'] = hashlib.sha256((share / 'package.xml').read_bytes()).hexdigest()
    if not any(key.startswith('lib/') for key in result):
        raise ValueError('No legacy executables were found')
    return result


def migrate(d):
    import hashlib
    from remote import manifest
    result = legacy_preflight(d)
    if result['already_managed']:
        return result
    with d.locked():
        legacy_preflight(d)
        d.begin()
        try:
            d.state['kind'] = 'legacy_migration'
            d.stage = 'snapshot legacy'; d.record()
            print(json.dumps({'stage': d.stage, 'candidate': str(d.attempt)}), flush=True)
            source_hashes = manifest(d.source)
            original_payload = legacy_payload(d.entry)
            snapshot = d.attempt / 'source' / 'create3_lidar_bringup'
            shutil.copytree(d.source, snapshot, ignore=shutil.ignore_patterns('__pycache__', '*.pyc', '.pytest_cache', '.git'))
            # The snapshot is private to this candidate and never used as the sync target.
            for directory in [snapshot, *[x for x in snapshot.rglob('*') if x.is_dir()]]:
                directory.chmod(directory.stat().st_mode | stat.S_IWUSR)
            if manifest(snapshot) != source_hashes:
                raise ValueError('Legacy source snapshot hash mismatch')
            q = shlex.quote
            base = f'source /opt/ros/jazzy/setup.bash && colcon --log-base {q(str(d.attempt / "logs/colcon"))}'
            args = f' --base-paths {q(str(snapshot))} --packages-select create3_lidar_bringup --build-base {q(str(d.attempt / "build"))} --install-base {q(str(d.attempt / "install"))} --merge-install'
            d.stage = 'build legacy'; d.record()
            print(json.dumps({'stage': d.stage, 'log': str(d.attempt / 'logs/build_legacy.log'), 'note': 'Building only; this may take several minutes.'}), flush=True)
            d.command(['bash', '--noprofile', '--norc', '-c', base + ' build' + args])
            d.stage = 'validate legacy'; d.record()
            print(json.dumps({'stage': d.stage}), flush=True)
            candidate = d.attempt / 'install'
            d.independent(candidate)
            for relative, expected in original_payload.items():
                path = candidate / relative
                if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
                    raise ValueError('Rebuilt legacy content differs from old install: ' + relative)
                if relative.startswith('lib/') and not os.access(path, os.X_OK):
                    raise ValueError('Legacy executable lost execute permission: ' + relative)
                if path.suffix == '.py':
                    ast.parse(path.read_text(), filename=str(path))
            d.command(['bash', '--noprofile', '--norc', '-c',
                       'source /opt/ros/jazzy/setup.bash && source ' + q(str(candidate / 'setup.bash')) +
                       ' && test "$(ros2 pkg prefix create3_lidar_bringup)" = ' + q(str(candidate))])
            # Old rollback packages have no tests. Do not describe them as test-passed.
            d.state['validation'] = {'payload_files_verified': len(original_payload), 'source_snapshot_verified': True,
                                      'package_resolution_verified': True, 'hardware_verified': False,
                                      'legacy_automated_tests': 'not_claimed; old version preserved by content equivalence'}
            if manifest(d.source) != source_hashes or legacy_payload(d.entry) != original_payload:
                raise ValueError('Legacy source/install changed during maintenance; refuse activation')
            d.stage = 'activate legacy'; d.record()
            backup = d.ws / ('.legacy-install-' + d.attempt.name)
            backup.symlink_to(candidate)
            d.state.update(ever_activated=True, original_install_backup=str(backup))
            d.record()
            try:
                exchange(backup, d.entry)
            except Exception:
                if backup.is_symlink():
                    backup.unlink()
                raise
            # Original install directory is preserved at backup, and the original build is untouched.
            # The independent active candidate no longer relies on the mutable source mirror.
            d.stage = 'mirror permissions'; d.record()
            for directory in [d.source, *[x for x in d.source.rglob('*') if x.is_dir()]]:
                if directory.is_symlink():
                    raise ValueError('Source directory changed during permission update')
                directory.chmod(directory.stat().st_mode | stat.S_IWUSR)
            d.state['status'] = 'success'; d.record()
            return dict(d.state, note='Legacy version preserved, not upgraded. Next run sync.sh --apply. Open a fresh terminal after deployment. No nodes started; maps unchanged.')
        except Exception as error:
            print(json.dumps(d.failure(error)), flush=True)
            raise


def remote_main(apply=False):
    from remote import Deployment
    d = Deployment('/home/create3-pi/create3_ws')
    try:
        print(json.dumps(migrate(d) if apply else legacy_preflight(d), indent=2), flush=True)
    except Exception as error:
        if d.attempt is None:
            print(json.dumps(d.failure(error)), flush=True)
        raise SystemExit(getattr(error, 'returncode', 1) or 1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--stopped', action='store_true', help='Explicitly confirm robot launches/teleop are stopped for maintenance')
    args = parser.parse_args()
    if args.apply and not args.stopped:
        parser.error('--apply requires --stopped after stopping teleop and all project launches')
    import yaml
    here = Path(__file__).resolve().parent
    cfg = yaml.safe_load((here / 'config.yaml').read_text())
    host = cfg['host']
    if host.startswith('-') or any(c.isspace() for c in host):
        parser.error('Invalid SSH destination')
    remote_code = base64.b64encode((here / 'remote.py').read_bytes()).decode()
    migration_code = base64.b64encode(Path(__file__).read_bytes()).decode()
    # No remote files are written before explicit --apply; code travels over SSH.
    bootstrap = f"import sys,types,base64;sys.dont_write_bytecode=True;r=types.ModuleType('remote');sys.modules['remote']=r;exec(base64.b64decode('{remote_code}'),r.__dict__);m=types.ModuleType('migration');exec(base64.b64decode('{migration_code}'),m.__dict__);m.remote_main({args.apply!r})"
    return subprocess.run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10', '-o', 'StrictHostKeyChecking=yes',
                           '-o', 'HostKeyAlias=' + cfg['host_key_alias'], host, 'python3 -c ' + shlex.quote(bootstrap)]).returncode

if __name__ == '__main__':
    sys.exit(main())
