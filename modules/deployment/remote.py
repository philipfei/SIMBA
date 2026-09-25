"""Pi deployment transaction. No node launch, settings changes or source rollback."""
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import uuid

PACKAGE = 'create3_lidar_bringup'


def stamp():
    return datetime.now(timezone.utc).isoformat()


def atomic_json(path, data):
    temp = path.with_name('.' + path.name + '.tmp')
    with temp.open('w') as f:
        json.dump(data, f, indent=2)
        f.flush()
        os.fsync(f.fileno())
    os.replace(temp, path)


def manifest(root):
    result = {}
    for path in sorted(root.rglob('*')):
        if any(x in ('__pycache__', '.pytest_cache', '.git') for x in path.relative_to(root).parts) or path.suffix == '.pyc':
            continue
        if path.is_symlink():
            raise ValueError('Source symlink rejected: ' + str(path))
        if path.is_file():
            result[str(path.relative_to(root))] = hashlib.sha256(path.read_bytes()).hexdigest()
    return result


class Deployment:
    def __init__(self, workspace, keep=3):
        self.ws = Path(workspace)
        self.source = self.ws / 'src' / PACKAGE
        self.entry = self.ws / 'install'
        self.root = self.ws / '.simba-deploy'
        self.candidates = self.root / 'candidates'
        if isinstance(keep, bool) or not isinstance(keep, int) or keep < 1:
            raise ValueError('failed_candidate_retention_count must be a positive integer')
        self.keep = keep
        self.stage = 'preflight'
        self.attempt = None
        self.state = {'created_at': stamp(), 'status': 'running', 'source_sync': 'not_started', 'ever_activated': False}

    def active(self):
        return str(self.entry.resolve()) if self.entry.is_symlink() else ('unmanaged_install' if self.entry.exists() else None)

    def safe_path(self, path):
        # No symlink in a mutable deployment or source path, including ancestors.
        for part in [path, *path.parents]:
            if part.is_symlink():
                raise ValueError('Unexpected symlink: ' + str(part))
        if path.exists() and not path.is_dir():
            raise ValueError('Expected directory: ' + str(path))

    def independent(self, install):
        if not (install / 'setup.bash').is_file():
            raise ValueError('No runnable install setup: ' + str(install))
        for f in install.rglob('*'):
            if f.is_symlink() and not f.resolve().is_relative_to(install.resolve()):
                raise ValueError('Install depends on mutable/external files: ' + str(f))
        for name in ('setup.bash', 'setup.sh', 'local_setup.bash', 'local_setup.sh'):
            f = install / name
            if f.exists() and str(self.source) in f.read_text():
                raise ValueError('Install setup references mutable source')

    def preflight(self):
        for path in (self.ws, self.ws / 'src', self.source, self.root, self.candidates):
            self.safe_path(path)
        if not self.ws.is_dir():
            raise ValueError('Workspace missing; create it explicitly before deployment')
        if self.entry.exists() and not self.entry.is_symlink():
            raise ValueError('Unmanaged install directory. Explicitly protect/validate the old version and convert the entry during maintenance; source was not synced.')
        if self.entry.is_symlink():
            target = self.entry.resolve(strict=True)
            if target.name != 'install' or target.parent.parent != self.candidates.resolve():
                raise ValueError('Install entry is outside managed candidates')
            state = json.loads((target.parent / 'state.json').read_text())
            if not state.get('ever_activated'):
                raise ValueError('Active version has no activation record')
            self.independent(target)
        return {'stage': 'preflight', 'active_install': self.active(), 'prior_version': bool(self.entry.is_symlink())}

    @contextmanager
    def locked(self):
        self.root.mkdir(exist_ok=True)
        with (self.root / 'lock').open('a') as stream:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
            yield

    def record(self):
        self.state.update(stage=self.stage, active_install=self.active(), updated_at=stamp())
        if self.attempt:
            atomic_json(self.attempt / 'state.json', self.state)

    def cleanup(self):
        if not self.candidates.exists():
            return
        failed = []
        active = Path(self.active()).parent if self.entry.is_symlink() else None
        # Unknown states and successful versions are deliberately never removed.
        protected = []
        for directory in self.candidates.iterdir():
            if directory.is_symlink() or not directory.is_dir():
                continue
            try:
                state = json.loads((directory / 'state.json').read_text())
            except (OSError, ValueError):
                protected.append(directory)
                continue
            if state.get('status') == 'failed' and not state.get('ever_activated') and directory != self.attempt and directory != active:
                failed.append((state.get('created_at', ''), directory))
            else:
                protected.append(directory)
        failed.sort(key=lambda pair: (pair[0], pair[1].name), reverse=True)
        current_failed = int(bool(self.attempt and self.state.get('status') == 'failed'))
        protected += [directory for _, directory in failed[:max(0, self.keep-current_failed)]]
        # Never remove a failed candidate referenced by another retained directory.
        referenced = set()
        for directory in protected:
            for item in directory.rglob('*'):
                if item.is_symlink():
                    target = item.resolve()
                    for _, candidate in failed:
                        if target.is_relative_to(candidate.resolve()):
                            referenced.add(candidate)
        for _, directory in failed[max(0, self.keep-current_failed):]:
            if directory in referenced or directory.is_symlink() or directory.resolve().parent != self.candidates.resolve():
                continue
            size = sum(f.stat().st_size for f in directory.rglob('*') if f.is_file() and not f.is_symlink())
            state = json.loads((directory / 'state.json').read_text())
            # Preserve the small failure record even after full logs expire.
            archive = self.root / 'history'
            archive.mkdir(exist_ok=True)
            atomic_json(archive / (directory.name + '.json'), dict(state, cleanup_reason='failed_retention_limit', removed_bytes=size))
            shutil.rmtree(directory)
            print(json.dumps({'cleanup': directory.name, 'reason': 'failed_retention_limit', 'bytes': size}), flush=True)

    def begin(self):
        self.candidates.mkdir(exist_ok=True)
        self.cleanup()
        self.attempt = self.candidates / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S') + '-' + uuid.uuid4().hex[:10])
        self.attempt.mkdir()
        (self.attempt / 'logs').mkdir()
        self.record()

    def command(self, args):
        log = self.attempt / 'logs' / (self.stage.replace(' ', '_') + '.log')
        environment = {'HOME': str(Path.home()), 'PATH': '/usr/bin:/bin', 'LANG': 'C.UTF-8',
                       'PYTHONDONTWRITEBYTECODE': '1', 'ROS_DOMAIN_ID': '91', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
                       'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
                       'CYCLONEDDS_URI': '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General></Domain></CycloneDDS>'}
        with log.open('a') as stream:
            proc = subprocess.run(args, cwd=self.ws, env=environment, stdout=stream, stderr=subprocess.STDOUT)
        if proc.returncode:
            raise subprocess.CalledProcessError(proc.returncode, args)

    def installed_check(self, expected):
        install = self.attempt / 'install'
        self.independent(install)
        share = install / 'share' / PACKAGE
        if json.loads((share / 'source_manifest.json').read_text()) != expected:
            raise ValueError('Installed source manifest mismatch')
        for rel, digest in expected.items():
            source = Path(rel)
            target = None
            if source.parts[0] in ('config', 'launch', 'rviz'):
                target = share / source
            elif source.parts[0] == 'scripts' or rel == 'tools/preview_coverage.py':
                target = install / 'lib' / PACKAGE / source.name
                if not os.access(target, os.X_OK):
                    raise ValueError('Installed executable not executable: ' + str(target))
            elif source.parts[0] == 'create3_coverage' and source.suffix == '.py':
                matches = list(install.glob('lib/python*/site-packages/' + rel))
                if len(matches) != 1:
                    raise ValueError('Missing installed Python module: ' + rel)
                target = matches[0]
            if target and hashlib.sha256(target.read_bytes()).hexdigest() != digest:
                raise ValueError('Installed content mismatch: ' + rel)
        # A new shell must resolve the package from this exact candidate, not the active link.
        import shlex
        setup = shlex.quote(str(install / 'setup.bash'))
        prefix = shlex.quote(str(install))
        self.command(['bash', '--noprofile', '--norc', '-c',
                      f'source /opt/ros/jazzy/setup.bash && source {setup} && test "$(ros2 pkg prefix {PACKAGE})" = {prefix}'])

    def activate(self):
        install = self.attempt / 'install'
        self.state['ever_activated'] = True
        # Protect the candidate before switching even if the process dies just afterwards.
        self.record()
        temporary = self.ws / ('.install-' + uuid.uuid4().hex)
        try:
            temporary.symlink_to(install)
            os.replace(temporary, self.entry)
        finally:
            if temporary.is_symlink():
                temporary.unlink()
        self.state['status'] = 'success'
        self.record()

    def finish(self, expected):
        self.stage = 'hash audit'
        self.state['source_sync'] = 'completed'
        self.record()
        if manifest(self.source) != expected:
            raise ValueError('Source SHA-256 mismatch')
        import shlex
        q = shlex.quote
        base = f'source /opt/ros/jazzy/setup.bash && colcon --log-base {q(str(self.attempt / "logs/colcon"))}'
        common = f' --base-paths {q(str(self.source))} --packages-select {PACKAGE} --build-base {q(str(self.attempt / "build"))} --install-base {q(str(self.attempt / "install"))} --merge-install'
        self.stage = 'build'; self.record()
        self.command(['bash', '--noprofile', '--norc', '-c', base + ' build' + common])
        self.stage = 'test'; self.record()
        self.command(['bash', '--noprofile', '--norc', '-c', base + ' test' + common + ' --return-code-on-test-failure'])
        self.command(['bash', '--noprofile', '--norc', '-c', base + ' test-result --test-result-base ' + q(str(self.attempt / 'build')) + ' --verbose'])
        import xml.etree.ElementTree as ET
        xml_files = list((self.attempt / 'build').rglob('*.xunit.xml')) + list((self.attempt / 'build').rglob('*.xunit'))
        if not any(sum(int(x.attrib.get('tests', 0)) for x in ET.parse(f).getroot().iter('testsuite')) > 0 for f in xml_files):
            raise ValueError('No executed pytest cases; zero tests is not success')
        self.stage = 'install check'; self.record()
        self.installed_check(expected)
        self.stage = 'activate'; self.record()
        self.activate()

    def failure(self, error):
        self.state.update(status='failed', error=str(error), exit_code=getattr(error, 'returncode', 1))
        self.record()
        return dict(self.state, log_directory=str(self.attempt / 'logs') if self.attempt else None,
                    recovery='Fix and explicitly re-run --apply, or inspect/revert source manually. No source rollback was attempted.')


def main():
    import base64
    payload = json.loads(base64.b64decode(sys.argv[1]))
    deployment = Deployment('/home/create3-pi/create3_ws', payload['keep'])
    try:
        result = deployment.preflight()
        if not payload['apply']:
            print(json.dumps(result), flush=True)
            return
        with deployment.locked():
            deployment.preflight()
            deployment.begin()
            try:
                deployment.stage = 'sync'
                deployment.state['source_sync'] = 'may_be_partial'
                deployment.record()
                print(json.dumps({'ready': True, 'active_install': deployment.active()}), flush=True)
                response = json.loads(sys.stdin.readline())
                if not response.get('synced'):
                    raise ValueError('Source sync failed; source may be partially updated')
                deployment.finish(payload['manifest'])
                print(json.dumps({'success': True, 'active_install': deployment.active(), 'note': 'Open a NEW clean shell and source ROS and the resolved install setup before launch. Existing processes were not restarted.'}), flush=True)
            except Exception as error:
                print(json.dumps(deployment.failure(error)), flush=True)
                raise
            finally:
                try:
                    deployment.cleanup()
                except OSError as cleanup_error:
                    print(json.dumps({'cleanup_warning': str(cleanup_error), 'active_install': deployment.active(), 'action': 'No rollback or broader deletion attempted; inspect retained candidates manually.'}), flush=True)
    except Exception as error:
        if deployment.attempt is None:
            print(json.dumps(deployment.failure(error)), flush=True)
        sys.exit(getattr(error, 'returncode', 1) or 1)

if __name__ == '__main__':
    main()
