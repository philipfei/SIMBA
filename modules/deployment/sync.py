#!/usr/bin/env python3
"""Read-only difference by default; explicit, staged Pi deployment with --apply."""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import sys
sys.dont_write_bytecode = True
import yaml

ROOT = Path(__file__).resolve().parents[2]
REMOTE_SOURCE = '/home/create3-pi/create3_ws/src/create3_lidar_bringup'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    cfg = yaml.safe_load((Path(__file__).with_name('config.yaml')).read_text())
    host = cfg['host']
    if not isinstance(host, str) or host.startswith('-') or any(c.isspace() for c in host):
        parser.error('Invalid SSH destination')
    source = ROOT / 'src/create3_lidar_bringup'
    for ancestor in [source, *source.parents]:
        if ancestor.is_symlink():
            parser.error('Source ancestry must not contain symlinks')
    sys.path.insert(0, str(Path(__file__).parent))
    from remote import manifest
    payload = {'apply': args.apply, 'keep': cfg['failed_candidate_retention_count'], 'manifest': manifest(source)}
    code = base64.b64encode(Path(__file__).with_name('remote.py').read_bytes()).decode()
    encoded = base64.b64encode(json.dumps(payload).encode()).decode()
    remote_command = 'python3 -c ' + shlex.quote(f"import base64;exec(compile(base64.b64decode('{code}'),'<simba-deployment>','exec'))") + ' ' + shlex.quote(encoded)
    ssh = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10', '-o', 'StrictHostKeyChecking=yes', '-o', 'HostKeyAlias=' + cfg['host_key_alias']]
    proc = subprocess.Popen(ssh + [host, remote_command], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    ready = not args.apply
    if args.apply:
        for line in proc.stdout:
            print(line, end='', flush=True)
            try:
                if json.loads(line).get('ready'):
                    ready = True
                    break
            except ValueError:
                pass
    else:
        for line in proc.stdout:
            print(line, end='')
        if proc.wait():
            return proc.returncode
    if not ready:
        return proc.wait() or 1
    command = ['rsync', '-avi', '--delete', '--exclude=__pycache__/', '--exclude=*.pyc', '--exclude=.pytest_cache/',
               '-e', shlex.join(ssh)]
    if not args.apply:
        command.append('--dry-run')
    command += [str(source) + '/', host + ':' + REMOTE_SOURCE + '/']
    result = subprocess.run(command)
    if not args.apply:
        return result.returncode
    proc.stdin.write(json.dumps({'synced': result.returncode == 0}) + '\n')
    proc.stdin.flush()
    proc.stdin.close()
    for line in proc.stdout:
        print(line, end='', flush=True)
    return proc.wait() or result.returncode

if __name__ == '__main__':
    sys.exit(main())
