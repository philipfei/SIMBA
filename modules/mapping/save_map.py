#!/usr/bin/env python3
"""Save a new Pi map and download all four files with SHA-256 verification."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess
import sys
sys.dont_write_bytecode = True
import tempfile
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'common'))
from pi import command, ssh_args, ROOT

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--name', default='room_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    parser.add_argument('--download-only', action='store_true')
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', args.name):
        parser.error('Use letters, digits, underscore and dash for map name')
    target = ROOT / 'maps' / args.name
    if target.exists():
        parser.error('Local map already exists; refusing overwrite')
    remote = '/home/create3-pi/create3_ws/maps/' + args.name
    if not args.download_only:
        save = f"mkdir {shlex.quote(remote)}\n"
        save += 'ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap ' + shlex.quote(json.dumps({'name': {'data': remote + '/map'}})) + '\n'
        save += 'ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph ' + shlex.quote(json.dumps({'filename': remote + '/map'}))
        subprocess.run(command(save), check=True)
    check = "from pathlib import Path;import hashlib,json;p=Path(" + repr(remote) + ");names=['map.yaml','map.pgm','map.posegraph','map.data'];assert all((p/n).is_file() and (p/n).stat().st_size for n in names);print(json.dumps({n:hashlib.sha256((p/n).read_bytes()).hexdigest() for n in names}))"
    expected = json.loads(subprocess.check_output(command('python3 -c ' + shlex.quote(check), ros=False), text=True))
    with tempfile.TemporaryDirectory(prefix='.map-download-', dir=ROOT / 'maps') as staging:
        for name in expected:
            ssh = ssh_args()
            subprocess.run(['rsync', '-a', '-e', shlex.join(ssh[:-1]), ssh[-1] + ':' + remote + '/' + name, staging + '/'], check=True)
            actual = hashlib.sha256((Path(staging) / name).read_bytes()).hexdigest()
            if actual != expected[name]:
                raise ValueError('Map hash mismatch: ' + name)
        # Publish the complete map only after all four verified files are present.
        if target.exists():
            raise ValueError('Destination appeared during download; refusing overwrite')
        Path(staging).rename(target)
    print(target)

if __name__ == '__main__':
    main()
