#!/usr/bin/env python3
"""Run an explicit command on the Pi in a new, pinned ROS environment."""
from pathlib import Path
import shlex
import subprocess
import sys
sys.dont_write_bytecode = True
import yaml

ROOT = Path(__file__).resolve().parents[2]

def ssh_args():
    cfg = yaml.safe_load((ROOT / 'modules/deployment/config.yaml').read_text())
    host = cfg['host']
    if host.startswith('-') or any(c.isspace() for c in host):
        raise ValueError('Invalid SSH destination')
    return ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10', '-o', 'StrictHostKeyChecking=yes',
            '-o', 'HostKeyAlias=' + cfg['host_key_alias'], host]


def command(script, ros=True):
    prefix = r"""set -e
source /opt/ros/jazzy/setup.bash
ip link show eth0 >/dev/null
ip link show wlan0 >/dev/null
installed=$(readlink -f /home/create3-pi/create3_ws/install)
test -f "$installed/setup.bash"
source "$installed/setup.bash"
export ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" priority="10" multicast="default"/><NetworkInterface name="wlan0" priority="20" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'
""" if ros else 'set -e\n'
    # A clean shell cannot inherit an old overlay from a login profile.
    return ssh_args() + ['env -i HOME=/home/create3-pi PATH=/usr/bin:/bin LANG=C.UTF-8 bash --noprofile --norc -c ' + shlex.quote(prefix + script)]


def main():
    if len(sys.argv) == 1 or sys.argv[1] in ('-h', '--help'):
        print('Usage: pi.py COMMAND [ARG ...]\nRuns only the explicitly requested command on the Pi.')
        return
    raise SystemExit(subprocess.run(command('exec ' + shlex.join(sys.argv[1:]))).returncode)

if __name__ == '__main__':
    main()
