#!/usr/bin/env python3
"""Save a live ROS map on this host, with bounded waits and optional pose graph."""
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
from datetime import datetime


def run(command, timeout):
    # ros2 run starts a child executable; terminate the whole group on timeout.
    process = subprocess.Popen(command, start_new_session=True)
    try:
        return process.wait(timeout=timeout)
    except (subprocess.TimeoutExpired, KeyboardInterrupt):
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path, default=Path(__file__).resolve().parents[1]
                        / 'maps' / datetime.now().strftime('room_%Y%m%d_%H%M%S'))
    parser.add_argument('--timeout', type=float, default=20.0,
                        help='map reception / pose graph wait in seconds (default: 20)')
    parser.add_argument('--pose-graph', action='store_true',
                        help='also serialize SLAM state for continued mapping')
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    directory = args.directory.expanduser().resolve()
    prefix = directory / 'map'
    try:
        directory.mkdir(parents=True, exist_ok=True)
        # Never overwrite a previously saved map or a partial save.
        if any(directory.glob('map.*')):
            print(f'ERROR: {directory} already contains map files; use a new directory.', file=sys.stderr)
            return 1
        print(f'Saving map to {directory}', flush=True)
        result = run([
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-t', '/map',
            '-f', str(prefix), '--fmt', 'pgm', '--ros-args',
            '-p', f'save_map_timeout:={args.timeout}',
            '-p', 'map_subscribe_transient_local:=true',
        ], args.timeout + 10)
        if result != 0:
            print('ERROR: map save failed. Keep the SLAM launch running; source ROS, '
                  '.env.local and install/setup.bash in this terminal. Check '
                  '`ros2 topic info /map -v` for a publisher and compatible QoS. '
                  'Pose graph serialization was not attempted.', file=sys.stderr)
            return 1
        if not all(path.is_file() and path.stat().st_size > 0
                   for path in (prefix.with_suffix('.yaml'), prefix.with_suffix('.pgm'))):
            print('ERROR: map saver did not produce nonempty map.yaml and map.pgm.', file=sys.stderr)
            return 1
        print(f'Map image saved: {prefix}.yaml + {prefix}.pgm', flush=True)
        if args.pose_graph:
            # JSON is valid YAML and safely quotes filenames in the service request.
            import json
            result = run([
                'ros2', 'service', 'call', '/slam_toolbox/serialize_map',
                'slam_toolbox/srv/SerializePoseGraph', json.dumps({'filename': str(prefix)}),
            ], args.timeout)
            graph_files = (prefix.with_suffix('.posegraph'), prefix.with_suffix('.data'))
            if result != 0 or not all(p.is_file() and p.stat().st_size > 0 for p in graph_files):
                print('ERROR: pose graph save failed. The map.yaml and map.pgm are still '
                      'available for copying and navigation.', file=sys.stderr)
                return 1
            print('Pose graph saved.', flush=True)
        print(f'Ready to copy this directory: {directory}', flush=True)
        return 0
    except subprocess.TimeoutExpired:
        print('ERROR: ROS command timed out. Keep SLAM running and check this terminal\'s '
              'DDS settings and `ros2 service list`. Any saved map.yaml/map.pgm '
              'remain available.', file=sys.stderr)
        return 1
    except OSError as error:
        print(f'ERROR: {error}. Check the output directory and source the ROS environment.',
              file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('\nMap save interrupted.', file=sys.stderr)
        return 130


if __name__ == '__main__':
    sys.exit(main())
