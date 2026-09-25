#!/usr/bin/env python3
"""Explicit native Dock/Undock only when project motion controllers are stopped."""
import argparse
from pathlib import Path
import subprocess
import sys
sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'common'))
from pi import command
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('action', choices=['dock', 'undock'])
a = p.parse_args()
type_name = 'Dock' if a.action == 'dock' else 'Undock'
script = r"""nodes=$(ros2 node list)
if echo "$nodes" | grep -Eq 'velocity_safety_gate|coverage_supervisor|controller_server|safe_cmd_vel'; then
  echo 'Stop project motion nodes first. Use supervised return_to_dock during an active coverage task.' >&2
  exit 2
fi
publishers=$(ros2 topic info /cmd_vel 2>/dev/null || true)
if echo "$publishers" | grep -Eq 'Publisher count: [1-9]'; then
  echo 'External velocity publishers remain; refuse native action.' >&2
  exit 2
fi
"""
script += f'exec ros2 action send_goal /{a.action} irobot_create_msgs/action/{type_name} "{{}}"'
sys.exit(subprocess.run(command(script)).returncode)
