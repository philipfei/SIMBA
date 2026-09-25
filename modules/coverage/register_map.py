#!/usr/bin/env python3
"""Register an explicit, map-frame example start for offline evaluation."""
from pathlib import Path
import sys
sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/create3_lidar_bringup'))
from create3_coverage.offline import register_main
if __name__ == '__main__':
    register_main()
