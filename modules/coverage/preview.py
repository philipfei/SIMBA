#!/usr/bin/env python3
"""Generate an offline ideal preview; never publish robot commands."""
from pathlib import Path
import sys
sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/create3_lidar_bringup'))
from create3_coverage.offline import preview_main
if __name__ == '__main__':
    preview_main()
