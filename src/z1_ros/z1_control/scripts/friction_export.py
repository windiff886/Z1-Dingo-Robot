#!/usr/bin/env python3
"""Export usable dynamic identification results; never changes a live controller."""
import argparse
from pathlib import Path
from z1_control.friction_profile import export_profile


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    source = Path(__file__).resolve().parents[1]
    default = source / 'config/friction_compensation.yaml'
    if not default.exists():
        default = source.parent / 'share/z1_control/config/friction_compensation.yaml'
    parser.add_argument('--output', type=Path, default=default)
    args = parser.parse_args()
    backup = export_profile(args.recording, args.output)
    print(
        'Updated:',
        args.output,
        '\nBackup:',
        backup,
        '\nRestart controllers to load the coefficients.',
    )
