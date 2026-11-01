#!/usr/bin/env python3
"""Recompute A/B metrics and plots from recorded data, without ROS or hardware."""
import argparse
from pathlib import Path
from z1_control.friction_comparison import analyze_recording

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    result = analyze_recording(args.directory)
    print('all_improved:', result['all_improved'], '\nReport:', args.directory / 'comparison.yaml')
