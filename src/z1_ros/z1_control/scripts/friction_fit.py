#!/usr/bin/env python3
"""Re-fit moving friction from an existing recording; does not connect to ROS."""
import argparse
import csv
from itertools import groupby
from pathlib import Path
import yaml
from z1_control.friction import identify_joint


def rows(path):
    with path.open(newline='') as source:
        for raw in csv.DictReader(source):
            if raw['kind'] != 'sweep':
                continue
            row = {key: raw[key] for key in ('arm', 'kind')}
            for key in ('joint', 'repeat', 'direction'):
                row[key] = int(raw[key])
            for key in ('speed', 'received_at'):
                row[key] = float(raw[key])
            for name in ('position', 'effort', 'gravity', 'command_velocity'):
                row[name] = [float(raw[name + str(j)]) for j in range(1, 7)]
            yield row


def refit(directory):
    report = yaml.safe_load((directory / 'result.yaml').read_text())
    report['results'] = report.get('results', {})
    seen = set()
    for (arm, joint), values in groupby(rows(directory / 'samples.csv'),
                                      key=lambda r: (r['arm'], r['joint'])):
        if (arm, joint) in seen:
            raise ValueError('recording has non-contiguous arm/joint groups')
        seen.add((arm, joint))
        previous = report['results'].setdefault(arm, {}).get('joint' + str(joint), {})
        try:
            result = identify_joint(list(values), joint - 1, report['config'])
        except ValueError as error:
            result = {'usable': False, 'reason': str(error)}
        if 'static' in previous:
            result['static'] = previous['static']
        report['results'][arm]['joint' + str(joint)] = result
    report['refit_from'] = 'samples.csv; static thresholds retained from acquisition'
    output = directory / 'result_refit.yaml'
    output.write_text(yaml.safe_dump(report, allow_unicode=True, sort_keys=False))
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path, help='recording directory containing samples.csv and result.yaml')
    print(refit(parser.parse_args().directory))
