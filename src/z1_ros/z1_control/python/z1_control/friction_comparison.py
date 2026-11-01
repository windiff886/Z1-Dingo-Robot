"""Measured A/B tracking metrics and plots; no ROS or hardware dependency."""
import csv
from itertools import groupby
from pathlib import Path
import numpy as np
import yaml
from .friction import local_velocity, sweep, Point


CASES = ('uncompensated', 'compensated')


def round_trip(center, axis, amplitude, speed, acceleration, jerk):
    start = np.asarray(center, dtype=float).copy()
    start[axis] -= amplitude
    forward = sweep(start, axis, center[axis] + amplitude, speed, acceleration, jerk)
    reverse = sweep(forward[-1].position, axis, center[axis] - amplitude, speed, acceleration, jerk)
    return forward + [
        Point(p.time + forward[-1].time, p.position, p.velocity, p.acceleration)
        for p in reverse[1:]
    ]


def trial_metrics(rows, axis, speed, max_gap=0.02):
    t = np.array([r['received_at'] for r in rows])
    q = np.array([r['position'][axis] for r in rows])
    reference = np.array([r['command_position'][axis] for r in rows])
    velocity = np.array([r['command_velocity'][axis] for r in rows])
    if len(t) < 20 or not np.isfinite(np.r_[t, q, reference, velocity]).all():
        raise ValueError('insufficient or nonfinite comparison data')
    dt = np.diff(t)
    if (dt <= 0).any() or dt.max() > max_gap:
        raise ValueError('missing or nonmonotonic comparison feedback')
    weights = np.r_[dt[0], dt]
    measured = local_velocity(t, q, 0.08)
    moving = np.abs(velocity) > 1e-5
    cruise = np.abs(np.abs(velocity) - speed) < speed * 0.01
    cruise &= np.abs(local_velocity(t, velocity, 0.08)) < 0.03
    if moving.sum() < 20 or cruise.sum() < 20:
        raise ValueError('comparison lacks moving/constant-reference-velocity samples')
    error = q - reference
    rms = lambda x, mask: float(np.sqrt(np.average(x[mask] ** 2, weights=weights[mask])))
    return {
        'position_rms_rad': rms(error, moving),
        'position_mae_rad': float(np.average(np.abs(error[moving]), weights=weights[moving])),
        'position_max_abs_rad': float(np.max(np.abs(error[moving]))),
        'cruise_velocity_error_rms_rad_s': rms(measured - velocity, cruise),
        'cruise_stall_fraction': float(
            np.average(
                (np.abs(measured[cruise]) < max(0.002, 0.2 * speed)), weights=weights[cruise]
            )
        ),
        'samples': len(t),
        'moving_samples': int(moving.sum()),
        'cruise_samples': int(cruise.sum()),
    }


def compare_joint(rows, axis, config):
    groups = {}
    for row in rows:
        case = row['kind']
        if case not in CASES:
            continue
        if bool(row['friction_compensation_enabled']) != (case == 'compensated'):
            raise ValueError('actual compensation switch does not match trial label')
        groups.setdefault((row['repeat'], case), []).append(row)
    pairs = []
    for repeat in range(config['repeats']):
        metrics = {
            case: trial_metrics(
                groups[(repeat, case)], axis, config['speeds'][0], config['max_sample_gap']
            )
            for case in CASES
        }
        pairs.append(dict(repeat=repeat, **metrics))
    summary = {}
    for case in CASES:
        summary[case] = {
            key: float(np.median([p[case][key] for p in pairs]))
            for key in (
                'position_rms_rad',
                'position_mae_rad',
                'position_max_abs_rad',
                'cruise_velocity_error_rms_rad_s',
                'cruise_stall_fraction',
            )
        }
    base, comp = (summary[case] for case in CASES)
    denominator = base['position_rms_rad']
    reduction = 1 - comp['position_rms_rad'] / denominator if denominator > 1e-12 else 0.0
    improved = (
        reduction >= config['minimum_rms_reduction']
        and comp['cruise_velocity_error_rms_rad_s']
        <= base['cruise_velocity_error_rms_rad_s'] + 1e-6
        and comp['cruise_stall_fraction'] <= base['cruise_stall_fraction'] + 1e-12
    )
    return dict(
        pairs=pairs,
        summary=summary,
        position_rms_reduction_percent=100 * reduction,
        improved=bool(improved),
        verdict='improved' if improved else 'no_clear_improvement',
    )


def plot_joint(rows, axis, result, output, title):
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(12, 7))
    colors = {'uncompensated': '#df7662', 'compensated': '#139c89'}
    for case in CASES:
        first = [r for r in rows if r['repeat'] == 0 and r['kind'] == case]
        t = np.array([r['received_at'] for r in first])
        t -= t[0]
        q = np.array([r['position'][axis] for r in first])
        qr = np.array([r['command_position'][axis] for r in first])
        vr = np.array([r['command_velocity'][axis] for r in first])
        axes[0, 0].plot(t, np.degrees(q), color=colors[case], label=case)
        axes[0, 1].plot(t, 1000 * (q - qr), color=colors[case], label=case)
        axes[1, 0].plot(t, local_velocity(t, q, 0.08), color=colors[case], label=case)
        if case == CASES[0]:
            axes[0, 0].plot(t, np.degrees(qr), '--', color='#34465a', label='sent reference')
            axes[1, 0].plot(t, vr, '--', color='#34465a', label='sent velocity')
        pairs = result['pairs']
        x = np.arange(len(pairs))
        offset = -0.18 if case == CASES[0] else 0.18
        axes[1, 1].bar(
            x + offset,
            [p[case]['position_rms_rad'] * 1000 for p in pairs],
            width=0.36,
            color=colors[case],
            label=case,
        )
    for ax, label in zip(
        axes.flat,
        (
            'Joint angle (deg)',
            'Position error (mrad)',
            'Velocity (rad/s)',
            'Moving-phase RMS (mrad)',
        ),
    ):
        ax.set_ylabel(label)
        ax.grid(alpha=0.2)
        ax.legend(fontsize=8)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
    for ax in (axes[0, 0], axes[0, 1], axes[1, 0]):
        ax.set_xlabel('Time since first trial sample (s)')
    axes[1, 1].set_xlabel('Paired repetition (AB / BA alternating)')
    fig.suptitle(
        title
        + ' | '
        + result['verdict']
        + ' | RMS reduction %.1f%%' % result['position_rms_reduction_percent']
    )
    fig.text(
        0.5,
        0.01,
        'Curves: first paired repetition; bars: all pairs. Same gains/trajectory/limits; only friction switch changes.',
        ha='center',
        fontsize=9,
    )
    fig.tight_layout(rect=(0, 0.04, 1, 0.94))
    fig.savefig(output, dpi=145)
    plt.close(fig)


def analyze_recording(directory, plots=True):
    directory = Path(directory)
    acquisition = yaml.safe_load((directory / 'result.yaml').read_text())
    completed = {int(t['command_id']) for t in acquisition.get('completed_trials', [])}
    config = acquisition['config']
    report = {
        'source': 'measured SDK control_samples',
        'acquisition_status': acquisition['status'],
        'error_reference': 'position minus actually sent command_position, without lag alignment',
        'velocity_estimator': '80 ms local position slope; cruise metrics exclude planned ramps/stops',
        'results': {},
        'all_improved': False,
    }

    def records():
        with (directory / 'samples.csv').open(newline='') as source:
            for raw in csv.DictReader(source):
                if raw['kind'] not in CASES or int(raw['active_command_id']) not in completed:
                    continue
                row = {k: raw[k] for k in ('arm', 'kind')}
                for key in ('joint', 'repeat'):
                    row[key] = int(raw[key])
                row['received_at'] = float(raw['received_at'])
                row['friction_compensation_enabled'] = (
                    raw['friction_compensation_enabled'] == 'True'
                )
                for key in ('position', 'command_position', 'command_velocity'):
                    row[key] = [float(raw[key + str(j)]) for j in range(1, 7)]
                yield row

    seen = set()
    for (side, joint), values in groupby(records(), key=lambda r: (r['arm'], r['joint'])):
        if (side, joint) in seen:
            raise ValueError('non-contiguous arm/joint recording')
        seen.add((side, joint))
        rows = list(values)
        try:
            result = compare_joint(rows, joint - 1, config)
            if plots:
                plot_joint(
                    rows,
                    joint - 1,
                    result,
                    directory / ('%s_joint%d.png' % (side, joint)),
                    side + ' J' + str(joint),
                )
        except (ValueError, KeyError) as error:
            result = {'improved': False, 'verdict': 'insufficient_valid_data', 'reason': str(error)}
        report['results'].setdefault(side, {})['joint' + str(joint)] = result
    expected = {(s, j) for s in config['arms'] for j in config['joints']}
    report['all_improved'] = (
        seen == expected
        and acquisition['status'] == 'complete'
        and all(v['improved'] for arm in report['results'].values() for v in arm.values())
    )
    (directory / 'comparison.yaml').write_text(
        yaml.safe_dump(report, allow_unicode=True, sort_keys=False)
    )
    return report
