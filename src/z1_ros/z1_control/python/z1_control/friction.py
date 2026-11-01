"""Joint-side friction identification. No ROS or hardware dependency.

The fit uses measured torque minus model gravity, paired at matching measured
positions in opposite directions. Coulomb and viscous parameters are the odd,
direction-averaged friction, not independently identifiable directional biases.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass
class Point:
    time: float
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray


def sweep(start, axis, end, speed, acceleration, jerk):
    """Quartic position ramps with a true constant-velocity middle segment.

    Each velocity ramp is cubic smoothstep. Max acceleration is 1.5*v/T,
    max jerk is 6*v/T^2. Quintic Hermite reproduces these ramps exactly.
    """
    start = np.asarray(start, dtype=float)
    if start.shape != (6,) or not np.isfinite(start).all() or axis not in range(6):
        raise ValueError('invalid sweep start or joint')
    if not all(math.isfinite(x) and x > 0 for x in (speed, acceleration, jerk)):
        raise ValueError('sweep limits must be finite and positive')
    distance = abs(end - start[axis])
    ramp = max(1.5 * speed / acceleration, math.sqrt(6 * speed / jerk))
    cruise = distance / speed - ramp
    if not math.isfinite(end) or cruise <= 0:
        raise ValueError('sweep interval is too short for the requested speed')
    direction = math.copysign(1., end - start[axis])
    points = []
    for t, offset, velocity in (
            (0., 0., 0.),
            (ramp, .5 * speed * ramp, speed),
            (ramp + cruise, distance - .5 * speed * ramp, speed),
            (2 * ramp + cruise, distance, 0.)):
        q = start.copy()
        q[axis] += direction * offset
        v = np.zeros(6)
        v[axis] = direction * velocity
        points.append(Point(t, q, v, np.zeros(6)))
    return points


def static_ramp(start, axis, offset, speed):
    start = np.asarray(start, dtype=float)
    if not math.isfinite(offset) or offset == 0 or speed <= 0:
        raise ValueError('invalid static ramp')
    end = start.copy()
    end[axis] += offset
    # Quintic smoothstep, peak velocity 1.875*distance/duration.
    duration = 1.875 * abs(offset) / speed
    return [Point(0., start.copy(), np.zeros(6), np.zeros(6)),
            Point(duration, end, np.zeros(6), np.zeros(6))]


def local_velocity(time, position, window=.08):
    """Centered least-squares slope on actual, possibly irregular timestamps."""
    t = np.asarray(time, dtype=float)
    q = np.asarray(position, dtype=float)
    if len(t) < 5 or not (np.diff(t) > 0).all():
        raise ValueError('insufficient or nonmonotonic fresh feedback')
    t = t - t[0]
    lo = np.searchsorted(t, t - window / 2)
    hi = np.searchsorted(t, t + window / 2, side='right')
    count = hi - lo
    def sums(values):
        c = np.r_[0., np.cumsum(values)]
        return c[hi] - c[lo]
    st, sq = sums(t), sums(q)
    denominator = sums(t * t) - st * st / count
    if (count < 3).any() or (denominator <= 1e-15).any():
        raise ValueError('feedback resolution is insufficient for velocity fitting')
    return (sums(t * q) - st * sq / count) / denominator


def _pass(rows, axis, speed, options):
    t = np.array([r['received_at'] for r in rows])
    q = np.array([r['position'][axis] for r in rows])
    tau = np.array([r['effort'][axis] - r['gravity'][axis] for r in rows])
    commanded = np.array([r['command_velocity'][axis] for r in rows])
    if not np.isfinite(np.r_[t, q, tau, commanded]).all():
        raise ValueError('nonfinite experiment data')
    if np.max(np.diff(t)) > options.get('max_sample_gap', .02):
        raise ValueError('missing feedback inside sweep')
    v = local_velocity(t, q, options.get('velocity_window', .08))
    a = local_velocity(t, v, options.get('velocity_window', .08))
    direction = math.copysign(1., np.median(commanded))
    cruise = np.abs(commanded - direction * speed) < max(1e-6, speed * .01)
    cruise &= (t > t[0] + .15) & (t < t[-1] - .15)
    if cruise.sum() < 20:
        raise ValueError('no constant reference velocity interval')
    valid = cruise & (np.abs(v - direction * speed) <= speed * options.get('speed_tolerance', .2))
    valid &= np.abs(a) <= options.get('steady_acceleration', .03)
    if valid.sum() / cruise.sum() < options.get('min_steady_fraction', .6):
        raise ValueError('stick-slip or insufficient steady measured velocity')
    accepted_time = t[valid]
    # Do not interpolate across a stall hidden by selecting only moving samples.
    if np.max(np.diff(accepted_time)) > options.get('max_steady_gap', .12):
        raise ValueError('discontinuous steady interval (possible stick-slip)')
    if accepted_time[-1] - accepted_time[0] < options.get('min_steady_duration', .5):
        raise ValueError('steady interval too short')
    order = np.argsort(q[valid])
    return q[valid][order], tau[valid][order], float(np.median(np.abs(v[valid])))


def paired_measurement(positive, negative, axis, speed, options=None):
    options = options or {}
    qp, tp, vp = _pass(positive, axis, speed, options)
    qn, tn, vn = _pass(negative, axis, speed, options)
    lower, upper = max(qp[0], qn[0]), min(qp[-1], qn[-1])
    if upper - lower < options.get('min_position_overlap', .01):
        raise ValueError('opposite passes have no matched position interval')
    grid = np.linspace(lower, upper, 101)
    values = .5 * (np.interp(grid, qp, tp) - np.interp(grid, qn, tn))
    value = float(np.median(values))
    return {'speed': .5 * (vp + vn), 'requested_speed': float(speed),
            'friction': value,
            'scatter': float(1.4826 * np.median(np.abs(values - value))),
            'overlap_rad': float(upper - lower)}


def fit_coulomb_viscous(points):
    if len(points) < 3:
        raise ValueError('at least three paired speeds are required')
    x = np.array([p['speed'] for p in points])
    y = np.array([p['friction'] for p in points])
    if not np.isfinite(np.r_[x, y]).all() or np.ptp(x) < .01:
        raise ValueError('insufficient measured speed range')
    if np.median(y) <= 0:
        raise ValueError('friction sign is inconsistent with positive drive torque')
    A = np.column_stack([np.ones(len(x)), x])
    weights = np.ones(len(x))
    coefficients = np.zeros(2)
    for _ in range(12):
        root = np.sqrt(weights)
        beta = np.linalg.lstsq(A * root[:, None], y * root, rcond=None)[0]
        candidates = [np.array([max(0., np.average(y, weights=weights)), 0.]),
                      np.array([0., max(0., np.sum(weights * x * y) / np.sum(weights * x * x))])]
        if (beta >= 0).all():
            candidates.append(beta)
        coefficients = min(candidates, key=lambda b: np.sum(weights * (y - A @ b) ** 2))
        residual = y - A @ coefficients
        scale = max(1e-6, 1.4826 * np.median(np.abs(residual - np.median(residual))))
        weights = np.minimum(1., 1.5 * scale / np.maximum(np.abs(residual), 1e-12))
    return {'coulomb_nm': float(coefficients[0]),
            'viscous_nm_s_per_rad': float(coefficients[1]),
            'training_mae_nm': float(np.mean(np.abs(y - A @ coefficients)))}


def identify_joint(rows, axis, options):
    groups = {}
    for row in rows:
        if row['kind'] == 'sweep':
            groups.setdefault((row['repeat'], row['speed'], row['direction']), []).append(row)
    points, rejected = [], []
    for repeat, speed in sorted({(key[0], key[1]) for key in groups}):
        try:
            point = paired_measurement(groups.get((repeat, speed, 1), []),
                                       groups.get((repeat, speed, -1), []), axis, speed, options)
            point['repeat'] = repeat
            points.append(point)
        except (ValueError, IndexError) as error:
            rejected.append({'repeat': repeat, 'speed': speed, 'reason': str(error)})
    validation_repeat = options['repeats'] - 1
    training = [p for p in points if p['repeat'] < validation_repeat]
    validation = [p for p in points if p['repeat'] == validation_repeat]
    if len({round(p['requested_speed'], 8) for p in training}) < 3:
        raise ValueError('fewer than three usable training speeds: ' + str(rejected))
    result = fit_coulomb_viscous(training)
    result['paired_measurements'] = points
    result['rejected_pairs'] = rejected
    if len(validation) < 3:
        result.update(usable=False, reason='fewer than three held-out speed pairs')
        return result
    errors = [abs(p['friction'] - result['coulomb_nm'] -
                  result['viscous_nm_s_per_rad'] * p['speed']) for p in validation]
    mae = float(np.mean(errors))
    noise = float(np.median([p['scatter'] for p in validation]))
    magnitude = float(np.median([abs(p['friction']) for p in validation]))
    limit = max(3 * noise, .2 * magnitude)
    resolved = magnitude > 3 * noise
    result.update(validation_mae_nm=mae, validation_limit_nm=limit,
                  usable=bool(resolved and mae <= limit),
                  reason=('friction is below within-pass scatter' if not resolved else
                          'validated' if mae <= limit else 'held-out residual exceeds limit'))
    return result


class BreakawayDetector:
    def __init__(self, base, direction, threshold, persistence=.05):
        self.base, self.direction = base, direction
        self.threshold, self.persistence = threshold, persistence
        self.history = []
        self.candidate = None
        self.result = None

    def update(self, time, position, residual_torque):
        if self.result is not None:
            return self.result
        moved = self.direction * (position - self.base) >= self.threshold
        if moved:
            if self.candidate is None:
                if not self.history:
                    return None
                self.candidate = (time, float(np.median(self.history[-5:])))
            elif time - self.candidate[0] >= self.persistence:
                self.result = {'onset_at': float(self.candidate[0]),
                               'residual_torque_nm': self.candidate[1]}
        else:
            self.candidate = None
        self.history.append(residual_torque)
        self.history = self.history[-8:]
        return self.result
