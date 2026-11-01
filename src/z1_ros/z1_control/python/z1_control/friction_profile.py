"""Validate and merge identified joint-side friction into the shared profile."""
import copy
from datetime import datetime
import math
from pathlib import Path
import shutil
import tempfile

import yaml


def validate_profile(profile):
    if (
        type(profile.get('schema_version')) is not int
        or profile.get('schema_version') != 1
        or type(profile.get('enabled')) is not bool
    ):
        raise ValueError('invalid friction profile schema or enabled flag')
    for key in (
        'velocity_scale_rad_s',
        'impedance_velocity_filter_s',
        'impedance_torque_rate_nm_s',
    ):
        if type(profile[key]) is bool or not math.isfinite(profile[key]) or profile[key] <= 0:
            raise ValueError(key + ' must be finite and positive')
    for side in ('left', 'right'):
        arm = profile['arms'][side]
        for key in ('coulomb_nm', 'viscous_nm_s_per_rad'):
            if len(arm[key]) != 6 or any(
                type(x) is bool or not math.isfinite(x) or x < 0 for x in arm[key]
            ):
                raise ValueError(side + ' ' + key + ' must contain six finite nonnegative numbers')
        if arm.get('has_gripper') is not None and type(arm['has_gripper']) is not bool:
            raise ValueError('has_gripper must be boolean or null')
        if not isinstance(arm.get('model_revision', ''), str):
            raise ValueError('model_revision must be a string')
    return profile


def merge_identification(profile, report):
    """All requested dynamic results must be usable; unmeasured axes survive."""
    result = copy.deepcopy(validate_profile(profile))
    if report.get('status') not in ('complete', 'complete_with_rejections') or not report.get(
        'results'
    ):
        raise ValueError('identification did not finish or contains no results')
    for side, joints in report['results'].items():
        if side not in ('left', 'right') or not joints:
            raise ValueError('invalid or empty arm results')
        source = report['models'][side]
        if type(source['has_gripper']) is not bool or not source['revision']:
            raise ValueError('missing actual model provenance')
        arm = result['arms'][side]
        if any(name not in {'joint%d' % j for j in range(1, 7)} for name in joints):
            raise ValueError('invalid joint result name')
        measured = {int(name[5:]) - 1 for name in joints}
        if not measured <= set(range(6)):
            raise ValueError('joint index outside 1..6')
        # Never silently mix old nonzero axes from another attachment/model.
        retained = [
            j
            for j in range(6)
            if j not in measured and (arm['coulomb_nm'][j] or arm['viscous_nm_s_per_rad'][j])
        ]
        if retained and (
            arm.get('has_gripper') != source['has_gripper']
            or arm.get('model_revision', '') != source['revision']
        ):
            raise ValueError(
                side + ': unmeasured nonzero axes belong to another model; re-identify them'
            )
        for name, values in joints.items():
            if values.get('usable') is not True:
                raise ValueError(
                    side
                    + ' '
                    + name
                    + ': dynamic result rejected: '
                    + values.get('reason', 'unknown')
                )
            j = int(name[5:]) - 1
            arm['coulomb_nm'][j] = values['coulomb_nm']
            arm['viscous_nm_s_per_rad'][j] = values['viscous_nm_s_per_rad']
        arm['has_gripper'], arm['model_revision'] = source['has_gripper'], source['revision']
    return validate_profile(result)


def export_profile(recording, output):
    recording, output = Path(recording), Path(output)
    report_file = recording / 'result.yaml' if recording.is_dir() else recording
    profile = yaml.safe_load(output.read_text())
    merged = merge_identification(profile, yaml.safe_load(report_file.read_text()))
    merged['last_identification'] = str(report_file.resolve())
    backup = output.with_name(output.name + '.bak-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    shutil.copy2(output, backup)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            'w', dir=output.parent, prefix=output.name + '.', delete=False
        ) as target:
            temporary = Path(target.name)
            yaml.safe_dump(merged, target, allow_unicode=True, sort_keys=False)
        temporary.chmod(output.stat().st_mode & 0o777)
        temporary.replace(output)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    return backup
