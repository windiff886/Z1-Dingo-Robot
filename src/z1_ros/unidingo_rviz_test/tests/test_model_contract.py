#!/usr/bin/env python3
"""Offline URDF/Xacro checks; requires xacro and numpy, no ROS master/nodes."""
from pathlib import Path
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np
import xacro

ROOT = Path(__file__).resolve().parents[4]
PACKAGES = {ET.parse(p).getroot().findtext('name'): p.parent
            for p in (ROOT / 'src').rglob('package.xml')}


def expand_model(path, **mappings):
    # Resolve this repo's package:// / $(find) references without a ROS 1 install.
    # Noetic xacro delegates to roslaunch rather than exposing its own
    # substitution_args module. Intercept only package lookup on both versions.
    original = xacro.eval_extension

    def extension(value):
        if value.startswith('$(find ') and value.endswith(')'):
            return str(PACKAGES[value[len('$(find '):-1]])
        return original(value)

    with patch.object(xacro, 'eval_extension', side_effect=extension):
        return ET.fromstring(xacro.process_file(str(path), mappings=mappings).toxml())


def expand(**mappings):
    return expand_model(PACKAGES['uni_dingo'] / 'urdf/uni_dingo_dual_arm.urdf.xacro', **mappings)


def rotation(axis, angle):
    axis = np.array(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * skew @ skew


def origin(joint):
    transform = np.eye(4)
    value = joint.find('origin')
    if value is not None:
        transform[:3, 3] = np.fromstring(value.get('xyz', '0 0 0'), sep=' ')
        r, p, y = np.fromstring(value.get('rpy', '0 0 0'), sep=' ')
        transform[:3, :3] = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
    return transform


def fk(robot, tip, positions):
    joints = {j.find('child').get('link'): j for j in robot.findall('joint')}
    result = np.eye(4)
    while tip in joints:
        joint = joints[tip]
        local = origin(joint)
        if joint.get('type') in ('revolute', 'continuous'):
            local[:3, :3] = local[:3, :3] @ rotation(
                np.fromstring(joint.find('axis').get('xyz'), sep=' '), positions[joint.get('name')])
        result = local @ result
        tip = joint.find('parent').get('link')
    return result


class ModelContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = expand()
        cls.control = ET.parse(PACKAGES['z1_description'] / 'urdf/z1_control.urdf').getroot()

    def test_complete_tree_and_local_meshes(self):
        links = [n.get('name') for n in self.robot.findall('link')]
        joints = self.robot.findall('joint')
        self.assertEqual(len(links), len(set(links)))
        self.assertEqual(len(joints), len({j.get('name') for j in joints}))
        self.assertEqual(len(joints), len(links) - 1)
        children = [j.find('child').get('link') for j in joints]
        self.assertEqual(len(children), len(set(children)))
        self.assertEqual(set(links) - set(children), {'base_link'})
        for link in links:
            # Walking every parent chain proves connectivity and absence of cycles.
            tip, visited = link, set()
            parents = {j.find('child').get('link'): j.find('parent').get('link') for j in joints}
            while tip in parents:
                self.assertNotIn(tip, visited)
                visited.add(tip)
                tip = parents[tip]
            self.assertEqual(tip, 'base_link')
        for name in ('chassis_link', 'left_tool', 'right_tool', 'pan_tilt_yaw_link',
                     'pan_tilt_pitch_link', 'd455_link', 'd455_color_optical_frame'):
            self.assertIn(name, links)
        for mesh in self.robot.findall('.//mesh'):
            filename = mesh.get('filename')
            self.assertTrue(filename.startswith('package://'))
            package, path = filename[len('package://'):].split('/', 1)
            self.assertTrue((PACKAGES[package] / path).is_file(), mesh.get('filename'))

    def test_arm_kinematics_match_control_model(self):
        rng = np.random.default_rng(42)
        limits = [self.control.find("joint[@name='joint{}']/limit".format(i)) for i in range(1, 7)]
        for _ in range(25):
            q = {f'joint{i}': rng.uniform(float(limit.get('lower')), float(limit.get('upper')))
                 for i, limit in enumerate(limits, 1)}
            expected = fk(self.control, 'tool', q)
            for side in ('left', 'right'):
                base = fk(self.robot, side + '_link00', {})
                full_q = {side + '_' + name: value for name, value in q.items()}
                actual = np.linalg.inv(base) @ fk(self.robot, side + '_tool', full_q)
                np.testing.assert_allclose(actual, expected, atol=1e-12)
        for side in ('left', 'right'):
            for i in range(1, 7):
                display = self.robot.find("joint[@name='{}_joint{}']".format(side, i))
                control = self.control.find("joint[@name='joint{}']".format(i))
                self.assertEqual(display.find('axis').attrib, control.find('axis').attrib)
                for bound in ('lower', 'upper'):
                    self.assertAlmostEqual(float(display.find('limit').get(bound)),
                                           float(control.find('limit').get(bound)))

    def test_official_groups_named_states_and_tcp_frames(self):
        srdf = ET.parse(PACKAGES['unidingo_rviz_test'] / 'config/dual_arm.srdf').getroot()
        self.assertEqual(srdf.get('name'), self.robot.get('name'))
        for side in ('left', 'right'):
            group = side + '_arm'
            chain = srdf.find("group[@name='{}']/chain".format(group))
            self.assertEqual(chain.attrib, {'base_link': side + '_link00', 'tip_link': side + '_tool'})
            home = srdf.find("group_state[@name='home'][@group='{}']".format(group))
            values = {j.get('name'): float(j.get('value')) for j in home}
            self.assertEqual([values[side + '_joint' + str(i)] for i in range(1, 7)],
                             [0, 0, 0, -.07435, 0, 0])
            nominal = {name: value for name, value in values.items()}
            base = fk(self.robot, side + '_link00', {})
            np.testing.assert_allclose(np.linalg.inv(base) @ fk(self.robot, side + '_tool', nominal),
                fk(self.control, 'tool', {'joint' + str(i): values[side + '_joint' + str(i)] for i in range(1, 7)}), atol=1e-12)

    def test_official_velocity_limits_match_control_and_display(self):
        # Unitree unitree_ros 8ebecd88, xacro/const.xacro: velocityMax = 3.1415 rad/s.
        for i in range(1, 7):
            control = self.control.find("joint[@name='joint{}']/limit".format(i))
            self.assertEqual(float(control.get('velocity')), 3.1415)
            for side in ('left', 'right'):
                display = self.robot.find("joint[@name='{}_joint{}']/limit".format(side, i))
                for field in ('lower', 'upper', 'velocity', 'effort'):
                    self.assertEqual(float(display.get(field)), float(control.get(field)))

    def test_original_gripper_inertials_replace_the_point_payload(self):
        # Official gripper at jointGripper = 0, reduced to the six arm coordinates.
        mass = 0.
        for link in ('gripperStator', 'gripperMover'):
            body = self.control.find("link[@name='{}']/inertial".format(link))
            expected = self.robot.find("link[@name='left_{}']/inertial".format(link))
            for tag in ('origin', 'mass', 'inertia'):
                for field, value in expected.find(tag).attrib.items():
                    np.testing.assert_allclose(np.fromstring(body.find(tag).get(field), sep=' '),
                                               np.fromstring(value, sep=' '), atol=1e-12)
            mass += float(body.find('mass').get('value'))
        self.assertAlmostEqual(mass, .80224957)
        self.assertIsNone(self.control.find("link[@name='tool']/inertial"))
        closed = self.control.find("joint[@name='gripper_closed_mount']")
        self.assertEqual(closed.get('type'), 'fixed')
        self.assertEqual(closed.find('child').get('link'), 'gripperMover')

    def test_mount_and_tcp_offsets(self):
        for side, y in (('left', 0.11708), ('right', -0.11708)):
            base = fk(self.robot, side + '_link00', {})
            np.testing.assert_allclose(base[:3, 3], [0.15091, y, 0.57228])
            np.testing.assert_allclose(base[:3, :3], np.eye(3))
            tool = self.robot.find("joint[@name='{}_tool_mount']".format(side))
            np.testing.assert_allclose(origin(tool)[:3, 3], [0.051, 0, 0])

    def test_optional_parts_still_expand(self):
        for pan in ('true', 'false'):
            for gripper in ('true', 'false'):
                robot = expand(PanTilt=pan, UnitreeGripper=gripper)
                links = {n.get('name') for n in robot.findall('link')}
                self.assertIn('left_tool', links)
                self.assertIn('right_tool', links)
                self.assertEqual('d455_link' in links, pan == 'true')
                self.assertEqual('left_gripperMover' in links, gripper == 'true')

    def test_optical_axis_and_driver_frame(self):
        joint = self.robot.find("joint[@name='d455_color_optical_joint']")
        np.testing.assert_allclose(origin(joint)[:3, :3] @ [0, 0, 1], [1, 0, 0], atol=1e-12)
        config = (ROOT / 'src/d455_camera/src/camera_config.h').read_text()
        self.assertIn('"d455_color_optical_frame"', config)


if __name__ == '__main__':
    unittest.main()
