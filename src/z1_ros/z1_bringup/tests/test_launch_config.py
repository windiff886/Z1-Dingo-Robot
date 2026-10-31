#!/usr/bin/env python3
"""Offline checks of the dual-arm entry point and YAML parameter placement."""
import ipaddress
import importlib.util
from pathlib import Path
import sys
from types import ModuleType
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

import yaml

BRINGUP = Path(__file__).resolve().parents[1]


class LaunchConfiguration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.launch = ET.parse(BRINGUP / 'launch/arm.launch').getroot()
        cls.controllers = ET.parse(BRINGUP / 'launch/controllers.launch').getroot()
        cls.descriptions = ET.parse(BRINGUP / 'launch/descriptions.launch').getroot()
        cls.communication = yaml.safe_load((BRINGUP / 'config/communication.yaml').read_text())
        cls.control = yaml.safe_load((BRINGUP / 'config/control.yaml').read_text())

    def test_default_launch_contains_both_controllers(self):
        self.assertEqual(self.launch.find('include').get('file'),
                         '$(find z1_bringup)/launch/controllers.launch')
        controller_groups = {group.get('ns'): group for group in self.controllers.findall('group')}
        self.assertEqual(set(controller_groups), {'left_arm', 'right_arm'})
        for side in ('left', 'right'):
            group = controller_groups[side + '_arm']
            controllers = group.findall("node[@type='z1_topic_node']")
            self.assertEqual(len(controllers), 1)
            node = controllers[0]
            self.assertEqual(node.get('name'), 'controller')
            self.assertIsNone(group.get('if'))
            self.assertIsNone(node.get('if'))
            self.assertEqual(node.find("param[@name='joint_prefix']").get('value'), side + '_')
            self.assertEqual(node.find('rosparam').get('file'), '$(find z1_bringup)/config/control.yaml')

    def test_all_transport_values_come_from_yaml(self):
        transport = {'mcu_ip', 'local_port', 'receive_budget_us'}
        self.assertEqual(self.controllers.find('rosparam').get('file'),
                         '$(find z1_bringup)/config/communication.yaml')
        self.assertFalse(transport & {param.get('name') for param in self.controllers.iter('param')})
        self.assertFalse(transport & set(self.control))
        destinations, ports = set(), set()
        for side in ('left', 'right'):
            parameters = self.communication[side + '_arm']['controller']
            self.assertEqual(set(parameters), transport)
            address = str(ipaddress.IPv4Address(parameters['mcu_ip']))
            self.assertNotIn(address, destinations)
            destinations.add(address)
            self.assertTrue(0 < parameters['local_port'] <= 65535)
            self.assertNotIn(parameters['local_port'], ports)
            ports.add(parameters['local_port'])
            self.assertGreater(parameters['receive_budget_us'], 0)
            self.assertLess(parameters['receive_budget_us'] * 1e-6, self.control['period'])

    def test_description_publishers_share_the_switch(self):
        helper = self.launch.find("node[@type='publish_description.py']")
        self.assertIsNotNone(helper)
        self.assertEqual(helper.get('pkg'), 'z1_bringup')
        self.assertIs(self.communication['publish_description'], False)
        for side in ('left', 'right'):
            publisher_group = self.descriptions.find(f"group[@ns='{side}_arm']")
            self.assertEqual(publisher_group.find('node').get('pkg'), 'robot_state_publisher')
            self.assertIn('prefix:=' + side + '_',
                          publisher_group.find('param').get('command'))

    def test_rviz_uses_same_controller_entry(self):
        path = BRINGUP.parent / 'unidingo_rviz_test/launch/rviz_tcp_test.launch'
        launch = ET.parse(path).getroot()
        self.assertFalse(launch.findall('arg'))
        include = launch.find('include')
        self.assertEqual(include.get('file'), '$(find z1_bringup)/launch/controllers.launch')
        self.assertIsNone(include.get('if'))
        self.assertIsNone(include.get('unless'))
        self.assertFalse(include.findall('arg'))
        sender = launch.find("node[@name='rviz_tcp_test']")
        self.assertEqual(sender.find('rosparam').get('file'),
                         '$(find unidingo_rviz_test)/config/rviz_tcp_test.yaml')
        rviz = launch.find("node[@name='rviz']")
        self.assertIsNone(rviz.get('if'))
        self.assertIsNone(rviz.get('unless'))

    def test_whole_robot_never_starts_standalone_publishers(self):
        # Check the actual include closure: enabling the standalone YAML switch
        # must not add a second TF authority to the whole-robot launch.
        entry = BRINGUP.parent / 'unidingo_rviz_test/launch/rviz_tcp_test.launch'
        nodes = []

        def collect(path):
            root = ET.parse(path).getroot()
            nodes.extend(root.iter('node'))
            for include in root.iter('include'):
                package, relative = include.get('file')[len('$(find '):].split(')/', 1)
                self.assertIn(package, ('z1_bringup', 'unidingo_rviz_test'))
                collect(BRINGUP.parent / package / relative)

        collect(entry)
        self.assertEqual(sum(node.get('pkg') == 'robot_state_publisher' for node in nodes), 1)
        self.assertFalse(any(node.get('type') == 'publish_description.py' for node in nodes))
        self.assertEqual(sum(node.get('type') == 'z1_topic_node' for node in nodes), 2)

    def test_launch_files_have_no_command_line_configuration_or_eval(self):
        paths = list((BRINGUP / 'launch').glob('*.launch'))
        paths.append(BRINGUP.parent / 'unidingo_rviz_test/launch/rviz_tcp_test.launch')
        for path in paths:
            with self.subTest(path=path.name):
                root = ET.parse(path).getroot()
                self.assertFalse(list(root.iter('arg')))
                self.assertNotIn('$(eval', path.read_text())


class DescriptionStartup(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location(
            'publish_description', BRINGUP / 'scripts/publish_description.py')
        cls.helper = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.helper)

    def run_helper(self, parameters, namespace='/', start_error=None):
        self.rospy = ModuleType('rospy')
        self.rospy.init_node = Mock()
        self.rospy.get_param = Mock(side_effect=lambda name, default=None: parameters.get(name, default))
        self.rospy.get_namespace = Mock(return_value=namespace)
        self.rospy.loginfo = Mock()
        self.rospkg = ModuleType('rospkg')
        self.rospkg.RosPack = Mock()
        self.rospkg.RosPack.return_value.get_path.return_value = str(BRINGUP)
        self.roslaunch = ModuleType('roslaunch')
        self.roslaunch.parent = Mock()
        self.parent = self.roslaunch.parent.ROSLaunchParent.return_value
        self.parent.start.side_effect = start_error
        with patch.dict(sys.modules, rospy=self.rospy, rospkg=self.rospkg, roslaunch=self.roslaunch):
            self.helper.main()

    def test_false_or_missing_switch_loads_no_models_and_starts_no_publishers(self):
        for parameters in ({}, {'publish_description': False}):
            with self.subTest(parameters=parameters):
                self.run_helper(parameters)
                self.rospkg.RosPack.assert_not_called()
                self.roslaunch.parent.ROSLaunchParent.assert_not_called()

    def test_true_starts_both_descriptions_in_the_entry_namespace(self):
        for namespace in ('/', '/robot/'):
            with self.subTest(namespace=namespace):
                self.run_helper({'publish_description': True, '/run_id': 'test-run'}, namespace)
                args, kwargs = self.roslaunch.parent.ROSLaunchParent.call_args
                self.assertEqual(args, ('test-run', []))
                group = ET.fromstring(kwargs['roslaunch_strs'][0]).find('group')
                self.assertEqual(group.get('ns'), namespace)
                self.assertEqual(group.find('include').get('file'),
                                 str(BRINGUP / 'launch/descriptions.launch'))
                self.parent.start.assert_called_once_with()
                self.parent.spin.assert_called_once_with()
                self.parent.shutdown.assert_called_once_with()

    def test_non_boolean_switch_fails_before_any_model_or_process_is_started(self):
        for value in ('false', 'true', 0, 1, None):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, 'YAML boolean'):
                    self.run_helper({'publish_description': value})
                self.rospkg.RosPack.assert_not_called()
                self.roslaunch.parent.ROSLaunchParent.assert_not_called()

    def test_start_failure_still_shuts_down_child_launch(self):
        with self.assertRaisesRegex(RuntimeError, 'startup failed'):
            self.run_helper({'publish_description': True, '/run_id': 'test-run'},
                            start_error=RuntimeError('startup failed'))
        self.parent.shutdown.assert_called_once_with()


if __name__ == '__main__':
    unittest.main()
