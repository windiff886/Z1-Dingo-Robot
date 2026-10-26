#!/usr/bin/env python3
"""Offline checks of the dual-arm entry point and YAML parameter placement."""
import ipaddress
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml

BRINGUP = Path(__file__).resolve().parents[1]


class LaunchConfiguration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.launch = ET.parse(BRINGUP / 'launch/arm.launch').getroot()
        cls.communication = yaml.safe_load((BRINGUP / 'config/communication.yaml').read_text())
        cls.control = yaml.safe_load((BRINGUP / 'config/control.yaml').read_text())

    def test_default_launch_contains_both_controllers(self):
        groups = {group.get('ns'): group for group in self.launch.findall('group')}
        self.assertEqual(set(groups), {'left_arm', 'right_arm'})
        for side in ('left', 'right'):
            group = groups[side + '_arm']
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
        args = {arg.get('name'): arg.get('default') for arg in self.launch.findall('arg')}
        self.assertEqual(args['config'], '$(find z1_bringup)/config/communication.yaml')
        self.assertEqual(self.launch.find('rosparam').get('file'), '$(arg config)')
        self.assertFalse(transport & {param.get('name') for param in self.launch.iter('param')})
        self.assertFalse(transport & set(args))
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
            self.assertLess(parameters['receive_budget_us'] * 1e-6
                            + self.control['lookahead_budget'], self.control['period'])

    def test_description_publishers_share_the_switch(self):
        for group in self.launch.findall('group'):
            publisher_group = group.find('group')
            self.assertEqual(publisher_group.get('if'), '$(arg publish_description)')
            self.assertEqual(publisher_group.find('node').get('pkg'), 'robot_state_publisher')
            self.assertIn('prefix:=' + group.get('ns').split('_')[0] + '_',
                          publisher_group.find('param').get('command'))

    def test_rviz_uses_same_controller_entry(self):
        path = BRINGUP.parent / 'unidingo_rviz_test/launch/rviz_tcp_test.launch'
        launch = ET.parse(path).getroot()
        self.assertFalse(launch.findall('arg'))
        include = launch.find('include')
        self.assertEqual(include.get('file'), '$(find z1_bringup)/launch/arm.launch')
        self.assertIsNone(include.get('if'))
        self.assertIsNone(include.get('unless'))
        self.assertEqual(include.find("arg[@name='publish_description']").get('value'), 'false')
        self.assertIsNone(include.find("arg[@name='config']"))
        sender = launch.find("node[@name='rviz_tcp_test']")
        self.assertEqual(sender.find('rosparam').get('file'),
                         '$(find unidingo_rviz_test)/config/rviz_tcp_test.yaml')
        rviz = launch.find("node[@name='rviz']")
        self.assertIsNone(rviz.get('if'))
        self.assertIsNone(rviz.get('unless'))


if __name__ == '__main__':
    unittest.main()
