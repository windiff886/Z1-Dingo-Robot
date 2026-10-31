#!/usr/bin/env python3
"""Check the official MoveIt editor boundary and the SDK-only execution route."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import yaml

PACKAGE = Path(__file__).resolve().parents[1]


class PreviewWiring(unittest.TestCase):
    def test_sdk_and_messages_still_have_no_moveit_dependency(self):
        for package in ('z1_control','z1_msgs'):
            for file in ('package.xml','CMakeLists.txt'):
                self.assertNotIn('moveit',(PACKAGE.parent/package/file).read_text().lower())

    def test_real_controllers_and_readonly_move_group_are_separate(self):
        root=ET.parse(PACKAGE/'launch/rviz_tcp_test.launch').getroot()
        includes=[n.get('file') for n in root.findall('include')]
        self.assertEqual(includes,['$(find z1_bringup)/launch/controllers.launch','$(find unidingo_rviz_test)/launch/preview_move_group.launch'])
        scene=ET.parse(PACKAGE/'launch/preview_move_group.launch').getroot()
        node=scene.find("node[@pkg='moveit_ros_move_group']")
        self.assertEqual(node.find("param[@name='allow_trajectory_execution']").get('value'),'false')
        self.assertFalse(any(n.get('pkg') in ('z1_hw','controller_manager') for n in scene.iter('node')))
        self.assertEqual(node.find('remap').get('to'),'/rviz_tcp_test/joint_states')

    def test_moveit_owns_goal_state_ik_and_frame_placement(self):
        source=(PACKAGE/'src/sdk_motion_planning.cpp').read_text()
        self.assertIn('public moveit_rviz_plugin::MotionPlanningDisplay',source)
        self.assertIn('scheduleDrawQueryGoalState(handler, error_changed)',source)
        self.assertIn('state.setToDefaultValues(jmg, "home")',source)
        self.assertIn('trajectory_visual_->reset()',source)
        self.assertNotIn('getFrameManager()->getTransform',source)
        self.assertFalse((PACKAGE/'src/framed_trajectory.cpp').exists())
        self.assertFalse((PACKAGE/'urdf/preview_arm.urdf.xacro').exists())
        script=(PACKAGE/'scripts/interactive_tcp_test.py').read_text()
        self.assertNotIn('InteractiveMarkerServer',script)
        self.assertNotIn('DisplayRobotState',script)
        self.assertNotIn('tf2_ros',script)

    def test_display_uses_whole_robot_and_official_scene_monitor(self):
        config=yaml.safe_load((PACKAGE/'rviz/tcp_test.rviz').read_text())
        displays=config['Visualization Manager']['Displays']
        native=next(d for d in displays if d['Class']=='unidingo_rviz_test/SdkMotionPlanning')
        self.assertEqual(native['Robot Description'],'robot_description')
        self.assertEqual(native['Planning Scene Topic'],'/move_group/monitored_planning_scene')
        self.assertEqual(native['Planned Path']['Trajectory Topic'],'/rviz_tcp_test/preview_trajectory')
        self.assertFalse(native['MoveIt_Use_Constraint_Aware_IK'])
        self.assertFalse(any('FramedTrajectory' in d['Class'] for d in displays))

    def test_both_kinematics_groups_use_official_trac_ik_settings(self):
        values=yaml.safe_load((PACKAGE/'config/kinematics.yaml').read_text())
        self.assertEqual(set(values),{'left_arm','right_arm'})
        for value in values.values():
            self.assertEqual(value,{'kinematics_solver':'trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin',
                'kinematics_solver_search_resolution':.005,'kinematics_solver_timeout':.005})

    def test_official_planning_limits_are_prefixed_without_changing_values(self):
        values=yaml.safe_load((PACKAGE/'config/joint_limits.yaml').read_text())
        self.assertEqual(values['default_velocity_scaling_factor'],.3)
        self.assertEqual(values['default_acceleration_scaling_factor'],1.)
        self.assertEqual(set(values['joint_limits']),
            {side+'_joint'+str(i) for side in ('left','right') for i in range(1,7)})
        for value in values['joint_limits'].values():
            self.assertEqual(value,{'has_velocity_limits':True,'max_velocity':3.14,
                'has_acceleration_limits':True,'max_acceleration':4})

    def test_sdk_service_reads_each_real_controller_and_has_no_execution_api(self):
        root=ET.parse(PACKAGE/'launch/rviz_tcp_test.launch').getroot()
        nodes=[n for n in root.iter('node') if n.get('type')=='z1_pose_preview']
        self.assertEqual(len(nodes),2)
        for side in ('left','right'):
            node=next(n for n in nodes if n.get('name')==side+'_preview')
            self.assertEqual(node.find('param').get('value'),'/'+side+'_arm/controller')
        service=(PACKAGE/'srv/PreviewPose.srv').read_text()
        self.assertNotIn('command_id',service)
        self.assertNotIn('execute',service.lower())
        source=(PACKAGE/'src/pose_preview.cpp').read_text()
        self.assertIn('z1::planPoseGoal',source)
        self.assertNotIn('McuTransport',source)
        self.assertNotIn('home_q',source)

    def test_plugins_and_runtime_dependencies_are_declared(self):
        manifest=ET.parse(PACKAGE/'package.xml').getroot()
        self.assertEqual(manifest.find('export/rviz').get('plugin'),'${prefix}/plugin_description.xml')
        registered=ET.parse(PACKAGE/'plugin_description.xml').getroot().findall('class')
        self.assertEqual({n.get('name') for n in registered},
            {'unidingo_rviz_test/TargetPanel','unidingo_rviz_test/SdkMotionPlanning'})
        dependencies={n.text for n in manifest.findall('exec_depend')}
        self.assertTrue({'moveit_ros_move_group','moveit_planners_ompl','trac_ik_kinematics_plugin'} <= dependencies)
        cmake=(PACKAGE/'CMakeLists.txt').read_text()
        self.assertIn('src/sdk_motion_planning.cpp',cmake)
        self.assertIn('EditorTarget.msg',cmake)


if __name__=='__main__': unittest.main()
