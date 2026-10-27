#!/usr/bin/env python3
"""Start optional standalone arm models using the YAML-loaded ROS parameter.

roslaunch evaluates XML conditions before loading parameters onto the server.
Read the switch after startup instead, and keep the child launch under the
same namespace and process supervision as this helper.
"""
from pathlib import Path
import xml.etree.ElementTree as ET


def main():
    import roslaunch
    import rospkg
    import rospy

    rospy.init_node('arm_description')
    enabled = rospy.get_param('publish_description', False)
    if not isinstance(enabled, bool):
        raise ValueError('publish_description must be a YAML boolean (true or false)')
    if not enabled:
        rospy.loginfo('Standalone arm descriptions are disabled')
        return

    description_file = Path(rospkg.RosPack().get_path('z1_bringup')) / 'launch/descriptions.launch'
    launch = ET.Element('launch')
    group = ET.SubElement(launch, 'group', ns=rospy.get_namespace())
    ET.SubElement(group, 'include', file=str(description_file))
    parent = roslaunch.parent.ROSLaunchParent(
        rospy.get_param('/run_id'), [], roslaunch_strs=[ET.tostring(launch, encoding='unicode')])
    try:
        parent.start()
        parent.spin()
    finally:
        parent.shutdown()


if __name__ == '__main__':
    main()
