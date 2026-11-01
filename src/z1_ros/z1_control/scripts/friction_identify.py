#!/usr/bin/env python3
"""Sequential friction identification; shared acquisition lives in the package."""
import rospy
from z1_control.friction_experiment import Experiment

def main():
    rospy.init_node('friction_identification')
    experiment = Experiment(rospy.get_param('~'))
    experiment.run()
    # Keep the launch and its controllers alive after results or a controlled stop.
    rospy.spin()


if __name__ == '__main__':
    main()
