#!/usr/bin/env python

import rospy
from kiba_control.kinematics_planner import KinematicsPlanner

if __name__ == '__main__':
    try:
        rospy.init_node('kinematics_planner')
        my_planner = KinematicsPlanner('/follow_joint_trajectory', rospy.Duration(2.0))
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr("{}".format(e))
