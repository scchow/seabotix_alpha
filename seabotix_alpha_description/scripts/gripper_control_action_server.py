#! /usr/bin/env python

import rospy

import actionlib

from control_msgs.msg    import FollowJointTrajectoryGoal
from control_msgs.msg    import FollowJointTrajectoryResult
from control_msgs.msg    import FollowJointTrajectoryFeedback
from control_msgs.msg    import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory
# from robot_movement_interface.msg import *
from sensor_msgs.msg	 import JointState

class FollowJointTrajectoryActionServer(object):
    # create messages that are used to publish feedback/result
    _feedback = FollowJointTrajectoryFeedback()
    _result = FollowJointTrajectoryResult()

    def __init__(self, name):
        self._action_name = '/seabotix/alpha/gripper_controller/follow_joint_trajectory'
        self._node_name = name
        self._action_server = actionlib.SimpleActionServer(self._action_name, FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._action_server.start()
      
    def execute_cb(self, goal):
        joint_names = goal.trajectory.joint_names
        trajectory_points = goal.trajectory.points
        rospy.loginfo("Got Goal")

        rospy.loginfo(goal)
        
        
if __name__ == '__main__':
    rospy.init_node('gripper_controller')
    server = FollowJointTrajectoryActionServer(rospy.get_name())
    rospy.spin()