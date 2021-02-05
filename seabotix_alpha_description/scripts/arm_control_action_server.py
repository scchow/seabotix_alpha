#! /usr/bin/env python

import rospy

import actionlib

from std_msgs.msg import Header
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
        self._action_name = '/seabotix/alpha/arm_controller/follow_joint_trajectory'
        self._node_name = name
        self._action_server = actionlib.SimpleActionServer(self._action_name, FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._action_server.start()

        # Publish to joint controllers
        self.pub = rospy.Publisher('/seabotix/alpha/arm_control/command', JointState, queue_size=10)
      
    def execute_cb(self, goal):
        joint_names = goal.trajectory.joint_names
        trajectory_points = goal.trajectory.points
        rospy.loginfo("Arm Controller Received Goal")

        # Start at t=0
        start_time = rospy.Duration()

        # Path Execution
        for i in range(len(trajectory_points)):
            point = trajectory_points[i]
            position = point.positions
            velocity = point.velocities
            acceleration = point.accelerations
            effort = point.effort
            time_from_start = point.time_from_start
            header = Header()
            header.stamp = rospy.Time.now()

            joint_state_msg = JointState(header, joint_names, position, velocity, effort)
            self.pub.publish(joint_state_msg)

            rospy.sleep(time_from_start - start_time)
            start_time += time_from_start


        self._action_server.set_succeeded()
        
        
if __name__ == '__main__':
    rospy.init_node('arm_controller')
    server = FollowJointTrajectoryActionServer(rospy.get_name())
    rospy.spin()