#! /usr/bin/env python

import rospy

import actionlib

from std_msgs.msg import Header
from control_msgs.msg    import FollowJointTrajectoryGoal
from control_msgs.msg    import FollowJointTrajectoryResult
from control_msgs.msg    import FollowJointTrajectoryFeedback
from control_msgs.msg    import FollowJointTrajectoryAction
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from math import pi

deg_to_rad = pi/180.

class VehicleActionServer(object):
    # create messages that are used to publish feedback/result
    _feedback = FollowJointTrajectoryFeedback()
    _result = FollowJointTrajectoryResult()

    def __init__(self, name):
        self._action_name = '/seabotix/alpha/vehicle_controller/follow_joint_trajectory'
        self._node_name = name
        self._action_server = actionlib.SimpleActionServer(self._action_name, FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start=False)
        self._action_server.start()
      
        # Publish to joint controllers
        self.pub = rospy.Publisher('/seabotix/cmd_pose', PoseStamped, queue_size=10)

    def execute_cb(self, goal):
        joint_names = goal.trajectory.joint_names
        trajectory_points = goal.trajectory.points
        rospy.loginfo("Vehicle Controller Received Goal")
        # rospy.logdebug(goal)
        # Start at t=0
        start_time = rospy.Duration()

        joint_to_index = {}
        for i, joint in enumerate(joint_names):
            joint_to_index[joint] = i
        # Path Execution
        # Skip the first point because it is the starting location

        for i in range(len(trajectory_points)):
            point = trajectory_points[i]
            goal_pose = point.positions
            time_from_start = point.time_from_start
            header = Header()
            header.stamp = rospy.Time.now()

            position = Point()
            position.x = goal_pose[joint_to_index["x_displacement"]]
            position.y = goal_pose[joint_to_index["y_displacement"]]
            position.z = goal_pose[joint_to_index["z_displacement"]]
            yaw_angle_deg = goal_pose[joint_to_index["yaw_displacement"]]
            orientation = Quaternion(0, 0, yaw_angle_deg*deg_to_rad, 0)
            pose = Pose(position, orientation)

            pose_stamped_msg = PoseStamped(header, pose)
            self.pub.publish(pose_stamped_msg)

            rospy.sleep(time_from_start - start_time)
            start_time += time_from_start
        self._action_server.set_succeeded()

        
if __name__ == '__main__':
    rospy.init_node('vehicle_controller')
    server = VehicleActionServer(rospy.get_name())
    rospy.spin()