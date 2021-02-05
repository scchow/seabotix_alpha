#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import copy

"""
The joint state republisher class gets joint states from seabotix/joint_states
and republishes them to a format suitable for MoveIt!

"""
class JointStateRepublisher(object):

    def __init__(self, node_name):

        # Create the node
        rospy.init_node(node_name)

        # create a subscriber on the joint angles topic
        self.subscriber = rospy.Subscriber('/seabotix/joint_states', JointState, self.joint_state_callback)

        # create a subscriber on the joint angles topic
        self.subscriber = rospy.Subscriber('/seabotix/pose_gt', Odometry, self.odom_callback)

        # create a publisher
        self.publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)

        self.rate = rospy.Rate(10.0)
        self.joints_to_publish = ['alpha/joint1', 'alpha/joint2', 'alpha/joint3', 'alpha/joint4', 'alpha/joint5','alpha/joint6']

        self.position = [0, 0, 0, 0]
        self.velocity = [0, 0, 0, 0]
        self.acceleration = [0, 0, 0, 0]

    def odom_callback(self, msg):
        pose = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        pose_vel = msg.twist.twist.linear
        orientation_vel = msg.twist.twist.angular
        self.position = [pose.x, pose.y, pose.z, orientation.z]
        self.velocity = [pose_vel.x, pose_vel.y, pose_vel.z, orientation_vel.z]

    def joint_state_callback(self, msg):
        indices_to_keep = []
        # TODO: Add roll pitch
        name = ['x_displacement', 'y_displacement', 'z_displacement', 'yaw_displacement']

        position = copy.deepcopy(self.position)
        velocity = copy.deepcopy(self.velocity)
        effort = copy.deepcopy(self.acceleration)

        for i, joint_name in enumerate(msg.name):
            if joint_name in self.joints_to_publish:
                name.append(msg.name[i])
                position.append(msg.position[i])
                velocity.append(msg.velocity[i])
                effort.append(msg.effort[i])

        # Create a new JointState message keeping the attributes of the original
        joint_states = copy.deepcopy(msg)

        # overwrite message information with minimized information
        joint_states.name = name
        joint_states.position = position
        joint_states.velocity = velocity
        joint_states.effort = effort

        # republish message to topic MoveIt! expects it to be in
        self.publisher.publish(joint_states)
        

if __name__ == '__main__':
    republisher = JointStateRepublisher("joint_state_republisher")
    rospy.spin()