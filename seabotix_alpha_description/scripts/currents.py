#!/usr/bin/env python  

import rospy
from uuv_world_ros_plugins_msgs.srv import SetCurrentVelocity
# Because of transformations
# import tf_conversions

# import tf2_ros
# from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
# import copy
from math import pi, atan2

"""
This script instantiates a listener on robot position
and a publisher to the ocean current action server
depending on the location of the robot, the ocean current
is dynamically adjusted, thus simulating changing ocean currents
"""

class DynamicCurrentNode(object):

    def __init__(self, node_name):

        # Create the node
        rospy.init_node(node_name)

        # wait for service to come up
        self.current_velocity_service = '/hydrodynamics/set_current_velocity'
        rospy.wait_for_service(self.current_velocity_service)

        # create service proxy
        try:
            self.set_current_velocity = rospy.ServiceProxy(self.current_velocity_service, SetCurrentVelocity)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        # create a subscriber on the vehicle pose topic
        self.subscriber = rospy.Subscriber('/seabotix/pose_gt', Odometry, self.odom_callback)

        self.rate = rospy.Rate(10.0)

        # The current_function reads in vehicle pose using self.position
        # and returns a tuple (velocity, horizontal_angle, vertical_angle)
        # dictating the parameters of the current at that pose
        # Change this to a different function depending on your needs
        self.current_function = self.simple_z_mapping

    def __del__(self):
        rospy.rospy.loginfo("Ending current node, Setting velocities back to 0.")
        self.set_current_velocity(0.0, 0.0, 0.0)
        

    def odom_callback(self, msg):
        pose = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        pose_vel = msg.twist.twist.linear
        orientation_vel = msg.twist.twist.angular

        # Convert from quaternion to Euler angle
        # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        siny_cosp = 2*(orient.w * orient.z + orient.x*orient.y)
        cosy_cosp = 1-2*(orient.y**2 + orient.z**2)
        yaw = atan2(siny_cosp, cosy_cosp)

        self.position = [pose.x, pose.y, pose.z, yaw]
        self.velocity = [pose_vel.x, pose_vel.y, pose_vel.z, orientation_vel.z]

        self.update_current()

    def update_current(self):
        """ Sends a command to the ros service to update ocean currents
            based on vehicle position
        """
        velocity, horizontal_angle, vertical_angle = self.current_function()
        self.set_current_velocity(velocity, horizontal_angle, vertical_angle)

    def simple_x_mapping(self):
        """
            Very simple dummy current field that pushes vehicle towards x = 20m
        """

        velocity = 0.0 # current velocity in m/s
        horizontal_angle = 0.0 # horizontal angle in radians
        vertical_angle = 0.0 # vertical angle in radians

        # current velocity must be positive, so change horizontal angle
        if self.position[0] > 20:
            horizontal_angle = pi # push vehicle to left
        else:
            horizontal_angle = 0 # push vehicle to right
        
        # scale current based on difference between x=20 and current pose
        max_vel = 1.5
        velocity = min(max_vel, max_vel*abs(self.position[0] - 20.0))

        return velocity, horizontal_angle, vertical_angle      

    def simple_z_mapping(self):
        """
            Very simple dummy current field that pushes vehicle towards the surface z = 0
        """
        # very simple dummy current field that pushes vehicle towards x=20

        horizontal_angle = 0.0 # horizontal angle in radians
        vertical_angle = pi/2.0 # vertical angle in radians - up towards surface

        # scale current based on difference between x=20 and current pose
        max_vel = 1.5
        velocity = min(max_vel, max_vel*abs(self.position[2]))

        return velocity, horizontal_angle, vertical_angle   

if __name__ == '__main__':
    republisher = DynamicCurrentNode("dynamic_current_node")
    rospy.spin()