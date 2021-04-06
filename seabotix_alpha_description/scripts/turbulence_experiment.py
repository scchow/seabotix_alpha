#!/usr/bin/env python  

import os
import errno
import time

import rospy
from uuv_world_ros_plugins_msgs.srv import SetCurrentVelocity
# Because of transformations
# import tf_conversions

# import tf2_ros
# from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg	 import JointState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import GetLinkState

# import copy
from math import pi, cos, sin, atan2
from datetime import datetime
from quaternions import quaternion_to_yaw, quaternion_to_rpy, rpy_to_quaterion

"""
This script instantiates a listener on robot end effector pose
and a publisher to the ocean current action server and a pose to the vehicle pose controller
The ocean current is dynamically adjusted as a function of time using a sin wave.
This script records the  
"""

class TurbulenceExperimentNode(object):

    def __init__(self, node_name):

        # Period of sinusoidal turbulence
        self.period = 5

        # Omega - angular frequency
        self.w = 2*pi/self.period

        # Maximum current speed 
        self.max_vel = 0.0 

        # Delay Time: Time to wait before starting currents
        self.delay_time = 5.0

        # Create log files
        self.instantiate_logs()

        # Create the node
        rospy.init_node(node_name)

        # wait for current velocity service to come up
        self.current_velocity_service = '/hydrodynamics/set_current_velocity'
        rospy.wait_for_service(self.current_velocity_service)

        # create service proxy
        try:
            self.set_current_velocity = rospy.ServiceProxy(self.current_velocity_service, SetCurrentVelocity)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            raise

        # wait for model_link state service to come up
        self.model_link_state_service = '/gazebo/get_link_state'
        rospy.wait_for_service(self.model_link_state_service)

        # create service proxy
        try:
            self.get_model_state = rospy.ServiceProxy(self.model_link_state_service, GetLinkState)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            raise

        # create a subscriber on the vehicle pose topic
        # self.subscriber = rospy.Subscriber('/seabotix/pose_gt', Odometry, self.odom_callback)

        # Publish to vehicle pose controllers
        self.pub_vehicle_pose = rospy.Publisher('/seabotix/cmd_pose', PoseStamped, queue_size=10)

        # Publish to arm state controller
        self.pub_arm_pose = rospy.Publisher('/seabotix/alpha/arm_control/command', JointState, queue_size=10)

        # The current_function reads in vehicle pose using self.position
        # and returns a tuple (velocity, horizontal_angle, vertical_angle)
        # dictating the parameters of the current at that pose
        # Change this to a different function depending on your needs
        self.current_function = self.sinusoid_current

        self.current_info = (0,0,0) # variable to save current state in format [velocity, horizontal angle, vertical angle]

        rospy.sleep(1)

        self.rate = rospy.Rate(10.0)
        self.start_time = self.get_time() # time in seconds

        while not rospy.is_shutdown():
            self.loop()
            self.rate.sleep()



    def __del__(self):
        rospy.loginfo("Ending turbulence experiment node, Setting velocities back to 0.")
        self.set_current_velocity(0.0, 0.0, 0.0)
        close(self.log_file_desc)
        

    @classmethod
    def get_time(cls):
        """
            Returns the ROS time in seconds
        """
        now = rospy.Time.now()
        return now.secs + now.nsecs*(10**-9) # time in seconds

    def loop(self):
        """ 
        """

        # Get end effector pose
        x, y, z, roll, pitch, yaw = self.get_end_effector_state()

        # log: time, current_state, end effector pose
        self.log_file_desc.write("\n{}, {}, {}, {}, {}, {}, {}, {}, {}, {}".format(self.get_time(), 
        self.current_info[0], self.current_info[1], self.current_info[2], 
        x, y, z, roll, pitch, yaw))

        # Publish goal vehicle and arm poses
        self.pub_goal_arm_pose()
        self.pub_goal_vehicle_pose()

        # Publish current pose
        self.current_info = self.current_function()
        self.set_current_velocity(*self.current_info)

    def instantiate_logs(self):
        """ Creates the log directories for the runs and
            saves initial run info
        """

        # Log file
        timestamp = datetime.now().strftime("%Y-%m-%dT%H%M%S")
        self.log_dir = os.path.join("home", "developer", "uuv_ws", "experiment_logs", timestamp)

        # Create Log directory if it does not exist
        try:
            os.makedirs(self.log_dir)
        except OSError as exception:
            if exception.errno != errno.EEXIST:
                raise

        self.info_file = os.path.join(self.log_dir, "run_info.txt")
        self.log_file = os.path.join(self.log_dir, "data.csv")

        with open(self.info_file, "w+") as f:
            f.write("Period = {}\nMaxVel = {}".format(self.period, self.max_vel))

        self.log_file_desc = open(self.log_file, "w+")
        self.log_file_desc.write("t, current_vel, current_h_angle, current_v_angle, x, y, z, roll, pitch, yaw")

    def pub_goal_vehicle_pose(self):
        """ Sends the goal vehicle pose to the simulator
        """

        header = Header()
        header.stamp = rospy.Time.now()

        position = Point(20.5, -10, -85)
        yaw = 0

        # Converting yaw to quaternion
        # See https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        # For better intuition about quaternions: https://eater.net/quaternions
        orientation = Quaternion(*rpy_to_quaterion(0, 0, yaw))
        pose = Pose(position, orientation)

        pose_stamped_msg = PoseStamped(header, pose)
        self.pub_vehicle_pose.publish(pose_stamped_msg)

    def pub_goal_arm_pose(self):
        current_time = self.get_time()
        time_to_goal = 20.0
        scale = min(1.0, current_time / time_to_goal)

        header = Header()
        header.stamp = rospy.Time.now()
        joint_names = ['alpha/joint1', 'alpha/joint2', 'alpha/joint3', 'alpha/joint4']
        position = [0.0, scale*pi/2, scale*pi/2, 0.0]
        velocity = [0, 0, 0, 0]
        effort = [0, 0, 0, 0]
        joint_state_msg = JointState(header, joint_names, position, velocity, effort)
        self.pub_arm_pose.publish(joint_state_msg)

    def get_end_effector_state(self):
        response = self.get_model_state('seabotix::alpha/wrist_link', 'world')
        if response.success:
            state = response.link_state
            position = state.pose.position
            orient = state.pose.orientation
            roll, pitch, yaw = quaternion_to_rpy(orient.w, orient.x, orient.y, orient.z)

            # state also contains velocities, but unused for now
            # state.twist.linear, state.twist.angular

            return position.x, position.y, position.z, roll, pitch, yaw


    def update_current(self):
        """ Sends a command to the ros service to update ocean currents
            based on vehicle position
        """
        velocity, horizontal_angle, vertical_angle = self.current_function()
        self.set_current_velocity(velocity, horizontal_angle, vertical_angle)

    def sinusoid_current(self):
        """ A simple time-varying current field in the x direction based on
            a sinusoid.
        """

        t = self.get_time() # time in seconds
        offset_time = t - self.start_time - self.delay_time # time offset by start/delay time

        # Don't start the currents until t > start + 10
        if t < self.start_time + self.delay_time:
            print("Waiting for delay time to pass: {}/{}".format(t, self.start_time + self.delay_time))
            velocity = 0
        else:
            velocity = self.max_vel * sin(self.w*offset_time) # current velocity in m/s

        vertical_angle = 0.0 # vertical angle in radians

        # current velocity must be positive, so change horizontal angle
        # such that if velocity < 0, push vehicle to left, else right
        horizontal_angle = pi if velocity < 0 else 0
        
        speed = abs(velocity)

        return speed, horizontal_angle, vertical_angle           


if __name__ == '__main__':
    republisher = TurbulenceExperimentNode("TurbulenceExperimentNode")
    rospy.spin()