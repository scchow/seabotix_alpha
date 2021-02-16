#!/usr/bin/env python
# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# File directly from uuv_manipulators_control/scripts/joint_position_controller.py
import os
import rospy
from copy import deepcopy
from uuv_manipulator_interfaces import ArmInterface
from PID import PIDRegulator
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class JointPositionController:
    def __init__(self):
        # Initializing arm interface
        self._arm_interface = ArmInterface()

        # PID controllers
        self._controllers = dict()
        # Current reference for each joint
        self._reference_pos = dict()
        # Output command topics
        self._command_topics = dict()
        # Axes mapping
        self._axes = dict()
        # Axes gain values
        self._axes_gain = dict()

        # arm topic subscriber
        self._arm_sub = rospy.Subscriber('arm_control/command', JointState, self._joint_command_cb)

        # Reading the controller configuration
        controller_config = rospy.get_param('~controller_config')
        for joint in self._arm_interface.joint_names:
            for tag in controller_config:
                if tag in joint:
                    try:
                        # Read the controller parameters
                        self._controllers[joint] = PIDRegulator(controller_config[tag]['controller']['p'],
                                                                controller_config[tag]['controller']['i'],
                                                                controller_config[tag]['controller']['d'],
                                                                1000)
                        self._command_topics[joint] = rospy.Publisher(
                            controller_config[tag]['topic'],
                            Float64,
                            queue_size=1)

                        # Setting the starting reference to the home position
                        # in the robot parameters file
                        self._reference_pos[joint] = deepcopy(self._arm_interface.home[joint])
                    except:
                        raise rospy.ROSException('Error while trying to setup controller for joint <%s>' % joint)

        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            pos = self._arm_interface.joint_angles
            for joint in pos:
                u = self._controllers[joint].regulate(self._reference_pos[joint] - pos[joint], rospy.get_time())
                self._command_topics[joint].publish(Float64(u))
            rate.sleep()

    def _check_joint_limits(self, value, joint):
        """Check the joint position and maintain it within joint limits"""
        output = value
        if self._arm_interface.joint_limits[joint] is not None:
            if output < self._arm_interface.joint_limits[joint]['lower']:
                output = self._arm_interface.joint_limits[joint]['lower']
            if output > self._arm_interface.joint_limits[joint]['upper']:
                output = self._arm_interface.joint_limits[joint]['upper']
        return output

    def _joint_command_cb(self, msg):
        try:
            # Parse the JointState input to set the joint angle reference
            for i, joint in enumerate(msg.name):
                if joint in self._arm_interface.joint_names:
                    self._reference_pos[joint] = msg.position[i]
                    # Check for the joint limits
                    # self._reference_pos[joint] = self._check_joint_limits(self._reference_pos[joint], joint)
                else:
                    error_str = "Joint Controller received trajectory with goal for joint %s but the only controllable joint are: " % joint
                    error_str += str(self._arm_interface.joint_names)
                    rospy.logwarn(error_str)                    
        except Exception, e:
            print 'Error during joy parsing, message=', e

if __name__ == '__main__':
    # Start the node
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    rospy.loginfo('Starting [%s] node' % node_name)

    controller = JointPositionController()

    rospy.spin()
    rospy.loginfo('Shuting down [%s] node' % node_name)
