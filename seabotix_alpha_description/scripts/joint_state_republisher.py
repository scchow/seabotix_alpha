#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
from sensor_msgs.msg import JointState
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
        self.subscriber = rospy.Subscriber('/seabotix/joint_states', JointState, self.received_joint_state)

        # create a publisher
        self.publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)

        self.rate = rospy.Rate(10.0)
        self.joints_to_publish = ['alpha/joint1', 'alpha/joint2', 'alpha/joint3', 'alpha/joint4', 'alpha/joint5','alpha/joint6']

    def received_joint_state(self, msg):
        indices_to_keep = []
        # TODO: Figure out how to find position, velocity, and effort of our vehicle
        name = ['x_displacement', 'y_displacement', 'z_displacement', 'yaw_displacement']
        position = [0 , 0 , 0 , 0]
        velocity = [0 , 0 , 0 , 0]
        effort = [0 , 0 , 0 , 0]
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