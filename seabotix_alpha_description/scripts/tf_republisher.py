#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
import copy

"""
The TF rebroadcaster class takes in the transform from the world
to the vehicle (base_link) and rebroadcasts 4 transforms:
world_link_x, world_link_y, world_link_z, world_link_yaw,
where world_link_x, contains identity transform from the origin,
world_link_y, contains the x displacement,
world_link_z, contains the x+y displacement,
and world_link_yaw, contains the x+y+z displacement

"""
class TFRebroadcaster(object):

    def __init__(self, node_name):

        # Create the node
        rospy.init_node(node_name)

        # create a listener on the tf tree
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # create a tf broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        self.rate = rospy.Rate(10.0)
        self.run()    

    def run(self):
        while not rospy.is_shutdown():
            try:
                trans = self.tfBuffer.lookup_transform('world', 'seabotix/base_link', rospy.Time())
                
                # This deep copy might not be necessary
                t = copy.deepcopy(trans)

                # transform 
                t.header.frame_id = "world"
                t.child_frame_id = "world_link_yaw"
                # world_link_yaw is before the yaw (or any rotational transforms are applied)
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0
                self.br.sendTransform(t)

                # world_link_z is before the z translation is applied
                t.child_frame_id = "world_link_z"
                t.transform.translation.z = 0.0
                self.br.sendTransform(t)

                # world_link_y is before the y translation is applied
                t.child_frame_id = "world_link_y"
                t.transform.translation.y = 0.0
                self.br.sendTransform(t)

                # world_link_x is before the x translation is applied
                # should be identity transform from "world" frame
                t.child_frame_id = "world_link_x"
                t.transform.translation.x = 0.0
                self.br.sendTransform(t)            
            
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue

if __name__ == '__main__':
    rebroadcaster = TFRebroadcaster("world_frame_rebroadcaster")
    rospy.spin()