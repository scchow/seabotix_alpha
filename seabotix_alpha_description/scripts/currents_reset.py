#!/usr/bin/env python  

import rospy
from uuv_world_ros_plugins_msgs.srv import SetCurrentVelocity

"""
This script sets the current velocity back to 0.
"""


if __name__ == '__main__':
    # wait for service to come up
    current_velocity_service = '/hydrodynamics/set_current_velocity'
    rospy.wait_for_service(current_velocity_service)

    # create service proxy
    try:
        set_current_velocity = rospy.ServiceProxy(current_velocity_service, SetCurrentVelocity)
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))

    set_current_velocity(0.0, 0.0, 0.0)