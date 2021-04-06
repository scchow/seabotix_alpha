# This file contains helper functions to convert from quaternion to Euler angle
# https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles


from math import pi, sin, cos, atan2, asin, copysign

def quaternion_to_rpy(w, x, y, z):

    roll = quaternion_to_roll(w, x, y, z)

    # Pitch
    pitch = quaternion_to_pitch(w, x, y, z)

    # Yaw    
    yaw = quaternion_to_yaw(w, x, y, z)

    return roll, pitch, yaw

def quaternion_to_roll(w, x, y, z):

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    return atan2(sinr_cosp, cosr_cosp)

def quaternion_to_pitch(w, x, y, z):

    sinp = 2 * (w * y - z * x)
    if (abs(sinp) >= 1):
        return copysign(pi / 2.0, sinp); # use 90 degrees if out of range
    else:
        return asin(sinp)
    
def quaternion_to_yaw(w, x, y, z):

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    return atan2(siny_cosp, cosy_cosp)

def rpy_to_quaterion(roll, pitch, yaw):

    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return w, x, y, z