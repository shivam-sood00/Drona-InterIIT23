#!/usr/bin/env python
import rospy
import numpy as np
import math
import sys
import mav_msgs.msg
# from mav_msgs.srv import Empty
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from trajectory_msgs.msg import MultiDOFJointTrajectory



class Rotors:
    def __init__(self):
        self.rotor_angle = 1.0
        self.arm_length = 0.17
        self.force_constant = 8.54858e-06
        self.moment_constant = 1.6e-2
        self.direction = 1
        # direction of 1 implies rotor rotates counter-clockwise

class Inertia:
    def __init__(self):
        self.xx = 0.007
        self.yy = 0.007
        self.zz = 0.012

class Parameters:
    def __init__(self):
        self.name = "hummingbird"
        self.mass = 0.716
        self.inertia = Inertia()
        self.rotors = [Rotors() for _ in range(6)]
        self.no_rotors = 4

class Mav:
    def __init__(self):
        self.parameters = Parameters()
        self.inp_to_rotor_matrix = np.zeros((4, 4))

class State:
    def __init__(self):
        # initialise the values to some defaults
        self.position = np.zeros((3,1),dtype=np.float64)
        self.velocity = np.zeros((3,1))
        self.acceleration = np.zeros((3,1))
        self.jerk = np.zeros((3,1))
        self.snap = np.zeros((3,1))

        self.euler_angle = np.zeros((3,1))
        self.angular_velocity = np.zeros((3,1))
def s(x):
    return math.sin(x)
def c(x):
    return math.cos(x)


def wRb(roll, pitch, yaw):
    R = np.zeros((3, 3))

    # Rx = np.array([[c(roll),-s(roll),0],[s(roll),c(roll),0],[0,0,1]])
    # Ry = np.array([[c(pitch),0,s(pitch)],[0,1,0],[-s(pitch),0,c(pitch)]])
    # Rz = np.array([[1,0,0],[0,c(yaw),-s(yaw)],[0,s(yaw),c(yaw)]])
    # R = np.dot(Rz,np.dot(Ry,Rx)) #correct order of multiplication oif roll->pitch->yaw
    # R = math.cos(yaw)*math.cos(pitch) - math.sin(roll)*math.sin(yaw)*math.sin(pitch),
    # -math.cos(roll)*math.sin(yaw),
    # math.cos(yaw)*math.sin(pitch) + math.cos(pitch)*math.sin(roll)*math.sin(yaw),
    # math.sin(yaw)*math.cos(pitch) + math.sin(roll)*math.cos(yaw)*math.sin(pitch),
    # math.cos(roll)*math.cos(yaw),
    # math.sin(yaw)*math.sin(pitch) - math.cos(pitch)*math.sin(roll)*math.cos(yaw),
    # -math.cos(roll)*math.sin(pitch),
    # math.sin(roll),
    # math.cos(roll)*math.cos(pitch)
    R = np.array([[math.cos(yaw)*math.cos(pitch) - math.sin(roll)*math.sin(yaw)*math.sin(pitch),
    -math.cos(roll)*math.sin(yaw),
    math.cos(yaw)*math.sin(pitch) + math.cos(pitch)*math.sin(roll)*math.sin(yaw)],
    [math.sin(yaw)*math.cos(pitch) + math.sin(roll)*math.cos(yaw)*math.sin(pitch),
    math.cos(roll)*math.cos(yaw),
    math.sin(yaw)*math.sin(pitch) - math.cos(pitch)*math.sin(roll)*math.cos(yaw)],
    [-math.cos(roll)*math.sin(pitch),
    math.sin(roll),
    math.cos(roll)*math.cos(pitch)]])

    return R

def bRw(roll, pitch, yaw):
    R = np.zeros((3, 3))
    R=np.transpose(wRb(roll, pitch, yaw))
    return R
def omega2thetadot(roll,pitch):
    R=np.array([[1,0,-s(pitch)],[0,c(roll),s(roll)*c(pitch)],[0,-s(roll), c(roll)*c(pitch)]])
    return R
mav = Mav()

def initialise_params(gravity_z):
    rospy.get_param("/mavName", mav.parameters.name)
    rospy.get_param("/{}/mass".format(mav.parameters.name), mav.parameters.mass)
    rospy.get_param("/{}/inertia/xx".format(mav.parameters.name), mav.parameters.inertia.xx)
    rospy.get_param("/{}/inertia/yy".format(mav.parameters.name), mav.parameters.inertia.yy)
    rospy.get_param("/{}/inertia/zz".format(mav.parameters.name), mav.parameters.inertia.zz)

    for i in range(6):
        prefix = "/{}/rotor_configuration/".format(mav.parameters.name)
        no = str(i)

        _angle = "{}{}/angle".format(prefix, no)
        _arm_length = "{}{}/arm_length".format(prefix, no)
        _direction = "{}{}/direction".format(prefix, no)
        _fc = "{}{}/rotor_force_constant".format(prefix, no)
        _mc = "{}{}/rotor_moment_constant".format(prefix, no)

        if rospy.get_param(_angle, mav.parameters.rotors[i].rotor_angle):
            mav.parameters.no_rotors = 6
        else:
            mav.parameters.no_rotors = 4
            break
        rospy.get_param(_arm_length, mav.parameters.rotors[i].arm_length)
        rospy.get_param(_direction, mav.parameters.rotors[i].direction)
        rospy.get_param(_fc, mav.parameters.rotors[i].force_constant)
        rospy.get_param(_mc, mav.parameters.rotors[i].moment_constant)

    rospy.get_param("/gazebo/gravity_z", gravity_z)
