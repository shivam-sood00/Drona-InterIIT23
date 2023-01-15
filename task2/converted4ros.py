#!/usr/bin/env python
'''
    TODO:
    Check frames in which odometry publishes data
    In InitialiseRotorMatrix, check the fourth row of the matrix (gamma vs Km)
    #clipping in the integral part 
    tunning

'''
import math
import tf
import rospy
import numpy as np
import numpy.matlib
from scipy import linalg
import std_msgs.msg
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
# from mav_msgs import Trajectory
from trajectory_msgs.msg import MultiDOFJointTrajectory
import geometry_msgs.msg
from pyquaternion import Quaternion
from trajectory_msgs.msg import MultiDOFJointTrajectory
from collections import namedtuple
import string
import wandb
import yaml

PI = 3.14159265359
dt =0.0001
# wandb.init(project="test-project", entity="interiit007")

from include2py import *

abs_gravity_z = 9.81

odom = Odometry()
mav = Mav()
curr_state = State()
des_state = State()
prev_state = State()

# PID gains for position control
# Kp_pos = np.array([0.35, 0.35, 0.35]).reshape(3,1) #should be less than 1 kp for z
# Kd_pos = np.array([0, 0, 0.1]).reshape(3,1)
# Ki_pos = np.array([0, 0, 0]).reshape(3,1)
# # PID gains for angle control
# Kp_ang = np.array([0.0, 0.0, 0.35]).reshape(3,1)
# Kd_ang = np.array([0, 0, 10]).reshape(3,1)
# Ki_ang = np.array([0, 0, 0]).reshape(3,1)

Kp_pos = None
Kd_pos = None
Ki_pos = None
# PID gains for angle control
Kp_ang = None
Kd_ang = None
Ki_ang = None

int_position = np.array([0.0, 0.0, 0.00]).reshape(3,1)
int_angle = np.array([0.0, 0.0, 0.00]).reshape(3,1)
prev_err_pos = np.array([0.0, 0.0, 0.00]).reshape(3,1)
prev_err_ang = np.array([0.0, 0.0, 0.00]).reshape(3,1)
prev_euler_angle = np.array([0.0, 0.0, 0.00]).reshape(3,1)
rotor_speed = np.array([0.0, 0.0, 0.0, 0.0]).reshape(4,1)

final_state = None

global r_inp
global no_rotors
global inp_r
global rotors
global count
count =0

# odom.pose.pose.orientation.x = 2.0
# odom.pose.pose.orientation.y = 2.0
# odom.pose.pose.orientation.z = 2.0
# odom.pose.pose.position.x = 2.0
# odom.pose.pose.position.y = 1.0
# odom.pose.pose.position.z = 3.0

# odom.twist.twist.linear.x = 1.0
# odom.twist.twist.linear.y = 1.0
# odom.twist.twist.linear.z = 1.0

# odom.twist.twist.angular.x = 3.0
# odom.twist.twist.angular.y = 1.0
# odom.twist.twist.angular.z = 2.0

def reset_waypoint():
    global int_position, int_angle, prev_err_pos, prev_err_ang, prev_euler_angle, rotor_speed
    # int_position = np.array([0.0, 0.0, 0.00]).reshape(3,1)
    # int_angle = np.array([0.0, 0.0, 0.00]).reshape(3,1)
    prev_err_pos = np.array([0.0, 0.0, 0.00]).reshape(3,1)
    prev_err_ang = np.array([0.0, 0.0, 0.00]).reshape(3,1)
    prev_euler_angle = np.array([0.0, 0.0, 0.00]).reshape(3,1)
    rotor_speed = np.array([0.0, 0.0, 0.0, 0.0]).reshape(4,1)

def get_euler_angles_from_quaternion(quaternion_msg):

    # quat = [quaternion_msg.x, quaternion_msg.y, quaternion_msg.z, quaternion_msg.w]
    
    rpy  = tf.transformations.euler_from_quaternion(quaternion_msg)
    # print("rpy", rpy.shape)
    roll = rpy[0]
    pitch = rpy[1]
    yaw = rpy[2]
    
    # sgf
    # roll = [0]
    # pitch = tf.transformations.euler_from_quaternion(quaternion_msg)[1]
    # yaw = tf.transformations.euler_from_quaternion(quaternion_msg)[2]
    return np.array([roll, pitch, yaw]).reshape(3,1)

def fix_euler(cur_ang,prev_ang):
    result = np.zeros((3,1))
    for i in range(3):
        # wandb.log({f"short_yaw_{i}": getShortestYawDistance(cur_ang[i],prev_ang[i])})
        if cur_ang[i] * prev_ang[i] <0 and getShortestYawDistance(cur_ang[i],prev_ang[i]) != (cur_ang[i] - prev_ang[i]):
            import pdb;pdb.set_trace()
            diff = abs(getShortestYawDistance(cur_ang[i],prev_ang[i]))
            result[i] = prev_ang[i] + diff * (-1)**(int(prev_ang[i]<0))
        else:
            result[i] = cur_ang[i]
    return result

def odometry_callback(msg):
    # print("odometry_callback")
    global curr_state
    global mav
    global wRb
    global prev_euler_angle

    pos_b = np.array([[msg.pose.pose.position.x],
                      [msg.pose.pose.position.y],
                      [msg.pose.pose.position.z]]).reshape(3,1)
    quat_b = np.array([msg.pose.pose.orientation.x,
                       msg.pose.pose.orientation.y,
                       msg.pose.pose.orientation.z,
                       msg.pose.pose.orientation.w])
    vel_b = np.array([[msg.twist.twist.linear.x],
                      [msg.twist.twist.linear.y],
                      [msg.twist.twist.linear.z]]).reshape(3,1)
    omg_b = np.array([[msg.twist.twist.angular.x],
                      [msg.twist.twist.angular.y],
                      [msg.twist.twist.angular.z]]).reshape(3,1)
    eul_b = np.zeros((3, 1))

    
    eul_b = get_euler_angles_from_quaternion(quat_b) ##made own function of this

    # wandb.log({"og_cur_ang_x": eul_b[0],"og_cur_ang_y": eul_b[1],"og_cur_ang_z": eul_b[2]})

    curr_state.position = pos_b
    # print("curr_state.position",curr_state.position)

    '''
    Check!! Currently its world-to-body frame
    '''
    curr_state.velocity = np.matmul(wRb(eul_b[0], eul_b[1], eul_b[2]).reshape(3,3) ,vel_b).reshape(3,1) #for velocity we take dot product and for position we take vector product
    # curr_state.velocity = vel_b
    # print("curr_state.velocity",curr_state.velocity)
    # curr_state.euler_angle = eul_b
    curr_state.euler_angle = fix_euler(eul_b,prev_euler_angle)
    prev_euler_angle = curr_state.euler_angle
    # wandb.log({"fixed_cur_ang_x": curr_state.euler_angle[0],"fixed_cur_ang_y": curr_state.euler_angle[1],"fixed_cur_ang_z": curr_state.euler_angle[2]})
    # print("eul_b",eul_b)
    curr_state.angular_velocity = omg_b
    # curr_state.angular_velocity = omg_b

    # print("omg_b",omg_b)



def trajectory_callback(msg):
    global des_state
    global mav
    global wRb


    des_posW = np.array([[msg.points.transforms.translation.x],
                           [msg.points.transforms.translation.y],
                           [msg.points.transforms.translation.z]]).reshape(3,1)
    des_velW = np.array([[msg.points.velocities.linear.x],
                           [msg.points.velocitie.linear.y],
                           [msg.points.velocities.linear.z]]).reshape(3,1)
    des_accW = np.array([[msg.points.accelerations.linear.x],
                           [msg.points.accelerations.linear.y],
                           [msg.points.accelerations.linear.z]]).reshape(3,1)
    des_jerkW = np.array([[msg.points.jerks.linear.x],
                            [msg.points.jerks.linear.x],
                            [msg.points.jerks.linear.x]]).reshape(3,1)
    des_snapW = np.array([[msg.points.snap.linear.x],[msg.points.snaps.linear.y],[msg.points.snaps.linear.z]])
    des_quatW = np.array([[msg.points.transforms.rotation.w],[msg.points.transforms.rotation.x],[msg.points.transforms.rotation.y],[msg.points.transforms.rotation.z]])

    des_omgW = np.array([[msg.points.velocities.angular.x],[msg.points.velocities.angular.y],[msg.points.velocities.angular.z]])

    des_state.position = des_posW
    # print(des_posW,"des_posW")
    des_state.velocity =  des_velW
    # print(des_velW,"des_velW")
    des_state.acceleration = des_accW
    # print(des_accW,"des_accW")
    des_state.jerk = des_jerkW

    des_state.euler_angle =  get_euler_angles_from_quaternion(des_quatW)
    # print(des_eulW,"des_eulW")
    des_state.angularVelocity = des_omgW
    # print(des_omgW,"des_omgW")

def InitialiseRotorMatrix():

    rotors = mav.parameters.rotors  # get a reference to the rotors array
    no_rotors = mav.parameters.no_rotors

    # create the input-to-rotor matrix
    inp_r = np.empty((no_rotors, 4))
    r_inp = np.empty((4, no_rotors))
    # print(r_inp)

    Force_constant = rotors[0].force_constant
    Arm_length = rotors[0].arm_length
    for i in range(no_rotors):
        r = rotors[i]  # get a reference to the i-th Rotor object
        r_inp[0, i] = r.force_constant
        # r_inp[1,i] = r.arm_length *math.sin(r.rotor_angle)
        # r_inp[2,i] = -1.0 * r.arm_length * math.cos(r.rotor_angle)
        r_inp[3, i] = 1 * math.pow(-1,i)*r.direction * r.moment_constant
    
    r_inp[1, 0] = Arm_length* math.sin(PI*(1/2)*0.0)*Force_constant
    r_inp[1, 1] = -Arm_length* math.sin(PI*(1/2)*1.0)*Force_constant
    r_inp[1, 2] = Arm_length* math.sin(PI*(1/2)*2.0)*Force_constant
    r_inp[1, 3] = -Arm_length* math.sin(PI*(1/2)*-1.0)*Force_constant

    r_inp[2, 0] = 1.0 * Arm_length* math.cos(PI*(1/2)*0)*Force_constant
    r_inp[2, 1] = -1.0 * Arm_length* math.cos(PI*(1/2)*1)*Force_constant
    r_inp[2, 2] = 1.0 * Arm_length* math.cos(PI*(1/2)*2)*Force_constant
    r_inp[2, 3] = -1.0 * Arm_length* math.cos(PI*(1/2)*-1)*Force_constant

    # print("r_inp", r_inp.shape)

    # calculate the pseudo-inverse of the input-to-rotor matrix
    inp_r = ((r_inp.T)@ linalg.inv(np.matmul(r_inp,r_inp.transpose()))) #check2
    # inp_r = linalg.inv(r_inp)
    
    # print(inp_r)

    # store the matrix in the mav object
    mav.inp_to_rotor_matrix = inp_r
    # print(" 1st", mav.inp_to_rotor_matrix)
    # print(mav.inp_to_rotor_matrix.shape)

def InputToRotorSpeed(U):
    global rotor_speed
    # print("U",U)
    # print("mav.inp_to_rotor_matrix_size",mav.inp_to_rotor_matrix.shape)
    # print("mav.inp_to_rotor_matrix",mav.inp_to_rotor_matrix)

    """
    Check motor directions
    """
    # rotor_speed2 = np.empty((mav.parameters.no_rotors, 1))
    # rotor_speed = np.empty((mav.parameters.no_rotors, 1))
    # # print(" 2nd", mav.inp_to_rotor_matrix)

    # rotor_speed2 = mav.inp_to_rotor_matrix @ U ##Check 1
    # print(rotor_speed2,"rotor_speed2")
    # rotor_speed[0] = np.sqrt(abs(rotor_speed2[0]))
    # rotor_speed[1] = np.sqrt(abs(rotor_speed2[1]))
    # rotor_speed[2] = np.sqrt(abs(rotor_speed2[2]))
    # rotor_speed[3] = np.sqrt(abs(rotor_speed2[3]))


    
    rotor_speed = np.sqrt(mav.inp_to_rotor_matrix @ U)
    # import pdb; pdb.set_trace()
    # print(rotor_speed.shape,"rotor_speed_shape")

    wandb.log({"rotor_speed_1":rotor_speed[0],"rotor_speed_2":rotor_speed[1],"rotor_speed_3":rotor_speed[2],"rotor_speed_4":rotor_speed[3]})
    print(rotor_speed,"rotor_speed")
    return rotor_speed

def getShortestYawDistance(yaw1, yaw2):
    yaw_mod = math.fmod(yaw1 - yaw2, 2*math.pi)
    if yaw_mod < -math.pi:
      yaw_mod += 2 * math.pi
    elif yaw_mod > math.pi:
      yaw_mod -= 2 * math.pi

    return yaw_mod

# def calc_error_angle(err_angle,prev_err_ang,tol = dt):
#     if err_angle in ()

def calc_error_all_angles(err_angle,prev_err_ang):
    result = np.array([0,0,0],dtype = np.float64).reshape((3, 1))
    for i in range(3):
        result[i] = getShortestYawDistance(err_angle[i],prev_err_ang[i])
    return result

def pid():
    # print("pid called")
    global curr_state, des_state,dt
    # Constants
    # Kp_pos = np.array([1, 1, 1])
    # Kd_pos = np.array([0, 0, 0])
    # Ki_pos = np.array([0, 0, 0])

    # Kp_ang = np.array([1, 1, 1])
    # Kd_ang = np.array([0, 0, 0])
    # Ki_ang = np.array([0, 0, 0])
    # curr_state = State()
    

    # print("curr_state ",curr_state)
    global Kp_pos
    global Kd_pos
    global Ki_pos
    global Kp_ang
    global Kd_ang
    global Ki_ang

    global int_position,int_angle,prev_err_ang,prev_err_pos,prev_euler_angle,final_state
    config= yaml.load(open('config.yaml','r'), Loader=yaml.FullLoader)
    # curr_state.position = np.array([odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z]).reshape(3,1)
    # curr_state.velocity = np.array([odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z]).reshape(3,1)
    # curr_state.euler_angle = np.array([odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z]).reshape(3,1)
    # curr_state.angular_velocity = np.array([odom.twist.twist.angular.x,odom.twist.twist.angular.y,odom.twist.twist.angular.z]).reshape(3,1)
    # des_state.position = np.array([0.0, 0.0, 3]).reshape(3,1)
    # final_state = np.array(config['state_0']).reshape(3,1)
    err_temp = np.array([0.0, 0.0, 0.0],dtype=np.float64).reshape(3,1)
    MAX_RES = config['MAX_RES']
    err_velocity = (des_state.velocity - curr_state.velocity).reshape(3,1) ##TO GIVE WAYPOINTS
    wandb.log({"err_velocity_x":err_velocity[0],"err_velocity_y":err_velocity[1],"err_velocity_z":err_velocity[2]})
    ERR_THRESH = config['ERR_THRESH']
    global count
    temp_count = 0
    if count < config['state_num']-1: 
        for i in range(3):
            if abs(final_state[i] - curr_state.position[i]) < ERR_THRESH and abs(err_velocity[i]) < config['ERR_VEL_THRESH']:
                temp_count = temp_count + 1
            # import pdb;pdb.set_trace()
            if temp_count == 3:
                # import pdb;pdb.set_trace()
                count +=1
                final_state = np.array(config[f'state_{count}']).reshape(3,1)
                reset_waypoint()
    # final_state = np.array(config[f'state_0']).reshape(3,1)
    
    wandb.log({"final_state_x":final_state[0],"final_state_y":final_state[1],"final_state_z":final_state[2]})

    # wandb.log({"temp_count":int(temp_count == 3)})
    #clipping the velocity so that it becomes stable and then moves to different waypoint




    for i in range(3):
        if final_state[i]-curr_state.position[i] > 0:
            err_temp[i] = min(MAX_RES,final_state[i]-curr_state.position[i])
        else:
            err_temp[i] = max(-MAX_RES,final_state[i]-curr_state.position[i])
    
    des_state.position = curr_state.position + err_temp

    print("curr_state.position",curr_state.position)
    wandb.log({"curr_state_x":curr_state.position[0],"curr_state_y":curr_state.position[1],"curr_state_z":curr_state.position[2]})
    wandb.log({"curr_ang_x":curr_state.euler_angle[0],"curr_ang_y":curr_state.euler_angle[1],"curr_ang_z":curr_state.euler_angle[2]})

    print("des_state.position",des_state.position)
    print("final_state.position",final_state)
    err_position = (des_state.position - curr_state.position).reshape(3,1) #CHECK ONce
    wandb.log({"err_position_x":err_position[0],"err_position_y":err_position[1],"err_position_z":err_position[2]})
    # err_diff_pos = (err_position - prev_err_pos).reshape(3,1)
    err_diff_pos=  (des_state.velocity - curr_state.velocity).reshape(3,1)
    prev_err_pos = err_position
    wandb.log({"err_diff_pos_x":err_diff_pos[0],"err_diff_pos_y":err_diff_pos[1],"err_diff_pos_z":err_diff_pos[2]})
    des_acc = des_state.acceleration.reshape(3,1)

    MAX_INT = np.array(config['MAX_INT']).reshape(6,1)

    for i in range(3):
        int_position[i] = max(-MAX_INT[i],min(MAX_INT[i],int_position[i] + err_position[i] *dt))


    wandb.log({"err_int_pos_x":int_position[0],"err_int_pos_y":int_position[1],"err_int_pos_z":int_position[2]})

    r_dd = err_position * Kp_pos + err_diff_pos * Kd_pos  + int_position * Ki_pos
    r_dd += des_acc
    wandb.log({"r_dd_x":r_dd[0],"r_dd_y":r_dd[1],"r_dd_z":r_dd[2]})
    c = np.cos(curr_state.euler_angle[2])
    s = np.sin(curr_state.euler_angle[2])
    abs_gravity_z = 9.81  # Absolute value of gravity along the z-axis
    

    des_state.euler_angle[0] = (r_dd[0] * s - r_dd[1] * c) / (abs_gravity_z +r_dd[2])
    des_state.euler_angle[1] = (r_dd[0] * c + r_dd[1] * s) / (abs_gravity_z+ r_dd[2])
    des_state.euler_angle[2] = 0.0 

    # temp1 = np.zeros(3)
    # temp1[0] = (r_dd[0] * s - r_dd[1] * c) / (abs_gravity_z +r_dd[2])
    # temp1[1] = (r_dd[0] * c + r_dd[1] * s) / (abs_gravity_z+ r_dd[2])
    # temp1[2] = 0.0
    # des_state.euler_angle = np.matmul(bRw(temp1[0],temp1[1],temp1[2]).reshape(3,3) , temp1.reshape(3,1))
    # des_state.euler_angle[0] = 0.0
    # des_state.euler_angle[1] = 0.0
    # des_state.euler_angle[2] = 0.0
    wandb.log({"des_ang_x": des_state.euler_angle[0],"des_ang_y": des_state.euler_angle[1],"des_ang_z": des_state.euler_angle[2]})
    # des_state.euler_angle*=config['scale_angle']
    # wandb.log({"scaled_des_ang_x": des_state.euler_angle[0],"scaled_des_ang_y": des_state.euler_angle[1],"scaled_des_ang_z": des_state.euler_angle[2]})

    # Orientation error and angular velocity error
    err_angle = (des_state.euler_angle - curr_state.euler_angle).reshape(3,1)

    # wandb.log({"des_ang_z": des_state.euler_angle[2], "cur_ang_z": curr_state.euler_angle[2],})

    

    # err_angle[2] = getShortestYawDistance(des_state.euler_angle[2], curr_state.euler_angle[2])
    wandb.log({"err_angle_x": err_angle[0], "err_angle_y": err_angle[1], "err_angle_z": err_angle[2]})

    # err_angular_velocity = (des_state.angular_velocity - curr_state.angular_velocity).reshape(3,1)
    # wandb.log({"err_angular_velocity_x": err_angular_velocity[0], "err_angular_velocity_y": err_angular_velocity[1], "err_angular_velocity_z": err_angular_velocity[2]})
    
    # err_diff_ang = (err_angle - prev_err_ang).reshape(3,1)
    # temp = np.zeros(3)
    # tempA = np.zeros(3)
    err_diff_ang = calc_error_all_angles(err_angle,prev_err_ang)

    # temp = calc_error_all_angles(err_angle,prev_err_ang)
    # tempA= np.linalg.inv(omega2thetadot(curr_state.euler_angle[0],curr_state.euler_angle[1]))
    # err_diff_ang = np.matmul(tempA,temp)
    prev_err_ang = err_angle
    wandb.log({"err_diff_ang_x":err_diff_ang[0],"err_diff_ang_y":err_diff_ang[1],"err_diff_ang_z":err_diff_ang[2]})
    # thetadot = np.zeros(3)
    # thetadot = np.linalg.inv(omega2thetadot(curr_state.euler_angle[0],curr_state.euler_angle[1]))
    # import pdb;pdb.set_trace()
    # if int_angle[0]<MAX_INT[3]:
    #     int_angle[0] += err_angle[0] * dt
    # else:
    #     int_angle[0] = 0

    # if int_angle[1]<MAX_INT[4]:
    #     int_angle[1] += err_angle[1] * dt
    # else:
    #     int_angle[1] = 0

    # if int_angle[2]<MAX_INT[5]:
    #     int_angle[2] += err_angle[2] * dt
    # else:
    #     int_position[2] = 0
    # int_angle[0] += err_angle[0] * dt
    # int_angle[1] += err_angle[1] * dt
    # int_angle[2] += err_angle[2] * dt
    
    for i in range(3):
        int_angle[i] = max(-MAX_INT[i+3],min(MAX_INT[i+3],int_angle[i] + err_angle[i] * dt))
    wandb.log({"err_int_ang_x":int_angle[0],"err_int_ang_y":int_angle[1],"err_int_ang_z":int_angle[2]})


    ang_dd = err_angle * Kp_ang + err_diff_ang * Kd_ang  + int_angle * Ki_ang #adding integral term
    # ang_dd = err_angle * Kp_ang + thetadot * Kd_ang  + int_angle * Ki_ang #adding integral term
    wandb.log({"ang_dd_x":ang_dd[0],"ang_dd_y":ang_dd[1],"ang_dd_z":ang_dd[2]})
    # print(ang_dd.shape,"ang_dd")
    # print(err_angle,"err_angle")
    # print(err_angular_velocity,"err_angular_velocity")
    # print(int_angle,"ang_dd")
    # print(ang_dd,"ang_dd")

    # print(ang_dd)
    #CHANGED HERE

    # Update integral terms
    # print("gay")
    # print(err_position.shape)
    # print(int_position.shape)



    # print(int_position)
    # print(int_angle)



    # Compute total thrust
    u1 = mav.parameters.mass * (abs_gravity_z + r_dd[2])/(np.cos(des_state.euler_angle[0]) * np.cos(des_state.euler_angle[1]))
    # u1 = mav.parameters.mass * (abs_gravity_z + r_dd[2])
    # Compute moments about the x, y, and z axes
    I = np.array([[mav.parameters.inertia.xx, 0, 0],
                  [0, mav.parameters.inertia.yy, 0],
                  [0, 0, mav.parameters.inertia.zz]])
    u2 = I.dot(ang_dd)
    u2*=config['scale_u2']
    # print(I,"I")

    # U_temp = np.array([u1,ang_dd[0],ang_dd[1],ang_dd[2]])
    # U_temp = np.array([u1,u2[0],u2[1],u2[2]])
    U_temp = np.array([u1,-u2[0],-u2[1],u2[2]]) #inverted u2_x

    
    # wandb.log({"u1":u1,"ang_acc_x":ang_dd[0],"ang_acc_y":ang_dd[1],"ang_acc_z":ang_dd[2]})
    wandb.log({"u1":u1,"u2_x":U_temp[1],"u2_y":U_temp[2],"u2_z":U_temp[3]})
    
    # U_temp = np.array([u1,ang_dd[1],])

    # print(U_temp,"U_temp")
    # print(u1)

    return U_temp

def main():
    global prev_state,final_state
    global Kp_pos, Kd_pos, Ki_pos, Kp_ang, Kd_ang, Ki_ang

    rospy.init_node('converted4ros', anonymous=True)
    initialise_params(abs_gravity_z)
    InitialiseRotorMatrix()
    # exit()
    config= yaml.load(open('config.yaml','r'), Loader=yaml.FullLoader)
    final_state = np.array(config['state_0']).reshape(3,1)
    wandb.init(project="inter-iit-controls",config=config)

    Kp_pos = np.array(config['Kp_pos']).reshape(3,1) #should be less than 1 kp for z
    Kd_pos = np.array(config['Kd_pos']).reshape(3,1)
    Ki_pos = np.array(config['Ki_pos']).reshape(3,1)
    # PID gains for angle control
    Kp_ang = np.array(config['Kp_ang']).reshape(3,1)
    Kd_ang = np.array(config['Kd_ang']).reshape(3,1)
    Ki_ang = np.array(config['Ki_ang']).reshape(3,1)

    prev_state.position[2] = 1.0

    key = ["position","attitude"]
    # print(key)
    gain = ["kp","kd","ki"]
    # print(gain)

    axis = ["x","y","z"]

    # for i in range(2):
    #     for j in range(3):
    #         for k in range(3):
    #             print(key[i],gain[i])
    #             # temp = rospy.get_param("/"+mav.parameters.name+"/"+key[i]+"/"+axis[j]+"/"+gain[k])
    #             if i == 0:
    #                 if j==0:
    #                     Kp_pos[k] = posi
    #                 elif j==1:
    #                     Kd_pos[k] = temp
    #                 elif j==2:
    #                     Ki_pos[k] = temp
    #             elif i == 1:
    #                 if j==0:
    #                     Kp_ang[k] = temp
    #                 elif j==1:
    #                     Kd_ang[k] = temp
    #                 elif j==2:
    #                     Ki_ang[k] = temp

    a = rospy.Subscriber("/" + mav.parameters.name +"/odometry_sensor1/odometry", Odometry, odometry_callback, queue_size=10)
    subTraj = rospy.Subscriber("/" + mav.parameters.name +"/command/trajectory", MultiDOFJointTrajectory, trajectory_callback, queue_size=10)
    pubActuators = rospy.Publisher("/" + mav.parameters.name +"/command/motor_speed", Actuators, queue_size=10)
    
    msg = Actuators()
    while not rospy.is_shutdown():
        if subTraj.get_num_connections() == 0:
            des_state = prev_state
        else:
            prev_state = des_state
        input_u = pid()
        rs = InputToRotorSpeed(input_u)
        for i in range(mav.parameters.no_rotors):
            if rs[i] > 838:
                rs[i] = 838
            elif math.isnan(rs[i]):
                rs[i] = 0
        rotor_omega = [np.round(rs[i],4) for i in range(mav.parameters.no_rotors)]

        # print(rotor_omega,"rotors_omega")
        # msg = Actuators()
        # msg.header = Header()
        # msg.header.stamp = rospy.Time.now()
        # if not ((rotor_omega[0] == rotor_omega[1]) and  (rotor_omega[1] == rotor_omega[2])and (rotor_omega[2] == rotor_omega[3])):
        #     print(rotor_omega,"rotors_omega")
        msg.angular_velocities = rotor_omega
        pubActuators.publish(msg)
        rospy.sleep(dt)


main()