#!/usr/bin/env python
import math
import tf
import numpy as np
from scipy import linalg
import yaml
from include2py import *
import wandb

PI = 3.14159265359
dt =0.0001
abs_gravity_z = 9.81

odom = Odometry()
mav = Mav()
curr_state = State()
des_state = State()

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

final_out=np.array([0.0,0.0,0.0,0.0])
final_state = None

global r_inp
global no_rotors
global inp_r
global rotors
global count
count =0

def reset_waypoint():

    global int_position, int_angle, prev_err_pos, prev_err_ang, prev_euler_angle, rotor_speed
    prev_err_pos = np.array([0.0, 0.0, 0.00]).reshape(3,1)
    prev_err_ang = np.array([0.0, 0.0, 0.00]).reshape(3,1)
    prev_euler_angle = np.array([0.0, 0.0, 0.00]).reshape(3,1)
    rotor_speed = np.array([0.0, 0.0, 0.0, 0.0]).reshape(4,1)

def get_euler_angles_from_quaternion(quaternion_msg):
    """
    Converts a quaternion to euler angles
    param
    quaternion_msg: Quaternion message
    return:
    Euler angles in radians

    """
    rpy  = tf.transformations.euler_from_quaternion(quaternion_msg)
    roll = rpy[0]
    pitch = rpy[1]
    yaw = rpy[2]
    
    return np.array([roll, pitch, yaw]).reshape(3,1)

def fix_euler(cur_ang,prev_ang):
    """
    This function is used to fix the euler angle discontinuity
    param
    cur_ang: current euler angle
    prev_ang: previous euler angle
    return:
    fixed euler angle
    
    """
    result = np.zeros((3,1))
    for i in range(3):
        if cur_ang[i] * prev_ang[i] <0 and getShortestYawDistance(cur_ang[i],prev_ang[i]) != (cur_ang[i] - prev_ang[i]):
            import pdb;pdb.set_trace()
            diff = abs(getShortestYawDistance(cur_ang[i],prev_ang[i]))
            result[i] = prev_ang[i] + diff * (-1)**(int(prev_ang[i]<0))
        else:
            result[i] = cur_ang[i]
    return result

def odometry_callback(msg):
    """
    This function is used to update the current state of the drone
    param
    msg: Current state of the drone from the vision pipeline
    return:
    None
    """
    global curr_state
    global mav
    global wRb
    global prev_euler_angle

    pos_b = np.array([[msg[0]],
                      [msg[1]],
                      [msg[2]]]).reshape(3,1)

    eul_b = np.array([msg[3],msg[4],msg[5]]).reshape(3,1)   #get_euler_angles_from_quaternion(quat_b) ##made own function of this

    vel_b = np.array([[msg[6]],
                      [msg[7]],
                      [msg[8]]]).reshape(3,1)
    omg_b = np.array([[msg[9]],
                      [msg[10]],
                      [msg[11]]]).reshape(3,1)

    curr_state.position = pos_b
    curr_state.velocity = np.matmul(wRb(eul_b[0], eul_b[1], eul_b[2]).reshape(3,3) ,vel_b).reshape(3,1) #for velocity we take dot product and for position we take vector product
    curr_state.euler_angle = fix_euler(eul_b,prev_euler_angle)
    prev_euler_angle = curr_state.euler_angle
    curr_state.angular_velocity = omg_b

def getShortestYawDistance(yaw1, yaw2):
    """
    This function is used to calculate the shortest yaw distance between two angles and fix the discontinuity
    param
    yaw1: first angle; current parameters
    yaw2: second angle; previous parameters
    return:
    shortest yaw distance

    """
    yaw_mod = math.fmod(yaw1 - yaw2, 2*math.pi)
    if yaw_mod < -math.pi:
      yaw_mod += 2 * math.pi
    elif yaw_mod > math.pi:
      yaw_mod -= 2 * math.pi

    return yaw_mod

def calc_error_all_angles(err_angle,prev_err_ang):
    """
    This function is used to calculate the error in all the angles
    param
    err_angle: current error in angles
    prev_err_ang: previous error in angles
    return:
    error in all the angles
    
    """
    result = np.array([0,0,0],dtype = np.float64).reshape((3, 1))
    for i in range(3):
        result[i] = getShortestYawDistance(err_angle[i],prev_err_ang[i])
    return result

def u2theta(U):
    """
    This function is used to convert the control input apart from throttle, i.e. angular acceleration in x,y,z to the euler angles by integration
    param
    U: control input
    return:
    desired euler angles

    """
    global dt,curr_state,final_out
    acc=np.array([U[1]/mav.parameters.inertia.xx,U[2]/mav.parameters.inertia.yy,U[3]/mav.parameters.inertia.zz])
    omega=curr_state.angular_velocity + (dt*acc)
    final_out[0]=U[0]
    final_out[1]= curr_state.euler_angle[0] + (dt*omega[0])
    final_out[2]= curr_state.euler_angle[1] + (dt*omega[1])
    final_out[3]= omega[2]

    return final_out


def pid():
    """
    This function is used to implement the PID controller
    param
    None`
    return:
    U_temp: control input

    """
    # print("pid called")
    global curr_state, des_state,dt
    

    global Kp_pos
    global Kd_pos
    global Ki_pos
    global Kp_ang
    global Kd_ang
    global Ki_ang

    global int_position,int_angle,prev_err_ang,prev_err_pos,prev_euler_angle,final_state
    config= yaml.load(open('config.yaml','r'), Loader=yaml.FullLoader)
    
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

            if temp_count == 3:
                count +=1
                final_state = np.array(config[f'state_{count}']).reshape(3,1)
                reset_waypoint()
    
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
    err_position = (des_state.position - curr_state.position).reshape(3,1)
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

    wandb.log({"des_ang_x": des_state.euler_angle[0],"des_ang_y": des_state.euler_angle[1],"des_ang_z": des_state.euler_angle[2]})

    # Orientation error and angular velocity error
    err_angle = (des_state.euler_angle - curr_state.euler_angle).reshape(3,1)

    # wandb.log({"des_ang_z": des_state.euler_angle[2], "cur_ang_z": curr_state.euler_angle[2],})

    

    wandb.log({"err_angle_x": err_angle[0], "err_angle_y": err_angle[1], "err_angle_z": err_angle[2]})

    # err_angular_velocity = (des_state.angular_velocity - curr_state.angular_velocity).reshape(3,1)
    # wandb.log({"err_angular_velocity_x": err_angular_velocity[0], "err_angular_velocity_y": err_angular_velocity[1], "err_angular_velocity_z": err_angular_velocity[2]})
    
    err_diff_ang = calc_error_all_angles(err_angle,prev_err_ang)
    prev_err_ang = err_angle
    wandb.log({"err_diff_ang_x":err_diff_ang[0],"err_diff_ang_y":err_diff_ang[1],"err_diff_ang_z":err_diff_ang[2]})
   
    
    for i in range(3):
        int_angle[i] = max(-MAX_INT[i+3],min(MAX_INT[i+3],int_angle[i] + err_angle[i] * dt))
    wandb.log({"err_int_ang_x":int_angle[0],"err_int_ang_y":int_angle[1],"err_int_ang_z":int_angle[2]})


    ang_dd = err_angle * Kp_ang + err_diff_ang * Kd_ang  + int_angle * Ki_ang #adding integral term
    wandb.log({"ang_dd_x":ang_dd[0],"ang_dd_y":ang_dd[1],"ang_dd_z":ang_dd[2]})

    # Compute total thrust
    u1 = mav.parameters.mass * (abs_gravity_z + r_dd[2])/(np.cos(des_state.euler_angle[0]) * np.cos(des_state.euler_angle[1]))
    
    I = np.array([[mav.parameters.inertia.xx, 0, 0],
                  [0, mav.parameters.inertia.yy, 0],
                  [0, 0, mav.parameters.inertia.zz]])
    u2 = np.matmul(I,ang_dd)
    u2*=config['scale_u2']

    U_temp = np.array([u1,-u2[0],-u2[1],u2[2]]) #inverted u2_x
    wandb.log({"u1":u1,"u2_x":U_temp[1],"u2_y":U_temp[2],"u2_z":U_temp[3]})

    return U_temp

def mapped_actuator_commands(U):
    """
    Maps the actuator commands to the range of 1350 to 1650 for the roll, pitch and yaw and 1200 to 1800 for the thrust
    param
    U: control input
    return
    U: mapped control input

    """
    temp=U
    config= yaml.load(open('config.yaml','r'), Loader=yaml.FullLoader)
    U[0]=max(1200,min(1800,temp[0]*(1500)/(mav.parameters.mass*abs_gravity_z)))
    U[1]=max(1350,min(1650,temp[1]*(180/PI)*config['MAP_RP'] +1500))#need to map these out
    U[2]=max(1350,min(1650,temp[2]*(180/PI)*config['MAP_RP'] +1500))
    temp[3]*=(180/PI)
    if(temp[3]>=0 and temp[3]<41.79213684):
        U[3]=max(1350,min(1650,temp[3]*config['MAP_YR_1'] +1500))
    elif(temp[3]>=41.79213684):
        U[3]=max(1350,min(1650,temp[3]*config['MAP_YR_2'] +1500))
    elif(temp[3]<0 and temp[3]>=-41.79213684):
        U[3]=max(1350,min(1650,-temp[3]*config['MAP_YR_1'] +1500))
    elif(temp[3]<-41.79213684):
        U[3]=max(1350,min(1650,-temp[3]*config['MAP_YR_2'] +1500))
    return U


def main():
    """
    Main function
    param
    None
    return
    U: RC commands
    
    """
    global final_state
    global Kp_pos, Kd_pos, Ki_pos, Kp_ang, Kd_ang, Ki_ang

    initialise_params(abs_gravity_z)

    # InitialiseRotorMatrix()

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

    input_u = pid()
    input_u=u2theta(input_u)
    U=mapped_actuator_commands(input_u)

    # rs = InputToRotorSpeed(input_u)
    # for i in range(mav.parameters.no_rotors):
    #     if rs[i] > 838:
    #         rs[i] = 838
    #     elif math.isnan(rs[i]):
    #         rs[i] = 0
    # rotor_omega = [np.round(rs[i],4) for i in range(mav.parameters.no_rotors)]

    # return rotor_omega
    return U

main()