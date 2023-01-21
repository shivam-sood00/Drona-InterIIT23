import numpy as np
import yaml
from include2py import *
config= yaml.load(open('config.yaml','r'), Loader=yaml.FullLoader)

# PID parameters
Kp = config['Kp_pos']
Ki = config['Ki_pos']
Kd = config['Kd_pos']

curr_state = State()

def states(msg):
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
    # curr_state.euler_angle = fix_euler(eul_b,prev_euler_angle)
    # prev_euler_angle = curr_state.euler_angle
    # curr_state.angular_velocity = omg_b

def PID():
    U = np.zeros(4)
    U[0] = 1500 + Kp*()