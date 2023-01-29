#!/usr/bin/env python
import rospy
import casadi as ca
import numpy as np                                                                                                    
import time
import math 
from scipy.spatial import KDTree                                                                                       
import transform
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
import tf
from trajectory_msgs.msg import MultiDOFJointTrajectory
pi = math.pi
inf = np.inf
t_start = time.time()



"""
variable parameters 
"""

n_states = 12
n_controls = 4
N =10                                                                     # Prediction horizon(same as control horizon)
                                                                                                                
                      

U_ref = np.array([457.7,457.7,457.7,457.7], dtype ='f')     				                ############## Reference velocity and reference omega					                               		                                                                   

Q_phi =0                                                             ############## # gains to control error in x,y,theta during motion
Q_phi_dot =0
Q_theta = 1                                                                                                                                           
Q_theta_dot =0                                                                                                                                    
Q_psi = 0                                                                                                                                           
Q_psi_dot =0                                                                                                                                      
Q_z = 0
Q_x = 0
Q_Vz = 0
Q_Vx =0
Q_y = 0
Q_Vy =0

omega_cost = 0#0.1
R1 = omega_cost                                                             # gains to control magnitude of V and omega                                                                                                           
R2 = omega_cost 
R3 = omega_cost                                                                                                                                                             
R4 = omega_cost

error_allowed_in_g = 1e-100                                                 # error in contraints (should be ~ 0)

delta_T = 0.1

X_target = ca.vertcat(1 ,0, 1, 0 ,0 ,0, 1, 0, 0 ,0 ,0 ,0)

"""# parameters that depend on simulator 
"""

########################################################
m = 0.056
l = 0.065

Ixx = 6.5e-05
Iyy = 6.21e-05
Izz = 1.18e-04
Jr = 5.15e-07
b = 2.83e-08
d = 1.55e-10
gr = 9.81
########################################################

n_bound_var = n_states 

phi_bound_max = inf                  #################################   
phi_bound_min = -inf 
phi_dot_bound_max = inf                     
phi_dot_bound_min = -inf

theta_bound_max = inf                     
theta_bound_min = -inf 
theta_dot_bound_max = inf                     
theta_dot_bound_min = -inf

psi_bound_max = inf                     
psi_bound_min = -inf 
psi_dot_bound_max = inf                   
psi_dot_bound_min = -inf

x_bound_max = inf                                                           # enter x and y bounds when limited world like indoor environments                   
x_bound_min = -inf                     
y_bound_max = inf                      
y_bound_min = -inf                     
z_bound_max = inf                      
z_bound_min = -inf                     
Vx_bound_max = inf                                                           # enter x and y bounds when limited world like indoor environments                   
Vx_bound_min = -inf                     
Vy_bound_max = inf                      
Vy_bound_min = -inf                     
Vz_bound_max = inf                
Vz_bound_min = -inf                    


# omega_max = 460.0                                                                                                                                        
# omega_min = 450#-omega_max

omega_max = 450.0                                                                                                                                        
omega_min = 440#-omega_max              ###################################################



phi=0.0;phi_dot=0.0;theta=0.0;theta_dot=0.0;psi=0.0;psi_dot=0.0;x=0.0;y=0.0;z=0.0;Vx=0.0;Vy=0.0;Vz=0.0;
omega1=0.0;omega2=0.0;omega3=0.0;omega4=0.0                                                                                                                                                                       

# Global Variables made for integration


global total_path_points                                                                                                                                        
total_path_points = 0                                                                                                                                                                            
global path  



# path = np.zeros((100,3))
# i = 0
# while(des[2]>=path[i,2]):
#     path[i,:] = [0.,0.,1e-1]
#     i+=1
# path.reshape(i+1,3)

# path_resolution =  ca.norm_3(path[0,0:3] - path[1,0:3])                                                                                 


phi_casadi = ca.SX.sym('phi')
phi_dot_casadi = ca.SX.sym('phi_dot')
theta_casadi = ca.SX.sym('theta')
theta_dot_casadi = ca.SX.sym('theta_dot')
psi_casadi = ca.SX.sym('psi')
psi_dot_casadi = ca.SX.sym('psi_dot')


x_casadi =ca.SX.sym('x')
y_casadi = ca.SX.sym('y')
z_casadi = ca.SX.sym('z')
    ############### take feedback -- done

Vx_casadi =ca.SX.sym('Vx')
Vy_casadi = ca.SX.sym('Vy')
Vz_casadi = ca.SX.sym('Vz')

states =ca.vertcat(phi_casadi,phi_dot_casadi,theta_casadi,theta_dot_casadi,psi_casadi,psi_dot_casadi,z_casadi,Vz_casadi,x_casadi,Vx_casadi,y_casadi,Vy_casadi)
n_states = states.numel()           
print("STATES",n_states)
omega1_casadi =ca.SX.sym('omega1')
omega2_casadi = ca.SX.sym('omega2')
omega3_casadi =ca.SX.sym('omega3')
omega4_casadi = ca.SX.sym('omega4')
controls = ca.vertcat(omega1_casadi,omega2_casadi,omega3_casadi,omega4_casadi)
n_controls = controls.numel()       

rhs = ca.vertcat(phi_dot_casadi,
                theta_dot_casadi*psi_dot_casadi*(Iyy-Izz)/Ixx + theta_dot_casadi*(-omega2_casadi-omega4_casadi+omega1_casadi+omega3_casadi)*Jr/Ixx + b*(-(omega2_casadi**2) + (omega4_casadi**2))*l/Ixx,
                theta_dot_casadi,
                phi_dot_casadi*psi_dot_casadi*(Izz-Ixx)/Iyy - phi_dot_casadi*(-omega2_casadi-omega4_casadi+omega1_casadi+omega3_casadi)*Jr/Iyy + b*(-(omega3_casadi**2) + (omega1_casadi**2))*l/Iyy,
                psi_dot_casadi,
                theta_dot_casadi*phi_dot_casadi*(Ixx-Iyy)/Izz + d*(omega2_casadi**2+omega4_casadi**2-omega1_casadi**2-omega3_casadi**2)/Izz,
                Vz_casadi,
                gr - (ca.cos(phi_casadi))*(ca.cos(theta_casadi))*b*(omega2_casadi**2+omega4_casadi**2+omega1_casadi**2+omega3_casadi**2)/m,
                Vx_casadi,
                ((ca.cos(phi_casadi))*(ca.sin(theta_casadi))*(ca.cos(psi_casadi)) + (ca.sin(phi_casadi))*(ca.sin(psi_casadi)))*b*(omega2_casadi**2+omega4_casadi**2+omega1_casadi**2+omega3_casadi**2)/m, 
                Vy_casadi,
                ((ca.cos(phi_casadi))*(ca.sin(theta_casadi))*(ca.sin(psi_casadi)) - (ca.sin(phi_casadi))*(ca.cos(psi_casadi)))*b*(omega2_casadi**2+omega4_casadi**2+omega1_casadi**2+omega3_casadi**2)/m, 
                )

                        
f = ca.Function('f',[states,controls],[rhs])                                                            # function to predict rhs using states and controls                                                                 


U = ca.SX.sym('U', n_controls,N)                                                                        # For storing predicted controls                                                               
X =ca.SX.sym('X', n_states, N+1)                                                                        # For storing predicted states
P = ca.SX.sym('P', n_states + n_states )                                         # For storing odometry, next N path points and next N referance controls    

# obj = 0
# g = []

Q = ca.diagcat(Q_phi,Q_phi_dot,Q_theta,Q_theta_dot,Q_psi,Q_psi_dot,Q_z,Q_Vz,Q_x,Q_Vx,Q_y,Q_Vy)                                                                                                                                                  
R = ca.diagcat(R1, R2, R3, R4)    

obj = 0
g = X[:, 0] - P[:n_states] 
step_horizon_rk = 0.1
for k in range(0, N):
    state_i, control_i = X[:,k], U[:,k]
    
    obj = obj + (state_i - P[n_states:]).T @ Q @ (state_i - P[n_states:])+ (control_i - U_ref).T @ R @ (control_i - U_ref)
    st_next = X[:, k+1]
    k1 = f(state_i, control_i)
    print(control_i.numel())
    print(state_i.numel())
    print(k1.numel())
    k2 = f(state_i + step_horizon_rk/2*k1, control_i)
    k3 = f(state_i + step_horizon_rk/2*k2, control_i)
    k4 = f(state_i + step_horizon_rk * k3, control_i)
    st_next_RK4 = state_i + (step_horizon_rk / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
    g = ca.vertcat(g, st_next - st_next_RK4)  
# for i in range(0,N):                                                                                                                                                                                                                            
#     cost_pred_st = ca.mtimes(  ca.mtimes( (X[0:n_states,i] - P[n_states:].reshape((n_states,1)) ).T , Q )  ,  (X[0:n_states,i] - P[n_states:].reshape((n_states,1)) )  )  + ca.mtimes(  ca.mtimes( ( (U[0:n_controls,i]) - U_ref ).T , R )  ,  U[0:n_controls,i] - U_ref.reshape((n_controls,1))  )  
#     obj = obj + cost_pred_st  
        
# pred_st = np.zeros((n_states,1))     


# for i in range(0,N+1):
#                                                                                                           # adding contraints so the predictions are in sync with vehicle model
#     if i == 0:
#         g = ca.vertcat(g,(X[0:n_states,i] - P[0:n_states].reshape((n_states,1))))                                                                                                         
#         K1 = f(X[0:n_states,i-1],U[0:n_controls,i-1])#Runge Kutta method of order 4
#         K2 = f(X[0:n_states,i-1] + np.multiply(K1,delta_T/2),U[0:n_controls,i-1])                                              
#         K3 = f(X[0:n_states,i-1] + np.multiply(K2,delta_T/2),U[0:n_controls,i-1])                                            
#         K4 = f(X[0:n_states,i-1] + np.multiply(K3,delta_T),U[0:n_controls,i-1])                                                  
#         pred_st = X[0:n_states,i-1] + (delta_T/6)*(K1+2*K2+2*K3+K4)           # predicted state                                                          
                
#         g = ca.vertcat( g,(X[0:n_states,i] - pred_st[0:n_states].reshape((n_states,1)) )  )                                                                



# OPT_variables = X.reshape((n_states*(N+1),1))          
# OPT_variables = ca.vertcat( OPT_variables, U.reshape((n_controls*N,1)) )          
OPT_variables = ca.vertcat(X.reshape((-1,1)), U.reshape((-1,1)))

    
nlp_prob ={
            'f':obj,
            'x':OPT_variables,
            'g':g,
            'p':P
            }

opts = {
            'ipopt':
        {
            'max_iter': 100,
            'print_level': 0,
            'acceptable_tol': 1e-8,
            'acceptable_obj_change_tol': 1e-6
        },
            'print_time': 0
        }


solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)


lbg = ca.DM.zeros(((n_states)*(N+1),1))                                     # bounds on g                                                                                                      
ubg = ca.DM.zeros(((n_states)*(N+1),1))                                                                                                         
    
# lbg[0:(n_states)*(N+1)] = - error_allowed_in_g                                                                                                  
# ubg[0:(n_states)*(N+1)] =  error_allowed_in_g                                                                                                    



lbx = ca.DM.zeros((n_states*(N+1) + n_controls*N,1))                        # bounds on X    
ubx = ca.DM.zeros((n_states*(N+1) + n_controls*N,1)) 

lbx[0:n_bound_var*(N+1):n_states] = phi_bound_min                     
ubx[0:n_bound_var*(N+1):n_states] = phi_bound_max                     
lbx[1:n_bound_var*(N+1):n_states] = phi_dot_bound_min                     
ubx[1:n_bound_var*(N+1):n_states] = phi_dot_bound_max                     
lbx[2:n_bound_var*(N+1):n_states] = theta_bound_min                 
ubx[2:n_bound_var*(N+1):n_states] = theta_bound_max 
lbx[3:n_bound_var*(N+1):n_states] = theta_dot_bound_min                 
ubx[3:n_bound_var*(N+1):n_states] = theta_dot_bound_max       
lbx[4:n_bound_var*(N+1):n_states] = psi_bound_min                 
ubx[4:n_bound_var*(N+1):n_states] = psi_bound_max 
lbx[5:n_bound_var*(N+1):n_states] = psi_dot_bound_min                 
ubx[5:n_bound_var*(N+1):n_states] = psi_dot_bound_max  
lbx[6:n_bound_var*(N+1):n_states] = z_bound_min                     
ubx[6:n_bound_var*(N+1):n_states] = z_bound_max                     
lbx[7:n_bound_var*(N+1):n_states] = Vz_bound_min                     
ubx[7:n_bound_var*(N+1):n_states] = Vz_bound_max                     
lbx[8:n_bound_var*(N+1):n_states] = theta_bound_min                 
ubx[8:n_bound_var*(N+1):n_states] = theta_bound_max 
lbx[9:n_bound_var*(N+1):n_states] = theta_dot_bound_min                 
ubx[9:n_bound_var*(N+1):n_states] = theta_dot_bound_max       
lbx[10:n_bound_var*(N+1):n_states] = psi_bound_min                 
ubx[10:n_bound_var*(N+1):n_states] = psi_bound_max 
lbx[11:n_bound_var*(N+1):n_states] = psi_dot_bound_min                 
ubx[11:n_bound_var*(N+1):n_states] = psi_dot_bound_max     




lbx[n_bound_var*(N+1):(n_bound_var*(N+1)+n_controls*N):n_controls] = omega_min                       
ubx[(n_bound_var*(N+1)):(n_bound_var*(N+1)+n_controls*N):n_controls] = omega_max                      
lbx[(n_bound_var*(N+1)+1):(n_bound_var*(N+1)+n_controls*N):n_controls] = omega_min          ##################################################     
ubx[(n_bound_var*(N+1)+1):(n_bound_var*(N+1)+n_controls*N):n_controls] = omega_max 
lbx[n_bound_var*(N+1)+2:(n_bound_var*(N+1)+n_controls*N):n_controls] = omega_min                       
ubx[(n_bound_var*(N+1)+2):(n_bound_var*(N+1)+n_controls*N):n_controls] = omega_max                      
lbx[(n_bound_var*(N+1)+3):(n_bound_var*(N+1)+n_controls*N):n_controls] = omega_min               
ubx[(n_bound_var*(N+1)+3):(n_bound_var*(N+1)+n_controls*N):n_controls] = omega_max  

# lbx[n_bound_var*(N+1):(n_bound_var*(N+1)+n_controls*N):n_controls] = omega_min                       
# ubx[(n_bound_var*(N+1)):(n_bound_var*(N+1)+n_controls*N):n_controls] = omega_max                      
# lbx[(n_bound_var*(N+1)+1):(n_bound_var*(N+1)+n_controls*N):n_controls] = 0#omega_min          ##################################################     
# ubx[(n_bound_var*(N+1)+1):(n_bound_var*(N+1)+n_controls*N):n_controls] = 0#omega_max 
# lbx[n_bound_var*(N+1)+2:(n_bound_var*(N+1)+n_controls*N):n_controls] = omega_min                       
# ubx[(n_bound_var*(N+1)+2):(n_bound_var*(N+1)+n_controls*N):n_controls] = omega_max                      
# lbx[(n_bound_var*(N+1)+3):(n_bound_var*(N+1)+n_controls*N):n_controls] = 0#omega_min               
# ubx[(n_bound_var*(N+1)+3):(n_bound_var*(N+1)+n_controls*N):n_controls] = 0#omega_max  

lbx[n_bound_var*(N+1):(n_bound_var*(N+1)+n_controls*N):n_controls] = 0#omega_min                       
ubx[(n_bound_var*(N+1)):(n_bound_var*(N+1)+n_controls*N):n_controls] = 0#omega_max                      
lbx[(n_bound_var*(N+1)+1):(n_bound_var*(N+1)+n_controls*N):n_controls] = omega_min          ##################################################     
ubx[(n_bound_var*(N+1)+1):(n_bound_var*(N+1)+n_controls*N):n_controls] = omega_max 
lbx[n_bound_var*(N+1)+2:(n_bound_var*(N+1)+n_controls*N):n_controls] = 0#omega_min                       
ubx[(n_bound_var*(N+1)+2):(n_bound_var*(N+1)+n_controls*N):n_controls] = 0#omega_max                      
lbx[(n_bound_var*(N+1)+3):(n_bound_var*(N+1)+n_controls*N):n_controls] = omega_min               
ubx[(n_bound_var*(N+1)+3):(n_bound_var*(N+1)+n_controls*N):n_controls] = omega_max  

def wRb(roll, pitch, yaw):
    R = np.zeros((3, 3))
    R = math.cos(yaw)*math.cos(pitch) - math.sin(roll)*math.sin(yaw)*math.sin(pitch),
    -math.cos(roll)*math.sin(yaw),
    math.cos(yaw)*math.sin(pitch) + math.cos(pitch)*math.sin(roll)*math.sin(yaw),
    math.sin(yaw)*math.cos(pitch) + math.sin(roll)*math.cos(yaw)*math.sin(pitch),
    math.cos(roll)*math.cos(yaw),
    math.sin(yaw)*math.sin(pitch) - math.cos(pitch)*math.sin(roll)*math.cos(yaw),
    -math.cos(roll)*math.sin(pitch),
    math.sin(roll),
    math.cos(roll)*math.cos(pitch)
    return R

def get_euler_angles_from_quaternion(quaternion_msg): 
    rpy  = tf.transformations.euler_from_quaternion(quaternion_msg)

    roll = rpy[0]
    pitch = rpy[1]
    yaw = rpy[2]  
    return np.array([roll, pitch, yaw]).reshape(3,1)

def odometry_callback(msg):

    global phi,phi_dot,theta,theta_dot,psi,psi_dot,x,y,z,Vx,Vy,Vz
    x = msg.pose.pose.position.x
    y = 1*(msg.pose.pose.position.y)
    z = 1*msg.pose.pose.position.z
    Vx = msg.twist.twist.linear.x
    Vy = 1*(msg.twist.twist.linear.y)
    Vz = 1*msg.twist.twist.linear.z
    """"p
    Identify the change
    """
    # [Vx,Vy,Vz] = wRb(phi,theta,psi) * [Vx,Vy,Vz] ##########################################
    quat_b = np.array([msg.pose.pose.orientation.x,
                       msg.pose.pose.orientation.y,
                       msg.pose.pose.orientation.z,
                       msg.pose.pose.orientation.w])
    eul_b = np.zeros((3, 1))
    eul_b = get_euler_angles_from_quaternion(quat_b)
    phi = eul_b[0]; theta = eul_b[1]; psi = eul_b[2]
    
    phi_dot = (msg.twist.twist.angular.x)
    theta_dot = msg.twist.twist.angular.y
    psi_dot = (msg.twist.twist.angular.z)

    x = x
    y = y
    z = -z
    Vx = Vx
    Vy = Vy
    Vz = -Vz
    phi = -phi
    phi_dot = -phi_dot 
    theta = 0.2#theta   ###########################################################
    theta_dot = theta_dot
    psi = psi
    psi_dot = psi_dot

    print("                             phi",phi)

    # print("z=",z,"Vz=",Vz)
rospy.init_node("hello",anonymous=True)
a = rospy.Subscriber("/hummingbird/odometry_sensor1/odometry",Odometry,odometry_callback)
pubActuators = rospy.Publisher("hummingbird/command/motor_speed", Actuators, queue_size=10)
msg = Actuators()
rate = rospy.Rate(10)
rate.sleep()
print()


##########################################################################
 

##########################################################################
while(True):
    start_time = time.time()

    
    

    ############### take path (N X 3)
    
    X_init = np.zeros((12,1)) 
    X_init[0] = phi
    X_init[1] = phi_dot
    X_init[2] = theta
    X_init[3] = theta_dot
    X_init[4] = psi
    X_init[5] = psi_dot
    X_init[6] = z
    X_init[7] = Vz
    X_init[8] = x
    X_init[9] = Vx
    X_init[10] = y
    X_init[11] = Vy
    
                              
    # X_init = np.array([phi,phi_dot,theta,theta_dot,psi,psi_dot,z,Vz,x,Vx,y,Vy], dtype = 'f') ####################################################################       
    
    # X_init = np.array([phi],[phi_dot],[theta],[theta_dot],[psi],[psi_dot],[z],[Vz],[x],[Vx],[y],[Vy], dtype = 'f') ####################################################################                                                                                                                                                                                        
    # X_target = np.array([path[total_path_points-1][0], path[total_path_points-1][1], 0 ]  , dtype = 'f')   #########################################taken####
    
    P = ca.vertcat(X_init,  X_target)                                                                                 

    # close_index = KDTree(path).query(np.array([]))[1]  ###########################                                    

    # for i in range(0,N):                                                                           
    #     P = ca.vertcat(P,path[close_index+i,0:2])                                                         
    #     P = ca.vertcat(P, math.atan((path[close_index+i+1][1] -taken path[close_index+i][1])/(path[close_index+i+1][0] - path[close_index+i][0])) )        

    # for i in range(0,N):                                                                             
    #     P = ca.vertcat(P, U_ref[0])                                                                 
    #     P = ca.vertcat(P, U_ref[1])      
    #     P = ca.vertcat(P, U_ref[2])                                                                 
    #     P = ca.vertcat(P, U_ref[3])                                                                   

    initial_X = ca.repmat(X_init, 1, N+1)
    
    initial_con = ca.DM.zeros((n_controls*N,1))                             #initial search value of control matrix
    
  
    args = {
                'lbg': ca.DM.zeros((n_states*(N+1), 1)),  # constraints lower bound
                'ubg': ca.DM.zeros((n_states*(N+1), 1)),  # constraints upper bound
                'lbx': lbx,
                'ubx': ubx
            }
    
    args['p'] = ca.vertcat(X_init, X_target)
    # print("P",P)
    args['x0'] = ca.vertcat(ca.reshape(initial_X, n_states*(N+1), 1),ca.reshape(initial_con, n_controls*N, 1))
    
    sol = solver(x0=args['x0'],lbx=args['lbx'],ubx=args['ubx'],lbg=args['lbg'],ubg=args['ubg'],p=args['p'])
    u = ca.reshape(sol['x'][n_states * (N + 1):], n_controls, N)
    X0 = ca.reshape(sol['x'][: n_states * (N+1)], n_states, N+1)

    X0 = ca.horzcat(
        X0[:, 1:],
        ca.reshape(X0[:, -1], -1, 1)
    )

    # print(X0.numel())
    print("                             pred ",X0[1,1])

    u_to_apply = u[:,0]
    print("APPL",u_to_apply)
    omega1, omega2, omega3, omega4 =u_to_apply[0], u_to_apply[1], u_to_apply[2],u_to_apply[3]#float(u_to_apply[0,0].full()[0,0]), float(u_to_apply[1,0].full()[0,0]), float(u_to_apply[2,0].full()[0,0], float(u_to_apply[3,0].full()[0,0]))
    u0 = ca.horzcat(u[:, 1:],ca.reshape(u[:, -1], -1, 1))
    # sol = solver(
                    
    #                 x0=args['x0'],
                    
    #                 lbx=args['lbx'],
    #                 ubx=args['ubx'],
                
    #                 lbg=args['lbg'],
    #                 ubg=args['ubg'],print()
    #                 p=args['p']
                        
    #             )           

    # X_U_sol = sol['x']

    # omega1 = (X_U_sol[n_states*(N+1)].full())[0][0]      
    # omega2 = (X_U_sol[n_states*(N+1)+1].full())[0][0]
    # omega3 = (X_U_sol[n_states*(N+1)+2].full())[0][0]      
    # omega4 = (X_U_sol[n_states*(N+1)+3].full())[0][0]

    thrust_U1 = b*(omega2**2+omega4**2+omega1**2+omega3**2)################################################### #TODO 
    roll_U2 = b*(-(omega2_casadi**2) + (omega4_casadi**2))
    pitch_U3 = b*(-(omega3_casadi**2) + (omega1_casadi**2))
    yaw_U4 = d*(omega2_casadi**2+omega4_casadi**2-omega1_casadi**2-omega3_casadi**2)

    # omega = [omega1,omega2,omega3,omega4]
    omega = [omega1,omega4,omega3,omega2]
    # if omega1 != omega2 and omega2 != omega3 and omega3 != omega4:
        # print("reacting to disturbance########################################################################################") 
        
    
    msg.angular_velocities = omega
    pubActuators.publish(msg)
    


    P[0:n_states] = [phi,phi_dot,theta,theta_dot,psi,psi_dot,z,Vz,x,Vx,y,Vy]            ###############################################   feedback                                                                                                
    print("Frequency:",1/(time.time()-start_time))
    rate.sleep()

    # close_index = KDTree(path).query(np.array([x,y,z]))[1]#############################################

    # if N+(close_index-1) < total_path_points :                                                      # Updating P for next N path points and next N reference controls
    #     P[n_states:n_states*(N+1):n_states] = path[close_index:N+close_index,0] 
    #     P[n_states+1:n_states*(N+1):n_states] = path[close_index:N+close_index,1]
    #     for i in range(0,N):                                                                
    #         P[n_states*(i+1+1)-1] = math.atan( (path[i+close_index+1][1] - path[i+close_index][1])/(path[i+close_index+1][0] - path[i+close_index][0] + 1e-9) )         
        
    #     P[n_states*(N+1):n_states*(N+1)+n_controls*(N-1)]= P[n_states*(N+1)+n_controls:n_states*(N+1)+n_controls*(N)]                                                                                                                                                                                                             
    #     P[n_states*(N+1)+n_controls*(N-1):n_states*(N+1)+n_controls*(N)] = U_ref                                                                                     
    # else:
    #     print (" The end point in inside horizon, slowing down")
    #     P[n_states:n_states*(N)] = P[n_states*2:n_states*(N+1)]                                                                  
    #     P[n_states*(N):n_states*(N+1)-1] = path[(total_path_points-1),0:2]                                                                                                                                                                                                                    
    #     P[n_states*(N+1)-1] = math.atan( (path[total_path_points-1][1] - path[total_path_points-1-1][1])/(path[total_path_points-1][0] - path[total_path_points-1-1][0]) )
            
    #     P[n_states*(N+1):n_states*(N+1)+n_controls*(N-1)]= P[n_states*(N+1)+n_controls:n_states*(N+1)+n_controls*(N)]                                                                                                                                                                                                                  
    #     P[n_states*(N+1)+n_controls*(N-1):n_states*(N+1)+n_controls*(N)] = np.array([0,0], dtype ='f')                  # we need to stop the bot at end, hence referance controls 0 at end 
        
    # for i in range(0,N*n_states):                                           #initial search value of state for next iteration should be the predicted one for that iteration     
    #     initial_X[i] = X_U_sol[i+n_states]                 

    # for i in range(0,(N-1)*n_controls):                                     #initial search value of control for next iteration should be the predicted one for that iteration
    #     initial_con[i] = X_U_sol[n_states*(N+1)+i+n_controls]
                
        


    
            
  


    
