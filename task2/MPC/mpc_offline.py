
import casadi as ca
import numpy as np 

N =1

m = 0.056
l = 0.065
Ixx = 6.5e-05
Iyy = 6.21e-05
Izz = 1.18e-04
Jr = 5.15e-07
b = 2.83e-08
d = 1.55e-10
gr = 9.81


Vz_casadi = ca.SX.sym('Vz')
n_states = 1

inf = np.inf

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


omega_max = 460                                                                                                                                     
omega_min = 0#-omega_max

omega1_casadi =ca.SX.sym('omega1')
omega2_casadi = ca.SX.sym('omega2')
omega3_casadi =ca.SX.sym('omega3')
omega4_casadi = ca.SX.sym('omega4')
controls = ca.vertcat(omega1_casadi,omega2_casadi,omega3_casadi,omega4_casadi)
n_controls = controls.numel()   

rhs = gr - b*(omega2_casadi**2+omega4_casadi**2+omega1_casadi**2+omega3_casadi**2)/m
#rhs = gr - (ca.cos(phi_casadi))*(ca.cos(theta_casadi))*b*(omega2_casadi**2+omega4_casadi**2+omega1_casadi**2+omega3_casadi**2)/m


f = ca.Function('f',[Vz_casadi,controls],[rhs])        

U = ca.SX.sym('U', n_controls,N)                                                                        # For storing predicted controls                                                               
X =ca.SX.sym('X', n_states, N+1)                                                                        # For storing predicted states
P = ca.SX.sym('P', n_states + n_states )                                         # For storing odometry, next N path points and next N referance controls    

# obj = 0
# g=[]
# step_horizon_rk = 0.1
# state_i, control_i = X[:,0], U[:,0]

# obj = obj + (state_i - P[n_states:]).T @ 1 @ (state_i - P[n_states:])  #+ (control_i - U_ref).T @ R @ (control_i - U_ref)
# st_next = X[:, 1]
# k1 = f(state_i, control_i)
# k2 = f(state_i + step_horizon_rk/2*k1, control_i)
# k3 = f(state_i + step_horizon_rk/2*k2, control_i)
# k4 = f(state_i + step_horizon_rk * k3, control_i)
# st_next_RK4 = state_i + (step_horizon_rk / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
# g = ca.vertcat(g, st_next - st_next_RK4)  


obj = 0
g = X[:, 0] - P[:n_states] 
step_horizon_rk = 0.1
for k in range(0, N):
    state_i, control_i = X[:,k], U[:,k]
    
    obj = obj + (state_i - P[n_states:]).T @ 1 @ (state_i - P[n_states:])  #+ (control_i - U_ref).T @ R @ (control_i - U_ref)
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
# for i in range(0,N)


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


lbx = ca.DM.zeros((n_states*(N+1) + n_controls*N,1))                        # bounds on X    
ubx = ca.DM.zeros((n_states*(N+1) + n_controls*N,1)) 


n_bound_var = n_states


ubx[0:n_bound_var*(N+1)] = [inf,inf]
lbx[0:n_bound_var*(N+1)] = [-inf,-inf]


lbx[n_bound_var*(N+1):(n_bound_var*(N+1)+n_controls*N):n_controls] = omega_min                       
ubx[(n_bound_var*(N+1)):(n_bound_var*(N+1)+n_controls*N):n_controls] = omega_max                      
lbx[(n_bound_var*(N+1)+1):(n_bound_var*(N+1)+n_controls*N):n_controls] = omega_min               
ubx[(n_bound_var*(N+1)+1):(n_bound_var*(N+1)+n_controls*N):n_controls] = omega_max 
lbx[n_bound_var*(N+1)+2:(n_bound_var*(N+1)+n_controls*N):n_controls] = omega_min                       
ubx[(n_bound_var*(N+1)+2):(n_bound_var*(N+1)+n_controls*N):n_controls] = omega_max                      
lbx[(n_bound_var*(N+1)+3):(n_bound_var*(N+1)+n_controls*N):n_controls] = omega_min               
ubx[(n_bound_var*(N+1)+3):(n_bound_var*(N+1)+n_controls*N):n_controls] = omega_max      



    
                               
X_init = np.array([100], dtype = 'f')                                                                                                                                                                                  
# X_target = np.array([path[total_path_points-1][0], path[total_path_points-1][1], 0 ]  , dtype = 'f')   #############################################
X_target = -1

P = ca.vertcat(X_init, X_target)                                                                                 

initial_X = ca.repmat(X_init, 1, N+1)

initial_con = ca.DM.zeros((n_controls*N,1))                             #initial search value of control matrix


args = {
            'lbg': ca.DM.zeros((n_states*(1+1), 1)),  # constraints lower bound
            'ubg': ca.DM.zeros((n_states*(1+1), 1)),  # constraints upper bound
            'lbx': lbx,
            'ubx': ubx
        }

args['p'] = ca.vertcat(X_init, X_target)

args['x0'] = ca.vertcat(ca.reshape(initial_X, n_states*(N+1), 1),ca.reshape(initial_con, n_controls*N, 1))

sol = solver(x0=args['x0'],lbx=args['lbx'],ubx=args['ubx'],lbg=args['lbg'],ubg=args['ubg'],p=args['p'])
u = ca.reshape(sol['x'][n_states * (N + 1):], n_controls, N)
X0 = ca.reshape(sol['x'][: n_states * (N+1)], n_states, N+1)

X0 = ca.horzcat(
    X0[:, 1:],
    ca.reshape(X0[:, -1], -1, 1)
)

u_to_apply = u[:,0]
print("APPL",u_to_apply)
omega1, omega2, omega3, omega4 =u_to_apply[0], u_to_apply[1], u_to_apply[2],u_to_apply[3]#float(u_to_apply[0,0].full()[0,0]), float(u_to_apply[1,0].full()[0,0]), float(u_to_apply[2,0].full()[0,0], float(u_to_apply[3,0].full()[0,0]))
u0 = ca.horzcat(u[:, 1:],ca.reshape(u[:, -1], -1, 1))




   