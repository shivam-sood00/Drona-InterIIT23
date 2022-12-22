#!usr/bin/python3
import numpy as np
import math



class Drone_control: 
    
    def __init__(self):
        
        #initilizing the drone parameters
        self.kx=np.array([0,0,0]) #{kp,kd,ki}
        self.ky=np.array([0,0,0])
        self.kz=np.array([0,0,0])
        self.k_roll=np.array([0,0,0])
        self.k_pitch=np.array([0,0,0])
        self.k_yaw=np.array([0,0,0])
        self.kyaw=np.array([0,0,0])
        self.t_sampling =0.01  #sec
        self.I=np.array([0.0015,0.0015,0.0015])  #kgm^2
        self.g=9.81  #N/m^2
        self.angle_max=np.array([10,10,10]) #degrees
        self.T_max= 1000 #N
        self.thrust=0 #N
        self.m=0.04 #kg
        
        #output_parameters
        self.T=0
        self.roll=0
        self.pitch=0
        self.yaw=0
        
            
        # here we initialize the state variables for the quadrotor
        #current position of the quadrotor in  the global frame.
        self.r_curr=np.array([0,0,0])
        self.r_des=np.array([2,2,3])
        
        #body frame velocity  in global frame measured along the corresponding axis in body frame.
        self.rdot_curr=np.array([0,0,0])
        self.rdot_des=np.array([0,0,0])
        
        #acceleration in the inertial frame
        self.rddot_c=np.array([0,0,0])
        self.rddot_des=np.array([0,0,0])
        
        
        # Euler angles defined with respect to different rotation frames
        self.angle_curr=np.array([0,0,0]) #{roll,pitch,yaw}  to be obtained from the drone
        self.angle_des=np.array([0,0,0])   #to be obtained from the position controller
        self.angle_c=np.array([0,0,0])     
        # Rate of change of Euler angles defined with respect to the corresponding axis on the body frame 
        self.omega_curr=np.array([0,0,0]) #{roll_rate,pitch_rate,yaw_rate}
        self.omega_des=np.array([0,0,0])
        self.omega_c=np.array([0,0,0])
        
        #angular accelerations
        self.omegadot_c=np.array([0,0,0])
        self.omegadot_des=np.array([0,0,0])
        
        
        # intializing the error in positions
        self.err_r=np.array([0,0,0])
        self.err_rdot=np.array([0,0,0])
        self.integral_err_r=np.array([0,0,0])
        
        #initializing the error in rotations
        self.err_angle=np.array([0,0,0])
        self.err_omega=np.array([0,0,0])
        self.integral_err_angle=np.array([0,0,0])
         
         
         
    def drone_pose(self,curr_r):
        self.r_curr=curr_r
        # self.r_des=des_r
        
    def drone_rotations(self,curr_angle,curr_omega):
        self.angle_curr=curr_angle
        self.omega_curr=curr_omega
        
           
    def position_controller(self):
        
        #calculating the error values
        self.err_r = self.r_des - self.r_curr
        self.err_rdot = self.rddot_des - self.rdot_curr
        self.integral_err_r = self.integral_err_r + self.err_r
        
        #defining the control law for the position control
        if abs(self.err_r[0])>0.04 and abs(self.err_r[1])>0.04 and abs(self.err_r[2])>0.04:
            self.rddot_c[0] = self.rddot_des[0] + self.kx[0]*self.err_r[0] + self.kx[1]*self.err_rdot[0] + self.kx[2]*self.intergral_err_r[0]
            self.rddot_c[1] = self.rddot_des[1] + self.ky[0]*self.err_r[1] + self.ky[1]*self.err_rdot[1] + self.ky[2]*self.intergral_err_r[1]
            self.rddot_c[2] = self.rddot_des[2] + self.kz[0]*self.err_r[2] + self.kz[1]*self.err_rdot[2] + self.kz[2]*self.intergral_err_r[2]
            
        else:
            self.rddot_c={0,0,0}
        
        #calculating the desired roll and pitch and thrust
        self.angle_des[0] = math.atan2(self.rddot_c[0]/(self.rddot_c[2] -self.g))
        self.angle_des[1] = math.atan2((self.rddot_c[1]/self.rddot_c[0])*math.sin(self.angle_des[0]))
        self.thrust       = self.m*(self.rddot_c[1]/math.sin(self.angle_des[1]))
        
        
    #Now we calculate the roll,pitch,yaw,thrust to be given to the quadrotor
    def altitude_controller(self):
        
        #calculating the error values 
        self.err_angle = self.angle_des -self.angle_curr
        self.err_omega = self.omega_des -self.omega_curr
        self.integral_err_angle = self.integral_err_angle + self.err_angle
        
        #defining the control law for the altitude control.
        self.omegadot_c[0] = self.omegadot_des[0] + self.k_roll[0]*self.err_angle[0] + self.k_roll[1]*self.err_omega[0] + self.k_roll[2]*self.integral_err_angle[0]
        self.omegadot_c[1] = self.omegadot_des[1] + self.k_pitch[0]*self.err_angle[1] + self.k_pitch[1]*self.err_omega[1] + self.k_pitch[2]*self.integral_err_angle[1]
        self.omegadot_c[2] = self.omegadot_des[2] + self.k_yaw[0]*self.err_angle[2] + self.k_yaw[1]*self.err_omega[2] + self.k_yaw[2]*self.integral_err_angle[2]
        
        #calculating omega_c
        self.omega_c=self.omega_curr + self.omegadot_c*self.t_sampling
        #calculating angle_c
        self.angle_c=self.angle_curr +self.omega_c*self.t_sampling
        
        
    #final drone commands
    def motor_control(self):
        self.T=min(self.T_max,self.thrust)
        self.roll=min(self.angle_max[0],self.angle_c[0])
        self.pitch=min(self.angle_max[1],self.angle_c[1])
        self.yaw=min(self.angle_max[2],self.angle_c[2])
        
        
      
def main(args=None):
    drone=Drone_control()
    while True:
        drone.drone_pose(np.array([0,0,0]))
        drone.drone_rotations(np.array([0,0,0]),np.array([0,0,0]))
        drone.position_controller()
        drone.altitude_controller()
        drone.motor_control()
        print(drone.thrust)
        
        
if __name__ == '__main__':
    main()
        
        
        
    
    
      
        
        
        
        
        
        
        
        
        
        
        
    
        
        
        
        
        
    

