import numpy as np
class QuadrotorDynamics():
    """
    A helper class for Quadrotor Dynamics. Initialized with mass (m), gravity(g) and the three moment of intertia (I_x, I_y, I_z) and
    calculates the Matrix A, B and C for the state space equation X' = AX + BU + C.
    
    Attributes:
    	m: Mass of the drone. Default is 0.060 (in Kg)
    	g: Gravity. Default is 9.81 (in m/s^2)
    	I_x: Moment of Inertia about X. Default is 6.50e-05 (in Kg m^2)
    	I_y: Moment of Inertia about Y. Default is 6.21e-05 (in Kg m^2)
    	I_z: Moment of Inertia about Z. Default is 1.18e-04 (in Kg m^2)
    	
    """
    def __init__(self,m = 0.060 ,g = 9.81, I_x = 6.50e-05, I_y = 6.21e-05, I_z = 1.18e-04):
        # Initial Conditions
        self.m = m
        self.g = g
        self.I_x = I_x
        self.I_y = I_y
        self.I_z = I_z

    def getA(self):
        """
        Return:
            A: Dynamics Matrix A
        """
        A = np.array([[ 0, 0, 0, 1.0, 0, 0 ], #x
                      [ 0, 0, 0, 0, 1, 0 ], #y
                      [ 0, 0, 0, 0, 0, 1 ], #z
                      [ 0, 0, 0, 0, 0, 0 ], #x_dot
                      [ 0, 0, 0, 0, 0, 0 ], #y_dot
                      [ 0, 0, 0, 0, 0, 0 ]]) #z_dot
        return A

    def getB(self,imuData):
        """
        Parameters:
            imuData: IMU Data
            
        Returns:
            B: Control Matrix B
        """
        phi = imuData["Roll"]
        theta = imuData["Pitch"]
        psi = imuData["Yaw"]

        a =(np.sin(phi)*np.sin(psi) + np.cos(phi)*np.cos(psi)*np.sin(theta))/self.m
        b =(-np.sin(phi)*np.cos(psi) + np.cos(phi)*np.sin(psi)*np.sin(theta))/self.m
        c =(np.cos(phi)*np.cos(theta))/self.m
        B = np.array([[ 0 ],
                      [ 0 ],
                      [ 0 ],
                      [ a ],
                      [ b ],
                      [ c ]])
        return B
    
    def getC(self):
        """
        Returns:
            C: Constant Matrix C
        """
        C = np.array([[ 0 ],
                      [ 0 ],
                      [ 0 ],
                      [ 0 ],
                      [ 0 ],
                      [ -self.g ]])
        return C
