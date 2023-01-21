import numpy as np
class QuadrotorDynamics():
    def __init__(self,m = 0.060 ,g = 9.81, I_x = 6.50e-05, I_y = 6.21e-05, I_z = 1.18e-04):
        #Init Conditions
        self.m = m
        self.g = g
        self.I_x = I_x
        self.I_y = I_y
        self.I_z = I_z

    def getA(self):
        
        A = np.array([[ 0., 0., 0., 1., 0., 0. ], #x
                      [ 0., 0., 0., 0., 1., 0. ], #y
                      [ 0., 0., 0., 0., 0., 1. ], #z
                      [ 0., 0., 0., 0., 0., 0. ], #x_dot
                      [ 0., 0., 0., 0., 0., 0. ], #y_dot
                      [ 0., 0., 0., 0., 0., 0. ]]) #z_dot
        return A

    def getB(self,state):
        
        phi = state["Roll"]
        theta = state["Pitch"]
        psi = state["Yaw"]

        a = np.sin(phi) * np.sin(psi) + np.cos(phi) * np.cos(psi) * np.sin(theta)
        b =-np.cos(psi) * np.sin(phi) + np.cos(phi) * np.sin(psi) * np.sin(theta)
        c = np.cos(phi) * np.cos(theta)
        B = np.array([[ 0. ],
                      [ 0. ],
                      [ 0. ],
                      [ a ],
                      [ b ],
                      [ c ]])
        return B
    
    def getC(self,state):
        phi = state["Roll"]
        theta = state["Pitch"]
        psi = state["Yaw"]
        a = np.sin(phi) * np.sin(psi) + np.cos(phi) * np.cos(psi) * np.sin(theta)
        b =-np.cos(psi) * np.sin(phi) + np.cos(phi) * np.sin(psi) * np.sin(theta)
        c = np.cos(phi) * np.cos(theta)
        B = np.array([[ 0. ],
                      [ 0. ],
                      [ 0. ],
                      [ self.g*a/c ],
                      [ self.g*b/c ],
                      [ 0.         ]])
        return C

    # def getA(self,state):

    #     x  = state[0]
    #     phi = state[6]
    #     theta = state[7]
        
    #     p = state[9]
    #     q = state[10]

    #     u = state[12]
    #     v = state[13]
    #     w = state[14]

        
    #     if x==0:
    #         x = 0.0000001

    #     _a = self.g/x
    #     _b =np.sin(phi)*np.tan(theta)
    #     _c =np.cos(phi)*np.tan(theta)

    #     _d = np.cos(phi)
    #     _e = -np.sin(phi)

    #     _f = np.sin(phi)/np.cos(theta)
    #     _g = np.cos(phi)/np.cos(theta)

    #     _h =((self.I_y-self.I_z)*q)/self.I_x
    #     _i =((self.I_z-self.I_x)*p)/self.I_y
    #     _j =((self.I_x-self.I_y)*q)/self.I_z

    #     _k = -self.g*np.sin(theta)/x
    #     _l = -w
    #     _m =  v

    #     _n = -(self.g*np.sin(phi)*np.cos(theta))/x
    #     _o =  w
    #     _p = -u

    #     _q = -(self.g*np.cos(phi)*np.cos(theta))/x
    #     _r = -v 
    #     _s =  u

    #     A = np.array([[ 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ], #x
    #                   [ 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ], #y
    #                   [ 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 ], #z
                    
    #                   [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ], #x_dot
    #                   [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ], #y_dot
    #                   [_a, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ], #z_dot
                    
    #                   [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,_b,_c, 0, 0, 0 ], #phi
    #                   [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,_d,_e, 0, 0, 0 ], #theta
    #                   [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,_f,_g, 0, 0, 0 ], #psi


    #                   [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,_h, 0, 0, 0 ], #p
    #                   [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,_i, 0, 0, 0 ], #q
    #                   [ 0, 0, 0, 0, 0, 0, 4, 0, 0,_j, 0, 0, 0, 0, 0 ], #r

    #                   [_k, 0, 0, 0, 0, 0, 0, 0, 0, 0,_l,_m, 0, 0, 0 ], #u
    #                   [_n, 0, 0, 0, 0, 0, 0, 0, 0,_o, 0,_p, 0, 0, 0 ], #v
    #                   [_q, 0, 0, 0, 0, 0, 4, 0, 0,_r,_s, 0, 0, 0, 0 ]] #w
    #                 )
    #     return A
    # def getB(self,state):
    #     phi = state[6]
    #     theta = state[7]
    #     psi = state[8]
    #     #################################################################################3
    #     _a =(-np.sin(phi)*np.sin(psi)-np.cos(phi)*np.cos(psi)*np.sin(theta))/self.m
    #     _b =( np.sin(phi)*np.cos(psi)-np.cos(phi)*np.sin(psi)*np.sin(theta))/self.m
    #     _c =(-np.cos(phi)*np.cos(theta))/self.m

    #     _d =  1/self.I_x
    #     _e =  1/self.I_y
    #     _f =  1/self.I_z
    #     _g = -1/self.m

    #     B = np.array([[ 0, 0, 0, 0],
    #                 [ 0, 0, 0, 0],
    #                 [ 0, 0, 0, 0],
    #                 [_a, 0, 0, 0],
    #                 [_b, 0, 0, 0],
    #                 [_c, 0, 0, 0],
    #                 [ 0, 0, 0, 0],
    #                 [ 0, 0, 0, 0],
    #                 [ 0, 0, 0, 0],
    #                 [ 0,_d, 0, 0],
    #                 [ 0, 0,_e, 0],
    #                 [ 0, 0, 0,_f],
    #                 [ 0, 0, 0, 0],
    #                 [ 0, 0, 0, 0],
    #                 [ 0, 0, 0,_g]])
    #     return B
        #################################################################################
    # def getH(self,state,control_input):
    #     U1 = control_input[0]
    #     x = state[0]

    #     _a = -U1/(self.m*x)
    #     # H = np.array([[ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ],
    #     #              [ 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ],
    #     #              [ 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ],
    #     #              [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 ],
    #     #              [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 ],
    #     #              [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 ],
    #     #              [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ],
    #     #              [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ],
    #     #              [_a, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ]])
        
    #     H = np.array([[ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ],
    #                   [ 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ],
    #                   [ 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ]])
    #     return H
