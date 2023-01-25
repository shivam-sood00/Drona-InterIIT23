# import imp
import numpy as np
import tf.transformations as transform
from scipy.spatial.transform import Rotation
def mpc_transformations(phi, theta, psi):
    # phi = 1.0; theta = 2.0; psi = 3.0
    qx = np.sin(phi/2)*np.cos(theta/2)*np.cos(psi/2) - np.cos(phi/2)*np.sin(theta/2)*np.sin(psi/2)
    qy = np.cos(phi/2)*np.sin(theta/2)*np.cos(psi/2) + np.sin(phi/2)*np.cos(theta/2)*np.sin(psi/2)
    qw = np.cos(phi/2)*np.cos(theta/2)*np.sin(psi/2) - np.sin(phi/2)*np.sin(theta/2)*np.cos(psi/2)
    qz = np.cos(phi/2)*np.cos(theta/2)*np.cos(psi/2) + np.sin(phi/2)*np.sin(theta/2)*np.sin(psi/2)

    t2 = transform.quaternion_matrix(np.array([qw,qx,qy, qz]))
    B = [[ 1.00,         0.00,        0.00],
         [ 0.00,  np.cos(phi), np.sin(phi)],
         [ 0.00, -np.sin(phi), np.cos(phi)]]
    
    C = [[np.cos(theta), 0.00, -np.sin(theta)],
         [         0.00, 1.00,           0.00],
         [np.sin(theta), 0.00, np.cos(theta)]]

    D = [[ np.cos(psi), np.sin(psi), 0.00],
         [-np.sin(psi), np.cos(psi), 0.00],
         [        0.00,        0.00, 1.00]]
    
    B = np.array(B)
    C = np.array(C)
    D = np.array(D)

    T2 = B@C@D

    T1 = np.array([[np.cos(np.deg2rad(-45)),-np.sin(np.deg2rad(-45)), 0.00],
                   [np.sin(np.deg2rad(-45)), np.cos(np.deg2rad(-45)), 0.00],
                   [                   0.00,                    0.00, 1.00]])
    
    T4 = T1 @ T2 @ np.linalg.inv(T1)
    # print(T4)

    x_d = np.array([1, 1, 0])
    x_t = T1 @ x_d
    
    # print(qx, qy, qz, qw)
    # print(t2)
    r = Rotation.from_matrix(np.linalg.inv(T4))
    
    print(r.as_euler('xyz', degrees=True))
    # print('T1', )
    print(x_t)

    return 

def waypoint_transformations(waypoints):
    T1 = np.array([[np.cos(np.deg2rad(-45)),-np.sin(np.deg2rad(-45)), 0],
                    [np.sin(np.deg2rad(-45)),np.cos(np.deg2rad(-45)), 0],
                    [0.0, 0.0, 1.0]])
    

    pass


mpc_transformations(np.pi/6, 0, 0)

