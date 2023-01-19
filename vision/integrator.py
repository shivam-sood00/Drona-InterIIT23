import numpy as np


def get_velocity(currentStateRates, imuData, filt_imuData, dt):

    phi = imuData["Roll"]
    theta = imuData["Pitch"]
    psi = imuData["Yaw"]

    a =  np.cos(psi) * np.cos(theta)
    b = -np.cos(phi) * np.sin(psi) + np.cos(psi) * np.sin(phi) * np.sin(theta)
    c =  np.sin(phi) * np.sin(psi) + np.cos(phi) * np.cos(psi) * np.sin(theta)
    d =  np.sin(psi) * np.cos(theta)
    e =  np.cos(phi) * np.cos(psi) + np.sin(phi) * np.sin(psi) * np.sin(theta)
    f = -np.cos(psi) * np.sin(phi) + np.cos(phi) * np.sin(psi) * np.sin(theta)
    g = -np.sin(theta)
    h =  np.cos(theta) * np.sin(phi)
    i = np.cos(phi) * np.cos(theta)

    R = np.array(  [a,  b,  c ],          
                    [d,  e,  f ],          
                    [g,  h,  i ])

    current_body_vel = R.T @ np.array([currentStateRates[0],currentStateRates[1],currentStateRates[2]])
    current_body_vel += np.array([filt_imuData[0],filt_imuData[1],filt_imuData[2]]) * dt
    current_vel = R @ current_body_vel

    velocity_x  = current_vel[0]
    velocity_y  = current_vel[1]
    velocity_z  = current_vel[2]

    return [velocity_x,velocity_y,velocity_z]

def get_angle_rate(imuData, filt_gyroData):
    phi = imuData["Roll"]
    theta = imuData["Pitch"]

    a =  np.sin(phi) * np.tan(theta)
    b =  np.cos(phi) * np.tan(theta)
    c =  np.cos(phi)
    d = -np.sin(phi)
    e =  np.sin(phi)/np.cos(theta)
    f =  np.cos(phi)/np.cos(theta)

    R = np.array(   [1,  a,  b ],          
                    [0,  c,  d ],          
                    [0,  e,  f ] )
    
    currentAngleRate =  R @ np.array([filt_gyroData[0],filt_gyroData[1],filt_gyroData[2]])

    roll_rate = currentAngleRate[0] 
    pitch_rate = currentAngleRate[1]
    yaw_rate = currentAngleRate[2]

    return [roll_rate,pitch_rate,yaw_rate]
