import numpy as np
from kalman_filter import KalmanFilter
import plotly
import plotly.graph_objs as go

# [x, y, z, x', y', z', roll, pitch, yaw, roll', pitch', yaw']
init_X = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape((12, 1))
init_P = np.zeros((12, 12))
# Q = np.zeros((12, 12))
Q = np.eye(12) * 0.2
process_noise = np.zeros((12, 1))

# Drone 
mass = 0.1 # KG
gravity = 9.8
MOI = (0.1, 0.1, 0.1)



# Aruco
aruco_noise_loc = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
aruco_noise_std = [1.0, 1.0, 1.0, 0.5, 0.5, 0.5]

# IMU
imu_noise_loc = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
imu_noise_std = [0.4, 0.4, 0.4, 0.2, 0.2, 0.2]

# Control Input
u_loc = [0.0, 0.0, 0.0, 0.0]
u_std = [1.0, 1.0, 1.0, 1.0]

num_examples = 100
dt = 1.0

kf = KalmanFilter(init_X.copy(), init_P.copy(), process_noise.copy(), Q.copy(), mass, gravity, MOI)


gt_states = []
gt_states.append(kf.H @ init_X)


control_inputs = []


aruco_measurements = []
temp = gt_states[0].copy()
for j in range(6):
    temp[j][0] += np.random.normal(aruco_noise_loc[j], aruco_noise_std[j], 1)
aruco_measurements.append(temp.copy())


imu_measurements = []
temp = gt_states[0].copy()
for j in range(6):
    temp[j][0] += np.random.normal(imu_noise_loc[j], imu_noise_std[j], 1)
imu_measurements.append(temp.copy())


# Generate Data
for i in range(num_examples - 1):
    
    control_input = []
    for j in range(len(u_loc)):
        control_input.append(np.random.normal(u_loc[j], u_std[j], 1))
    control_input = np.array(control_input).reshape((4, 1))
    control_inputs.append(control_input)

    kf.apply_system_dynamics(control_input, dt)
    next_state = kf.H @ kf.X
    gt_states.append(next_state.copy())

    temp = next_state.copy()
    for j in range(6):
        temp[j][0] += np.random.normal(aruco_noise_loc[j], aruco_noise_std[j], 1)
    aruco_measurements.append(temp.copy())
    
    temp = next_state.copy()
    for j in range(6):
        temp[j][0] += np.random.normal(imu_noise_loc[j], imu_noise_std[j], 1)
    imu_measurements.append(temp.copy())



# Cov
aruco_cov = np.cov((np.vstack(aruco_measurements).reshape((num_examples, 6)) - np.vstack(gt_states).reshape((num_examples, 6))).T)
imu_cov = np.cov((np.vstack(imu_measurements).reshape((num_examples, 6)) - np.vstack(gt_states).reshape((num_examples, 6))).T)



del kf
# Apply KF
kf = KalmanFilter(init_X.copy(), init_P.copy(), process_noise.copy(), Q.copy(), mass, gravity, MOI)
kf_estimates = []
kf_estimates.append(kf.H @ init_X)

for i in range(len(gt_states)-1):
    kf.apply_system_dynamics(control_inputs[i], dt)
    
    # IMU update
    kf.update_measurement(imu_measurements[i+1], np.array(imu_noise_loc).reshape((6, 1)), imu_cov)
    
    # Aruco 
    kf.update_measurement(aruco_measurements[i+1], np.array(aruco_noise_loc).reshape((6, 1)), aruco_cov)

    kf_estimates.append((kf.H @ kf.X).copy())
    

kf_cov = np.cov((np.vstack(kf_estimates).reshape((num_examples, 6)) - np.vstack(gt_states).reshape((num_examples, 6))).T)

print("Aruco: ", aruco_cov)
print("IMU: ", imu_cov)
print("KF: ", kf_cov)



markercolor = np.linspace(0.0, 1.0, 99)
gt_states = np.vstack(gt_states).reshape((num_examples, 6))
aruco_measurements = np.vstack(aruco_measurements).reshape((num_examples, 6))
imu_measurements = np.vstack(imu_measurements).reshape((num_examples, 6))
kf_estimates = np.vstack(kf_estimates).reshape((num_examples, 6))

#Make Plotly figure
fig1 = go.Scatter3d(x=gt_states[1:, 0],
                    y=gt_states[1:, 1],
                    z=gt_states[1:, 2],
                    marker=dict(color=markercolor,
                                opacity=1,
                                reversescale=False,
                                colorscale='Blues',
                                size=2),
                    line=dict (width=2,
                    color=markercolor,
                    colorscale="Blues"),
                    mode='lines')



fig2 = go.Scatter3d(x=aruco_measurements[1:, 0],
                    y=aruco_measurements[1:, 1],
                    z=aruco_measurements[1:, 2],
                    marker=dict(color=markercolor,
                                opacity=1,
                                reversescale=False,
                                colorscale='Greens',
                                size=2),
                    line=dict (width=2,
                    color="green",),
                    mode='lines')


fig3 = go.Scatter3d(x=imu_measurements[1:, 0],
                    y=imu_measurements[1:, 1],
                    z=imu_measurements[1:, 2],
                    marker=dict(color=markercolor,
                                opacity=1,
                                reversescale=False,
                                colorscale='Reds',
                                size=2),
                    line=dict (width=2,
                    color="red"),
                    mode='lines')


fig4 = go.Scatter3d(x=kf_estimates[1:, 0],
                    y=kf_estimates[1:, 1],
                    z=kf_estimates[1:, 2],
                    marker=dict(color=markercolor,
                                opacity=1,
                                reversescale=False,
                                colorscale='purples',
                                size=2),
                    line=dict (width=2,
                    color="purple"),
                    mode='lines')


#Make Plot.ly Layout
mylayout = go.Layout(scene=dict(xaxis=dict( title="x"),
                                yaxis=dict( title="y"),
                                zaxis=dict(title="z")),)

#Plot and save html
plotly.offline.plot({"data": [fig1, fig2, fig3, fig4],
                     "layout": mylayout},
                     auto_open=True,
                     filename=("4DPlot.html"))
