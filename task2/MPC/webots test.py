from controller import Robot
robot = Robot()

timestep = int(robot.getBasicTimeStep())

gps = robot.getDevice('gps')
gps.enable(timestep)
gps_ee = robot.getDevice('gps_ee')
gps_ee.enable(timestep)
IU = robot.getDevice('inertial unit')
IU.enable(timestep)

while (robot.step(timestep) != -1):     
	
    theta_odom =IU.getRollPitchYaw()[1]  

    print(theta_odom)
    