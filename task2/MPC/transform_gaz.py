import rospy
import numpy as np
from nav_msgs.msg import Odometry
import tf

def get_euler_angles_from_quaternion(quaternion_msg): 
    rpy  = tf.transformations.euler_from_quaternion(quaternion_msg)

    roll = rpy[0]
    pitch = rpy[1]
    yaw = rpy[2]  
    return np.array([roll, pitch, yaw]).reshape(3,1)

def odometry_callback(msg):
    quat_b = np.array([msg.pose.pose.orientation.x,
                       msg.pose.pose.orientation.y,
                       msg.pose.pose.orientation.z,
                       msg.pose.pose.orientation.w])
    eul_b = np.zeros((3, 1))
    eul_b = get_euler_angles_from_quaternion(quat_b)
    phi = eul_b[0]; theta = eul_b[1]; psi = eul_b[2]
    print("phi:",phi,"theta:",theta,"psi:",psi)
while(True):

    rospy.init_node("hello",anonymous=True)
    a = rospy.Subscriber("/hummingbird/ground_truth/odometry",Odometry,odometry_callback)
    