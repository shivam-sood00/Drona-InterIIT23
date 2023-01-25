import rospy
from std_srvs.srv import Empty
from mav_msgs.msg import Actuators
import numpy as np
import time
rospy.init_node('reset_world')

pubActuators = rospy.Publisher("/" + "hummingbird" +"/command/motor_speed", Actuators, queue_size=10)
msg = Actuators()
s = time.time()
while(time.time() - s < 1):
    msg.angular_velocities = np.array([0,0,0,0]).reshape(4,1)
    pubActuators.publish(msg)
rospy.wait_for_service('/gazebo/reset_world')
reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
reset_world()