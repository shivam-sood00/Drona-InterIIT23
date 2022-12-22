# Code for testing the communication with drone

import socket
import comms
import time
#if camera connected:
# TCP_IP = '192.168.0.1'
# TCP_PORT = 9060

#if not
# TCP_IP = '192.168.4.1'
# TCP_PORT = 23

myDrone = comms.comms('192.168.4.1',23,True)
myDrone.arm()
myDrone.setThrottle(1000)
myDrone.setThrottle(1050)
time.sleep(3)
myDrone.disarm()
myDrone.disconnect()

