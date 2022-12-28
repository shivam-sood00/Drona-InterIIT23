# Code for testing the communication with drone
import socket
import comms
import time
import threading


# TCP_IP = '192.168.4.1'
# TCP_PORT = 23
# BUFFER_SIZE = 1024
# # if(self.debug):
# print("Trying to Connect")
# mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# # if(self.debug):

# mySocket.connect((TCP_IP, TCP_PORT))
# # mySocket.setblocking(False)
# print("Connection Established")
# RC = comms.MSP_SET_RAW_RC(mySocket,True)
# # CMD = comms.MSP_SET_COMMAND(mySocket,True)
# ATT = comms.MSP_ATTITUDE(mySocket,True)
# RC.arm()
# RC.disarm()

comms = comms.COMMS(debug=True)
comms.IN.Arm(False)
comms.disconnect()