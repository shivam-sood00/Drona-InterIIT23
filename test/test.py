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



TCP_IP = '192.168.4.1'
TCP_PORT = 23
BUFFER_SIZE = 1024
# if(self.debug):
print("Trying to Connect")
mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# if(self.debug):
mySocket.connect((TCP_IP, TCP_PORT))
print("Connection Established")

RC = comms.MSP_SET_RAW_RC(mySocket,True)
CMD = comms.MSP_SET_COMMAND(mySocket,True)
ATT = comms.MSP_ATTITUDE(mySocket,True)
RC.arm()

RC.setThrottle(900)
RC.setThrottle(1000)
ATT.sendPacket()
arr = ATT.recieveResponse()
arr = list(arr)
print(arr)
# CMD.takeOff()
time.sleep(3)
RC.disarm()
mySocket.close()

