# Code for testing the communication with drone

import socket
#if camera connected:
# TCP_IP = '192.168.0.1'
# TCP_PORT = 9060

#if not
TCP_IP = '192.168.4.1'
TCP_PORT = 23
mysocket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
print("Trying to connect")
mysocket.connect((TCP_IP,TCP_PORT)) # A successful connection has been setup
print ("Connected to drone")
