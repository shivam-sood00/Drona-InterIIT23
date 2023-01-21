# Code for testing the communication with drone
import socket
import comms
import time
import threading

comms = comms.COMMS(debug=True)
comms.IN.Arm(False)
comms.disconnect()