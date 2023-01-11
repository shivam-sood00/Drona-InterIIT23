# Code for testing the communication with drone
import socket
import plutoComms
import time
import threading

comms = plutoComms.COMMS(debug=False)
comms.disArm()
comms.disconnect()