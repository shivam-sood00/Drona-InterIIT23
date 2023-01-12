# Code for testing the communication with drone
import socket
from MSP_comms.plutoComms import COMMS
import time
import threading

comms = COMMS(debug=False)
comms.disArm()
comms.disconnect()