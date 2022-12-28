# Code for testing the communication with drone
import socket
import comms
import time
import threading
#if camera connected:
# TCP_IP = '192.168.0.1'
# TCP_PORT = 9060

#if not
# TCP_IP = '192.168.4.1'
# TCP_PORT = 23


if __name__=="__main__":
    
    comms = comms.COMMS(debug=True)
    comms.IN.Arm(True)
    comms.IN.setThrottle(900)
    time.sleep(1)
    comms.OUT.requestMSPAttitude()
    readThread = threading.Thread(target=comms.OUT.receiveMSPAttitude)
    readThread.start()
    s = time.time()
    while(time.time()-s<10):
        comms.OUT.requestMSPAttitude()
        time.sleep(0.5)
    readThread.join()
    comms.IN.Arm(False)
    comms.disconnect()