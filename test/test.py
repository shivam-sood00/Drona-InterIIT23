import socket
import comms
import time
import threading


if __name__=="__main__":
    
    comms = comms.COMMS(debug=True)
    comms.IN.Arm(True)
    comms.IN.setThrottle(900)
    time.sleep(1)
    # comms.OUT.requestMSPAttitude()
    readThread = threading.Thread(target=comms.updateParams)
    readThread.start()
    s = time.time()
    while(time.time()-s<10):
        comms.OUT.requestMSPAltitude()
        comms.OUT.requestMSPAttitude()
        comms.OUT.requestMSPRawIMU()
        time.sleep(0.1)
    readThread.join()
    comms.IN.Arm(False)
    comms.disconnect()