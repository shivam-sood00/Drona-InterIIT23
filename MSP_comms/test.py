import socket
import comms
import time
import threading


if __name__=="__main__":
    
    comms = comms.COMMS(debug=True)
    comms.IN.Arm(True)
    # comms.IN.setThrottle(900)
    # comms.IN.setThrottle(1000)
    # comms.IN.setThrottle(1100)
    # comms.IN.setThrottle(1200)
    # time.sleep(1)
    # comms.OUT.requestMSPAttitude()
    readThread = threading.Thread(target=comms.updateParams)
    readThread.start()
    # comms.IN.setThrottle(1500)
    # comms.IN.setThrottle(1500)
    # comms.IN.setThrottle(1500)
    # comms.IN.setThrottle(1500)
    
    # time.sleep(5)
    # comms.IN.takeOff()
    # comms.IN.Arm(False)
    # comms.IN.Arm(False)
    # comms.IN.Arm(True)
    s = time.time()
    i = 0
    while(time.time()-s<2):
        print(i)
        i +=1
        comms.IN.takeOff()
    #     val = min(1700,900+i)
    #     comms.IN.setThrottle(val)
    #     # comms.OUT.requestMSPAltitude()
    #     # comms.OUT.requestMSPAttitude()
        # comms.OUT.requestMSPRawIMU()
        # time.sleep(0.1)
    
    s = time.time()
    i = 0
    while(time.time()-s<2):
        print("Throttle: ",i)
        i +=1
        val = min(1500,900+i)
        comms.IN.setThrottle(val)
    # comms.IN.takeOff()
    # s = time.time()
    # while(time.time()-s<1):
    #     comms.IN.land()
    # comms.IN.land()
    readThread.join()
    comms.IN.Arm(False)
    comms.disconnect()