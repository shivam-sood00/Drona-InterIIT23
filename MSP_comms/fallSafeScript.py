from MSP_comms.plutoComms import COMMS
import time
import threading

if __name__=="__main__":
    
    drone = COMMS(debug=False)
    # readThread = threading.Thread(target=drone.read)
    writeThread = threading.Thread(target=drone.write)
    writeThread.start()
    # readThread.start()
    
    # drone.land()
    drone.arm()
    drone.reset()
    drone.paramsSet["Throttle"] = 1200
    time.sleep(0.3)
    drone.paramsSet["Throttle"] = 1100
    time.sleep(2)
    drone.paramsSet["Throttle"] = 1000
    time.sleep(3)
    # drone.paramsSet["Throttle"] = 1000
    # time.sleep(2)
    drone.disArm()
    
    drone.disconnect()
    writeThread.join()
    # readThread.join()