import plutoComms
import time
import threading

if __name__=="__main__":
    
    drone = plutoComms.COMMS(debug=False)
    readThread = threading.Thread(target=drone.read)
    writeThread = threading.Thread(target=drone.write)
    writeThread.start()
    readThread.start()
    
    drone.arm()
    time.sleep(5)
    drone.decreaseHeight()
    time.sleep(10)
    
    drone.disArm()
    
    drone.disconnect()
    writeThread.join()
    readThread.join()