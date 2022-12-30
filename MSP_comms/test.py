import socket
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
    # drone.takeOff()
    drone.takeOff()
    time.sleep(5)
    s = time.time()
    i = 0
    # while time.time()-s<10:
    #     i+=1
    #     drone.paramsSet["Throttle"] = min(1500,1000+i)
        # print(drone.paramsReceived)
    drone.land()
    time.sleep(10)
    print("hereeeeeeeeeee!!!!!!1")
    drone.disArm()    
    drone.disArm()  
    time.sleep(1)  
    drone.sendData = False
    readThread.join()
    writeThread.join()
    
    drone.disconnect()