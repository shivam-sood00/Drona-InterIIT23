import plutoComms
import time
import threading

if __name__=="__main__":
    
    drone = plutoComms.COMMS(debug=False)
    # readThread = threading.Thread(target=drone.read)
    writeThread = threading.Thread(target=drone.write)
    writeThread.start()
    # readThread.start()
    
    drone.land()
    
    drone.disconnect()
    writeThread.join()