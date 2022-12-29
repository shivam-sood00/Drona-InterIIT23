# Code for testing the communication with drone
import socket
# import comms
import old
import time
import threading
#if camera connected:
# TCP_IP = '192.168.0.1'
# TCP_PORT = 9060

#if not
# TCP_IP = '192.168.4.1'
# TCP_PORT = 23

def readFun():
    arr = []
    while(True):
        orientation = ATT.recieveResponse(arr)
        if(orientation==-1):
            break
        else:
            if(type(orientation)!=type(1)):
                arr = []
            print("orientation: ",orientation)



if __name__=="__main__":
    
    
    TCP_IP = '192.168.4.1'
    PORT = 23

    
    mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    mySocket.connect((TCP_IP, PORT))
    # if(debug):
    print("Socket Connected")
    
    readThread = threading.Thread(target=readFun)
    
    # comms = comms.COMMS(debug=True)
    # comms.IN.Arm(True)
    # comms.IN.setThrottle(900)
    RC = old.MSP_SET_RAW_RC(mySocket)
    ATT = old.MSP_ATTITUDE(mySocket,True)
    RC.arm()
    RC.setThrottle(900)
    time.sleep(1)
    ATT.sendPacket()
    readThread.start()
    s = time.time()
    while(time.time()-s<10):
        ATT.sendPacket()
        time.sleep(0.5)
    readThread.join()
    RC.disarm()
    mySocket.close()
    
    

# if __name__=="__main__":
    
#     comms = comms.COMMS(debug=True)
#     comms.IN.Arm(True)
#     comms.IN.setThrottle(900)
#     time.sleep(1)
#     comms.OUT.requestMSPAttitude()
#     readThread = threading.Thread(target=comms.updateParams)
#     readThread.start()
#     s = time.time()
#     while(time.time()-s<10):
#         comms.OUT.requestMSPAttitude()
#         time.sleep(0.5)
#     readThread.join()
#     comms.IN.Arm(False)
#     comms.disconnect()