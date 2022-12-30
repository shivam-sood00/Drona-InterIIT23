"""
Control Your Drone!
---------------------------
Moving around:
    u    i    o
    j    k    l
    n    m    ,


spacebar : arm or disarm
w : increase height
s : decrease height
q : take off
e : land
a : yaw left
d : yaw right
t : auto pilot on/off
Up arrow : go forward
Down arrow : go backward
Left arrow : go left
Right arrow : go right

CTRL+C to quit

"""

import threading,comms,time


inputVal = None

def inputFun():
    global inputVal
    while(inputVal!="p"):
        inputVal = input("enter key: ")

if __name__=="__main__":
    inputThread = threading.Thread(target=inputFun)
    inputThread.start()
    comms = comms.COMMS()
    # toggle = False
    inputVal = "o"
    while(inputVal!="p"):
        print("inputVal: ",inputVal)
        if inputVal=="q":
            comms.IN.takeOff()
        elif inputVal==",":
            # toggle = not toggle
            comms.IN.Arm(True)
        elif inputVal==".":
            comms.IN.Arm(False)
        elif inputVal=="e":
            comms.IN.land()
        elif inputVal=="w":
            comms.IN.setThrottle(1800)
        elif inputVal=="s":
            comms.IN.setThrottle(1000)
        elif inputVal=="a":
            comms.IN.setYaw(1200)
        elif inputVal=="d":
            comms.IN.setYaw(1800)
        elif inputVal=="u":
            comms.IN.setPitch(1800)
        elif inputVal=="j":
            comms.IN.setPitch(1200)
        elif inputVal=="h":
            comms.IN.setRoll(1200)
        elif inputVal=="k":
            comms.IN.setRoll(1800)
        elif inputVal=="b":
            comms.IN.backFlip()
        time.sleep(0.1)
        # q = cv2.waitKey(1)
        # if q!=-1:
        #     print(q)
        #     inputVal=q
        
    inputThread.join()