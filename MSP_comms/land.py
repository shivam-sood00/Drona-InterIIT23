# Code for testing the communication with drone
import comms,time

comms = comms.COMMS(debug=True)
s = time.time()
while(time.time()-s<4):
    comms.IN.land()

comms.IN.Arm(False)
comms.disconnect()