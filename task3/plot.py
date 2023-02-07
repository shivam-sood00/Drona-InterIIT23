import time
file = open('debug.txt', 'r')
# This will print every line one by one in the file
''''
Utilites for parsing flight data and making required plots.
'''
x=[]
y=[]
z=[]

errX=[]
errY=[]
errZ=[]

roll=[]
pitch=[]
throttle=[]
yaw=[]

stateYaw=[]
statePitch=[]
stateRoll=[]

errX_with_sse=[]
errY_with_sse=[]
errZ_with_sse=[]

err_diffX=[]
err_diffY=[]
err_diffZ=[]

err_intX=[]
err_intY=[]
err_intZ=[]

vel = []


stateCont = 3
i= 0
for each in file:
    i += 1
    print(i)
    if each.strip()=="":
        continue
    if stateCont<3:
        stateCont-=1
        if stateCont==0:
            stateCont==3
        continue
    if len(each) > len("Landing") and each.startswith('Landing'):
        break
    elif (each[0]>="A" and each[0]<="Z") or (each[0]>="a" and each[0]<="z"):
        continue
    elif each[0]=='[' and each[1]=='[':
        stateCont -=1
        continue
    elif each[:2] == "--":
        continue
    q=0
    msg=[]
    sign=0
    decimal = 0
    dig = 0
    eva = 0
    for i in range(len(each)):
        if each[i]==" ":
            if decimal:
                q = q/10**dig
            if sign:
                q = -q
            if eva!=0:
                q = q/10**eva
            msg.append(q)
            q=0
            dig=0
            decimal=0
            sign=0
            eva=0
        elif each[i]=="-":
            sign=1
        elif each[i]==".":
            decimal=1
            dig = 0
        elif each[i]=="\n":
            if decimal:
                q = q/10**dig
            if sign:
                q = -q
            msg.append(q)
            q=0
            dig=0
            decimal=0
            sign=0
        elif each[i]=="[" or each[i]=="]":
            continue
        elif each[i]=='e':
            i = i+3
            eva = int(each[i])
        else:
            q = q*10 + int(each[i])
            dig += 1
    
    # msg.append(q)
    
    print(msg)
    x.append(msg[0])
    y.append(msg[1])
    z.append(msg[2])
    
    roll.append(msg[3])
    pitch.append(msg[4])
    yaw.append(msg[5])
    throttle.append(msg[6])
    
    errY.append(msg[7])
    errX.append(msg[8])
    errZ.append(msg[9])
    
    stateYaw.append(msg[10])
    stateRoll.append(msg[11])
    statePitch.append(msg[12])
    
    errX_with_sse.append(msg[13])
    errY_with_sse.append(msg[14])
    errZ_with_sse.append(msg[15])

    err_diffX.append(msg[16])
    err_diffY.append(msg[17])
    err_diffZ.append(msg[18])

    err_intX.append(msg[19])
    err_intY.append(msg[20])
    err_intZ.append(msg[21])

    vel.append(msg[22])


import matplotlib.pyplot as plt

from datetime import datetime
from os.path import join
from configparser import ConfigParser
from os import makedirs
# import yaml
import shutil


folder_name = datetime.now().strftime("%m-%d, %H:%M:%S")
folder_name = join("graphs",folder_name)
makedirs(folder_name)

# config = yaml.load(open("controls/config.yaml"), Loader=yaml.FullLoader)

# with open(join(folder_name,"config.yaml"),'w') as outfile:
#     yaml.dump(config,outfile,default_flow_style=False)
config = ConfigParser()
config.read("controls/droneData.ini")
with open(join(folder_name,"DroneData.ini"), 'w') as configfile:
    config.write(configfile)

shutil.copy2('debug.txt',folder_name)




plt.plot(roll)
plt.savefig(f"{folder_name}/roll.png")
plt.clf()

plt.plot(pitch)
plt.savefig(f"{folder_name}/pitch.png")
plt.clf()

plt.plot(yaw)
plt.savefig(f"{folder_name}/yaw.png")
plt.clf()

plt.plot(throttle)
plt.savefig(f"{folder_name}/throttle.png")
plt.clf()

plt.plot(x)
plt.savefig(f"{folder_name}/x.png")
plt.clf()

plt.plot(y)
plt.savefig(f"{folder_name}/y.png")
plt.clf()

plt.plot(z)
plt.savefig(f"{folder_name}/z.png")
plt.clf()


plt.plot(errX)
plt.savefig(f"{folder_name}/errX.png")
plt.clf()

plt.plot(errY)
plt.savefig(f"{folder_name}/errY.png")
plt.clf()

plt.plot(errZ)
plt.savefig(f"{folder_name}/errZ.png")
plt.clf()

plt.plot(stateYaw)
plt.savefig(f"{folder_name}/stateYaw.png")
plt.clf()

plt.plot(stateRoll)
plt.savefig(f"{folder_name}/stateRoll.png")
plt.clf()

plt.plot(statePitch)
plt.savefig(f"{folder_name}/statePitch.png")
plt.clf()

plt.plot(errX_with_sse)
plt.savefig(f"{folder_name}/errX_with_sse.png")
plt.clf()

plt.plot(errY_with_sse)
plt.savefig(f"{folder_name}/errY_with_sse.png")
plt.clf()

plt.plot(errZ_with_sse)
plt.savefig(f"{folder_name}/errZ_with_sse.png")
plt.clf()

plt.plot(x,y)
plt.savefig(f"{folder_name}/x vs y.png")
plt.clf()

plt.plot(err_diffX)
plt.savefig(f"{folder_name}/err_diffX.png")
plt.clf()

plt.plot(err_diffY)
plt.savefig(f"{folder_name}/err_diffY.png")
plt.clf()

plt.plot(err_diffZ)
plt.savefig(f"{folder_name}/err_diffZ.png")
plt.clf()

plt.plot(err_intX)
plt.savefig(f"{folder_name}/err_intX.png")
plt.clf()

plt.plot(err_intY)
plt.savefig(f"{folder_name}/err_intY.png")
plt.clf()

plt.plot(err_intZ)
plt.savefig(f"{folder_name}/err_intZ.png")
plt.clf()

plt.plot(vel)
plt.savefig(f"{folder_name}/vel.png")
plt.clf()
