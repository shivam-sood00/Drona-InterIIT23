# a file named "geek", will be opened with the reading mode.
import time
file = open('debug.txt', 'r')
# This will print every line one by one in the file
x=[]
y=[]
z=[]

errX=[]
errY=[]
errZ=[]

targetX=[]
targetY=[]
targetZ=[]


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
    
    
    errX.append(msg[3])
    errY.append(msg[4])
    errZ.append(msg[5])

    targetX.append(msg[6])
    targetY.append(msg[7])
    targetZ.append(msg[8])


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



plt.plot(x, label='GT')
plt.plot(targetX, '-', label='Target')
plt.legend()
plt.savefig(f"{folder_name}/x.png")
plt.clf()

plt.plot(y, label='GT')
plt.plot(targetY, '-', label='Target')
plt.legend()
plt.savefig(f"{folder_name}/y.png")
plt.clf()

plt.plot(z, label='GT')
plt.plot(targetZ, '-', label='Target')
plt.legend()
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


plt.plot(x,y)
plt.savefig(f"{folder_name}/x vs y.png")
plt.clf()