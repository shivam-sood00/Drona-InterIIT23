import pandas as pd
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
print(folder_name)

# config = yaml.load(open("controls/config.yaml"), Loader=yaml.FullLoader)

# with open(join(folder_name,"config.yaml"),'w') as outfile:
#     yaml.dump(config,outfile,default_flow_style=False)
config = ConfigParser()
config.read("controls/droneData.ini")
with open(join(folder_name,"DroneData.ini"), 'w') as configfile:
    config.write(configfile)

# shutil.copy2('../vision_debug.txt',folder_name)

shutil.copy2('../vision_debug.csv',folder_name)


df = pd.read_csv("../vision_debug.csv")
plt.plot(df['aruco_x'])
plt.plot(df['x'])
plt.savefig(f"{folder_name}/x.png")
plt.clf()

plt.plot(df['aruco_y'])
plt.plot(df['y'])
plt.savefig(f"{folder_name}/y.png")
plt.clf()