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

# config = yaml.load(open("controls/config.yaml"), Loader=yaml.FullLoader)

# with open(join(folder_name,"config.yaml"),'w') as outfile:
#     yaml.dump(config,outfile,default_flow_style=False)
config = ConfigParser()
config.read("controls/droneData.ini")
with open(join(folder_name,"DroneData.ini"), 'w') as configfile:
    config.write(configfile)

# shutil.copy2('debug.txt',folder_name)

shutil.copy2('../debug.csv',folder_name)


df = pd.read_csv("../debug.csv")
plt.scatter(df['x1'],df['y1'])
plt.savefig(f"{folder_name}/xvsy1.png")
plt.clf()

plt.scatter(df['x2'],df['y2'])
plt.savefig(f"{folder_name}/xvsy2.png")
plt.clf()