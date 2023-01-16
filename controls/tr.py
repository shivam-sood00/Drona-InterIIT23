
from configparser import ConfigParser
  
configur = ConfigParser()
print(configur.read('droneData.ini'))
  
print("Sections : ", configur.sections())
print("d0: wifi,id",configur.get("Drone 0","wifi"),configur.get("Drone 0","id"))
print("d1: wifi,id",configur.get("Drone 1","wifi"),configur.get("Drone 1","id"))
print("d2: wifi,id",configur.get("Drone 2","wifi"),configur.get("Drone 2","id"))
# print ("Installation Library : ", configur.get('installation','library'))
# print ("Log Errors debugged ? : ", configur.getboolean('debug','log_errors'))
# print ("Port Server : ", configur.getint('server','port'))
# print ("Worker Server : ", configur.getint('server','nworkers'))