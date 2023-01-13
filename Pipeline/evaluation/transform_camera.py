#!/usr/bin/env python3
import numpy as np
import csv
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import cv2

class debugTransform:
    def __init__(self, cam_rvec = np.array([-2.97019626, -0.3456304, 0.31979031]) , cam_tvec = np.array([31.5464837, -24.46613193, 277.88341232]),
    fx = 640.381164550781, cx = 631.432983398438, fy = 639.533020019531, cy = 409.294647216797):
        
        self.cam_rvec = np.array([ 2.03683066, 2.54700777, -0.12213006]) #cam_rvec
        self.cam_tvec = np.array([ 32.25636813,-25.42815553,278.15869858]) #cam_tvec
        cx = 906.3662801147559
        cy = 561.2820445300187
        fx = 1347.090250261588
        fy = 1332.103727995465
        self.cam_matrix = np.array([[fx,0,cx,0],[0,fy,cy,0],[0,0,1,0]])

    def readCSV(self,filename):
        csv_data = []
        file = open(filename, 'r')
        with file as csv_file:
            reader = csv.reader(csv_file)
            for item in reader:
                csv_data.append(item)
        return np.array(csv_data)
    
    def world_to_image(self,position):
        rot_mat = Rotation.from_rotvec(self.cam_rvec).as_matrix()
        tf = np.eye(4)
        tf[:3, :3] = rot_mat
        tf[:3, 3] = self.cam_tvec 
        pos = np.zeros(4)
        pos[3] = 1
        pos[0] = float(position[0])
        pos[1] = float(position[1])
        pos[2] = float(position[2])
        s_image_coord = self.cam_matrix @ tf @ pos 
        image_coord = s_image_coord/s_image_coord[2]
        return int(image_coord[0]),int(image_coord[1])

        
# a = debugTransform()
# data = a.readCSV('imp.csv')
# data = data[:,3:6]

# mask = np.zeros((1080, 1920), dtype=np.uint8)

# vid = cv2.VideoCapture("./imp.avi")

# path_ = []

# for position in data:
#     # print(position)
#     temp = a.world_to_image(position)
#     cv2.circle(mask, temp, 2, 255, -1)
# mask = mask / 255
# mask = mask.astype(bool)
#     # mask[temp[1]][temp[0]] = 1


# while True:
#     ret, frame = vid.read()

#     frame[mask] = (0, 0, 255)
  
#     # Display the resulting frame
#     cv2.imshow('frame', frame)
      
#     # the 'q' button is set as the
#     # quitting button you may use any
#     # desired button of your choice
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
  
# # After the loop release the cap object
# vid.release()
# # Destroy all the windows
# cv2.destroyAllWindows()

# # cv2.imshow("VIZ", img)
# # cv2.imwrite("aruco_pose.png", img)
# # cv2.waitKey(0)


