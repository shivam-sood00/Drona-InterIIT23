import cv2 as cv
import numpy as np
import csv
import pandas as pd
from matplotlib import pyplot as plt
from transform_camera import *
from draw_ROI import *
import math
from vision_pipeline import *

pipeline = VisionPipeline(rgb_res=(1080,1920))
pipeline.init_realsense()
pipeline.init_aruco_detector()

# f = open('vision_data.csv', 'r')
#df = pd.read_csv('vision_data.csv')

# reader = csv.reader(f)

point_3d_list = []
point_2d_list = []
point_list = []
mc_list = []
instance1 = debugTransform()
#csv_data_3d = instance1.readCSV()
csv_data_2d = []
csv_data_3d = []
csv_data = []
error_data = []
param_list = []
img = np.zeros((1080,1920,3), np.uint8)
instance2 = ROI()


def append_point_3d_list(list_3d):
    n_row = 4
    n_col = 3

    for i in range(n_row):
        point_3d = []
        # print("Enter point")
        # x,y,z = input().split()
        # point_3d = [x,y,z]
        # print(point_3d)
        list_3d = [[-50,-50,0],[-50,50,0],[50,50,0],[50,-50,0]]
    
    return list_3d

    # print(point_3d_list)
    
def convert_3d_to_2d(list_3d,list_2d,p_list):    
    for p in list_3d:
        x,y = instance1.world_to_image(p) 
        p_list.append([x,y])       
        point_2d = np.array([np.int32(x),np.int32(y)])
        list_2d.append(point_2d)
    
    return list_2d,p_list
    # print(point_2d_list)    
    
def draw_ROI_from_points(list_2d,frame):
    for c in list_2d:        
        instance2.process(frame,c)
    instance2.draw_ROI(frame,list_2d)

def create_line_from_points_2d(point1,point2):
    print(point1)
    m = (point2[1]-point1[1])/(point2[0]-point1[0])
    c = (point1[1] - m*point1[0])
    return m,c

def find_perpendicular_distance(line_param,pnt):
    num = float(pnt[1]-line_param[0]*pnt[0]-line_param[1])
    den = math.sqrt(line_param[0]**2 + 1)
    dist = (num/den)
    return dist

def create_line_list(line_param,p_list):
    
    m,c = create_line_from_points_2d(p_list[0],p_list[1])
    line_param.append([m,c])
    m,c = create_line_from_points_2d(p_list[1],p_list[2])
    line_param.append([m,c])
    m,c = create_line_from_points_2d(p_list[2],p_list[3])
    line_param.append([m,c])
    m,c = create_line_from_points_2d(p_list[0],p_list[3])
    line_param.append([m,c])

    return line_param
    
 
def find_error(frame,csv_2d,line_param):
    error_list = []
    for d in csv_2d:
        cv.circle(frame,(np.int32(d[0]),np.int32(d[1])),5,(255,0,0),-1)
        dist_array = []
        for data in line_param:
            dist_array.append(find_perpendicular_distance(data,d))
        k = min(dist_array)
        
        error_list.append(k)
    return error_list

def find_error_3d(csv_3d,line_param):
    error_list = []
    for d in csv_3d:
        
        dist_array = []
        for data in line_param:
            dist_array.append(find_perpendicular_distance_3d(data,d))
        k = min(dist_array)
        
        error_list.append(k)
    return error_list

def RMS_error(e_list):
    MS = np.square(e_list).mean()
    RMS = np.sqrt(MS)
    return RMS

def create_line_from_points_3d(point1,point2):
    l = point1[0] - point2[0]
    m = point1[1] - point2[1]
    n = point1[2] - point2[2]
    
    params = [point1[0],point1[1],point1[2],l,m,n]
    return params

def create_line_list_3d(list_3d,line_param_3d):
    params = create_line_from_points_3d(list_3d[0],list_3d[1])
    line_param_3d.append([params])
    params = create_line_from_points_3d(list_3d[1],list_3d[2])
    line_param_3d.append([params])
    params = create_line_from_points_3d(list_3d[2],list_3d[3])
    line_param_3d.append([params])
    params = create_line_from_points_3d(list_3d[0],list_3d[3])
    line_param_3d.append([params])
    return line_param_3d

def find_perpendicular_distance_3d(line_param,pnt):
    dist_array = []
    print(pnt)
    for pt in line_param:
        print(pt)
        vec_AP = np.array([pnt[0]-pt[0],pnt[1]-pt[1],pnt[2]-pt[2]])
        vec_d =  np.array([pt[0],pt[1],pt[2]])
        cross_p = np.cross(vec_AP,vec_d)        
        mag_cross_p = np.sqrt(cross_p.dot(cross_p))
        mag_d = np.sqrt(vec_d.dot(vec_d))
        dist = mag_cross_p/mag_d

    dist_array.append(dist)
    k = min(dist_array)
    return k

def csv_reader():
    csv_3d_pnt = []
    csv_2d_pnt = []
    csv_pnt = []
    
    csv_file = pd.read_csv('vision_data.csv')

    for lines in csv_file:
        if(len(lines)!=0):
            data = [float(lines[3]),float(lines[4]),float(lines[2])]
            csv_3d_pnt.append(data)
            print(data)
            
    
    csv_2d_pnt,csv_pnt = convert_3d_to_2d(csv_3d_pnt,csv_2d_pnt,csv_pnt)
    # print(csv_2d_pnt)
    return csv_2d_pnt,csv_3d_pnt


    

point_3d_list = append_point_3d_list(point_3d_list)
point_2d_list, point_list= convert_3d_to_2d(point_3d_list,point_2d_list,point_list)
csv_data_2d,csv_data_3d = csv_reader()
mc_list = create_line_list(mc_list,point_2d_list)
param_list = create_line_list_3d(point_3d_list,param_list)
#check error from csv

while True:
    aligned_frames = pipeline.get_frames()
    depth_frame = pipeline.extract_depth(aligned_frames)
    color_frame = pipeline.extract_rgb(aligned_frames)
    color_img = pipeline.to_image(color_frame)
    image = pipeline.to_image(depth_frame)
    draw_ROI_from_points(point_2d_list,color_img)
    x,y = instance1.world_to_image([0,0,0])
    cv.circle(color_img, (x,y),2,(255,0,0),-1)
    cv.putText(color_img,'{}'.format((x,y)),(x,y-10),cv.FONT_HERSHEY_PLAIN,1,(0,255,0),1)
    error_data = find_error(color_img,csv_data_2d,mc_list)
    error_data = find_error_3d(csv_data_3d,param_list)
    RMS = RMS_error(error_data)    
    cv.imshow("frame", color_img)
    key = cv.waitKey(1)
    if key == ord("q"):
        break
print(RMS)
cv.destroyAllWindows()
pipeline.stop()

