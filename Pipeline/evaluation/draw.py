import cv2 as cv
import numpy as np
import csv
from matplotlib import pyplot as plt
from transform_camera import *
from draw_ROI import *
import math


point_3d_list = []
point_2d_list = []
point_list = []
mc_list = []
instance1 = debugTransform()
#csv_data_3d = instance1.readCSV()
csv_data_2d = []
csv_data = []
error_data = []
img = np.zeros((1080,1920,3), np.uint8)
instance2 = ROI(img)


def append_point_3d_list(list_3d):
    n_row = 4
    n_col = 3

    for i in range(n_row):
        point_3d = []
        print("Enter point")
        for j in range(n_col):
            point_3d.append(int(input()))
        # print(point_3d)
        list_3d.append(point_3d)
    
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
    
def draw_ROI_from_points(list_2d):
    for c in list_2d:        
        instance2.process(img,c)

def create_line_from_points_2d(point1,point2):
    m = float(point2[1]-point1[1])/float(point2[0]-point1[0])
    c = float(point1[1] - m*point1[0])
    return m,c

def find_perpendicular_distance(line_param,pnt):
    num = float(pnt[1]-line_param[0]*pnt[0]-line_param[1])
    den = math.sqrt(line_param[0]**2 + 1)
    dist = (num/den)
    return dist

def create_line_list(line_param):
    for p1, p2 in point_list:
        m,c = create_line_from_points_2d(p1,p2)
        line_param.append([m,c])

    m,c = create_line_from_points_2d(point_list[0],point_list[3])
    line_param.append([m,c])
    return line_param
    
 
def find_error(frame,csv_2d,line_param):
    for d in csv_2d:
        cv.circle(frame,(np.int32(d[0]),np.int32(d[1])),2,(0,255,0),-1)
        dist_array = []
        for data in line_param:
            dist_array.append(find_perpendicular_distance(data,d))
        k = min(dist_array)
        error_data.append(k)

def RMS_error(e_list):
    MS = np.square(e_list).mean()
    RMS = np.sqrt(MS)
    return RMS

def create_line_from_points_3d(point1,point2):
    l = point1[0] - point2[0]
    m = point1[1] - point2[1]
    n = point1[2] - point2[2]
    
    params = [[point1],l,m,n]
    return params

def create_line_list_3d(list_3d,line_param_3d):
    for p1,p2 in list_3d:
        params = create_line_from_points_3d(p1,p2)
        line_param_3d.append(params)
    
    params = create_line_from_points_3d(list_3d[0],list_3d[3])
    line_param_3d.append(params)
    return line_param_3d

def find_perpendicular_distance_3d(line_param,pnt):
    dist_array = []
    for pt in line_param:
        vec_AP = np.array([pnt[0]-pt[0],pnt[1]-pt[1],pnt[2]-pt[2]])
        vec_d = np.array([pt[0],pt[1],pt[2]])
        cross_p = np.array(np.cross(vec_AP,vec_d))
        mag_cross_p = np.sqrt(cross_p.dot(cross_p))
        mag_d = np.sqrt(vec_d.dot(vec_d))
        dist = mag_cross_p/mag_d

    dist_array.append(dist)
    k = min(dist_array)
    return k

point_3d_list = append_point_3d_list(point_3d_list)
point_2d_list, point_list = convert_3d_to_2d(point_2d_list,point_list)
draw_ROI_from_points(point_2d_list)

cv.imshow('image',img)
cv.waitKey(0)
cv.destroyAllWindows()