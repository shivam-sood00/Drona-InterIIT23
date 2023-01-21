import cv2
from vision.vision_pipeline import VisionPipeline
import pyrealsense2 as rs
import numpy as np
import time
import cv2
from cv2 import aruco

line_pos = []


def mouse_click(event, x, y, 
                flags, param):
      
    # to check if left mouse 
    # button was clicked
    if event == cv2.EVENT_LBUTTONDOWN:
          
        # font for left click event
        font = cv2.FONT_HERSHEY_TRIPLEX
        LB = 'Left Button'

        line_pos.append([x, y])

        if (len(line_pos) == 2):
            pass
            # cv2.line(img, line_pos[0], line_pos[1], (0, 0, 255), 3)
            # print("SLOPE: ", (line_pos[1][1] - line_pos[0][1]) / (line_pos[1][0] - line_pos[0][0]))
        elif (len(line_pos) > 2):
            latest = line_pos[-1]
            line_pos.clear()
            line_pos.append(latest)
            # print("SLOPE: ", (line_pos[1][1] - line_pos[0][1]) / (line_pos[1][0] - line_pos[0][0]))

        # display that left button 
        # was clicked.

#from kalman_filter import KalmanFilter



# import csv

# used to record the time when we processed last frame
# prev_frame_time = 0

# # used to record the time at which we processed current frame
# new_frame_time = 0

# # open the file in the write mode
# f = open('vision_data.csv', 'w')

# # create the csv writer
# writer = csv.writer(f)

# We need to set resolutions.
# # so, convert them from float to integer.
# frame_width = 1920
# frame_height = 1080
   
# size = (frame_width, frame_height)
   
# # Below VideoWriter object will create
# # a frame of above defined 
# result = cv2.VideoWriter('Bhideo.avi', 
#                          cv2.VideoWriter_fourcc(*'MJPG'),
#                          30, size)


DEBUG = 1


if __name__ == '__main__':

    depth_res=(720, 1280)
    rgb_res=(1080, 1920)
    align_to="rgb"
    marker_size=3.62 #13.8
    marker_type=aruco.DICT_4X4_50
    required_marker_id = 6
    calib_file_path="vision/calib_data/MultiMatrix.npz"

    pipeline = VisionPipeline(depth_res, rgb_res, align_to, marker_size, marker_type, required_marker_id, calib_file_path, debug=DEBUG)
    pipeline.init_realsense()
    pipeline.init_aruco_detector()


    cv2.namedWindow("Image")
    cv2.setMouseCallback('Image', mouse_click)


    try:
        while True:

            aligned_frames = pipeline.get_frames()    
            color_frame = pipeline.extract_rgb(aligned_frames)
            depth_frame = pipeline.extract_depth(aligned_frames)

            if not depth_frame or not color_frame:
                continue



            depth_img = pipeline.to_image(depth_frame)
            color_img = pipeline.to_image(color_frame)
           
            if len(line_pos) == 2:
                cv2.line(color_img, line_pos[0], line_pos[1], (0, 0, 255), 3)
                cv2.putText(color_img, f"Slope: {(line_pos[1][1] - line_pos[0][1]) / (line_pos[1][0] - line_pos[0][0])}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                print("SLOPE: ", (line_pos[1][1] - line_pos[0][1]) / (line_pos[1][0] - line_pos[0][0]))   
            else:
                for i in range(len(line_pos)):
                    cv2.circle(color_img, (line_pos[i][0], line_pos[i][1]), 3, (0, 0, 255), -1)
         
            cv2.imshow("Image", color_img)

            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break



    finally:
        pipeline.stop()
        # f.close()
        
