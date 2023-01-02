#!/usr/bin/env python3
import cv2 as cv
from cv2 import aruco
import numpy as np
import time

cam_mat = np.array(((1448.722297970256, 0, 997.8480046779201),(0, 1448.473854721084, 460.7652079381467),(0,0,1.0)))
dist_coef = np.array((0.06032833802282229, 0.04535336453420969, -0.01767781240241698, 0.003250916898203592, -0.2271953059695576, 0, 0, 0))

MARKER_SIZE = 3.7 # centimeters
marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters_create()
tVec = []
# used to record the time when we processed last frame
prev_frame_time = 0
 
# used to record the time at which we processed current frame
new_frame_time = 0

##################
##################

def inversePerspective(rvec, tvec):
  R, _ = cv.Rodrigues(rvec)
  R = np.matrix(R).T
  invTvec = np.dot(R, np.matrix(-tvec))
  invRvec, _ = cv.Rodrigues(R)
  return invRvec, invTvec
    
def relativePosition(TvecA,RvecA,TvecB,RvecB):
  RvecA, TvecA = RvecA.reshape((3, 1)), TvecA.reshape((3, 1))
  RvecB, TvecB = RvecB.reshape((3, 1)), TvecB.reshape((3, 1))
  invRvec, invTvec = inversePerspective(RvecB, TvecB)
        
  info = cv.composeRT(RvecA, TvecA, invRvec, invTvec)
  composedRvec, composedTvec = info[0], info[1]
  composedRvec = composedRvec.reshape((3, 1))
  composedTvec = composedTvec.reshape((3, 1))

  return composedRvec, composedTvec
  
  
#cap = cv.VideoCapture(7)
cap = cv.VideoCapture("./4.mp4")
#cap.set(cv.CAP_PROP_FRAME_WIDTH,1920)
#cap.set(cv.CAP_PROP_FRAME_HEIGHT,1080)
#cap.set(cv.CAP_PROP_FPS,30)
#focus = 255  # min: 0, max: 255, increment:5
#cap.set(28, focus)
#cap.set(cv.CAP_PROP_EXPOSURE, 0) 
frame_detected = 0
total_frame = 0; 
while True:
    
    ret, frame = cap.read()
    if not ret:
        print(frame_detected)
        print(total_frame)
        break
    total_frame = total_frame + 1
        
    ####################
    kernel = np.array([[0, -1, 0],
                   [-1, 5,-1],
                   [0, -1, 0]])
    frame = cv.filter2D(src=frame, ddepth=-1, kernel=kernel)
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    
    

    
    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers
    )
    if marker_corners:
        frame_detected = frame_detected + 1
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
            marker_corners, MARKER_SIZE, cam_mat, dist_coef
        )
        total_markers = range(0, marker_IDs.size)
        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            cv.polylines(
                frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
            )
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            top_right = corners[0].ravel()
            top_left = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left = corners[3].ravel()


            distance = np.sqrt(
                tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
            )
            # Draw the pose of the marker
            point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
            cv.putText(
                frame,
                f"id: {ids[0]} Dist: {round(distance, 2)}",
                top_right,
                cv.FONT_HERSHEY_PLAIN,
                1.3,
                (0, 0, 255),
                2,
                cv.LINE_AA,
            )
            cv.putText(
                frame,
                f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
                bottom_right,
                cv.FONT_HERSHEY_PLAIN,
                1.0,
                (0, 0, 255),
                2,
                cv.LINE_AA,
            )
            # print(ids, "  ", corners)
            # print("Rotation Vector = " , rVec , ""
            #                                     ""
            #                                     ""
            #                                     "")
            # print("Translation Vector = " , tVec , ""
            #                                     ""
            #                                     ""
            #                                     "")
            
    # print(len(tVec))
    if len(tVec) == 2:
      tranRvec,tranTvec = relativePosition(tVec[1],rVec[1],tVec[0],rVec[0])
      print("Rotation Vector = " , tranRvec , ""
                                           ""
                                         ""
                                         "")
      print("Translation Vector = " , tranTvec , ""
                                         ""
                                         ""
                                         "")
    cv.imshow("frame", frame)
    #cv.imshow("gframe", gray_frame)
    # time when we finish processing for this frame
    new_frame_time = time.time()
    fps = 1/(new_frame_time-prev_frame_time)
    prev_frame_time = new_frame_time
    fps = int(fps)
    
    
    key = cv.waitKey(1)
    if key == ord("q"):
        break
    #print(cap.get(cv.CAP_PROP_FPS))
    print(fps)
cap.release()
cv.destroyAllWindows()
