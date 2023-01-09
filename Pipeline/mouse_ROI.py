import cv2 as cv
import numpy as np

ROI_list = []
global point
#just set up mouse callback in the file you will be using this class in
class ROI():
    def __init__(self,frame) -> None:
        self.frame = frame        
    
     

    def point_append(self,event,x,y,flags,param):        
        if event == cv.EVENT_LBUTTONDBLCLK:
            point = np.array([np.int32(x),np.int32(y)])        
            ROI_list.append(point)
            self.process(self.frame,point)    

    def process(self,frame,point):        
        x = point[0]
        y = point[1]    
        cv.circle(frame,(x,y),2,(0,0,255),-1)
        cv.putText(frame,'{}'.format((x,y)),(x,y-10),cv.FONT_HERSHEY_PLAIN,2,(0,255,0),1)
        if len(ROI_list)==4:
            self.draw_ROI(self.frame)

    def draw_ROI(self,frame):
        pnt = np.array([ROI_list[0],ROI_list[1],ROI_list[2],ROI_list[3]],np.int32)  
        cv.polylines(frame,[pnt],True,(255,0,0),2)
        #cv.rectangle(frame,(ROI_list[0][0],ROI_list[0][1]),(ROI_list[1][0],ROI_list[1][1]),(255,0,0),2)








