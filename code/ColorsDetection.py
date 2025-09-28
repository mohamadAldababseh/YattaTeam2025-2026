import cv2
import numpy as np

class ColorsCoordinations:

    def __init__(self,minsize=200,show=1,name="green",l=[35,92,17],u=[84,229,142]):
        # GPIO.setwarnings(False)
        self.show=show
        self.minsize=minsize
        self.name=name
        self.l=l
        self.u=u
        
        
    def getcoor(self,imageFrame):
        x=-1
        y=-1
        h=-1
        w=-1
        hsv_frame2 = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
        kernal = np.ones((5, 5), "uint8") 
        #dilated_frame = cv2.dilate(hsv_frame2, kernal)
        #dilated_frame = cv2.erode(dilated_frame, kernal)

        lower = np.array([self.l[0], self.l[1], self.l[2]], np.uint8) 
        upper = np.array([self.u[0], self.u[1], self.u[2]], np.uint8) 
        mask = cv2.inRange(hsv_frame2, lower, upper) 

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        if len(contours)>0:
            c = max(contours, key = cv2.contourArea)
            area = cv2.contourArea(c)
            if(area > self.minsize):
                cv2.drawContours(imageFrame, c, -1, (0, 255, 0), 2)
                x, y, w, h = cv2.boundingRect(c)

    

        if self.show:
            cv2.imshow(self.name,imageFrame)
            key = cv2.waitKey(1) & 0xFF
        return x,y,w,h