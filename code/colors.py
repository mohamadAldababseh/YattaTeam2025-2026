from ColorsDetection import  ColorsCoordinations as cc
from WebcamVideoStream import WebcamVideoStream as ws
import cv2 as cv
import time
import threading

class colors:
    green=cc(200,1,"green",(71,133,136),(82,255,255))
    orange=cc(0,0,"orange",(0,101,55),(179,203,172))
    red=cc(200,1,"red",(164,20,128),(179,115,255))
    blue=cc(200,0,"blue",(111,75,59),(129,174,135))
    #red=cc(200,1,"red",[52,69,17],[86,255,80])
    orangedetected=False
    bluedetected=False
    cap=ws(0)
    cap.start()
    #cap = cv.VideoCapture(0)  # Webcam
    # Set the resolution (width and height)

    def detect_orange(self,i):
        print("deticting!")
        x,y,w,h=self.orange.getcoor(i)
        if not x==-1:
            self.orangedetected=True
    def detect_blue(self,i):
        x,y,w,h =self.blue.getcoor(i)
        if not x==-1:
            self.bluedetected=True
    def reset(self):       
        self.orangedetected=False
        self.bluedetected=False
    
        
    def detect_green(self,i):
        return (self.green.getcoor(i))
    def detect_red(self,i):
        return (self.red.getcoor(i))
     

    time.sleep(1)
    def loop(self):
        while True:
            print("start")
    
            img = self.cap.read()
            img=cv.resize(img,(640,480))
            print("done")
            startY = 350
            endY = 480
            startX = 0
            endX = 120

            # Crop the image
            ib = img[startY:endY, startX:endX]
            startX = 520
            endX = 640
            io = img[startY:endY, startX:endX]

            self.detect_orange(io)
            self.detect_blue(ib)

            #print(x+w*0.5,y+0.5*h,w*h)
            #print(red.getcoor(img))
            
            if cv.waitKey(20) & 0xFF == ord('q'):
                break

        
        
