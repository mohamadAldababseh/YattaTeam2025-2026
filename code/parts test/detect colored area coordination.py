from ColorsDetection import  ColorsCoordinations as cc
import cv2 as cv
import time


green=cc()
#red=cc(200,1,"red",[52,69,17],[86,255,80])

cap = cv.VideoCapture(2)  # Webcam
# Set the resolution (width and height)


time.sleep(1)

while True:
        
    nframe=time.time()
    isSuccess, img = cap.read()
    img=cv.resize(img,(640,480))

    x,y,w,h=(green.getcoor(img))
    print(x+w*0.5,y+0.5*h,w*h)
    #print(red.getcoor(img))
    
    if cv.waitKey(20) & 0xFF == ord('q'):
        break
