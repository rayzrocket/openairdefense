from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

camera = PiCamera()
camera.resolution =(640,480)
camera.framerate=32
rawCapture = PiRGBArray(camera, size=(640,480))

time.sleep(0.1)

framewidth = int(640)
frameheight = int(480)
width=260
tpoint=(int(framewidth/2), int(frameheight/2))

SelctPtValB=15
halfx=80

for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
    start = time.time()
    image=frame.array

    bx1=int(tpoint[0]-(width/2))
    by1=int(tpoint[1]-(width/2))
    bx2=int(tpoint[0]+(width/2))
    by2=int(tpoint[1]+(width/2))
    if bx1<0:
        bx1=int(0)
        bx2=int(bx1+width)
    if by1<0:
        by1=int(0)
        by2=int(by1+width)
    if bx2>(framewidth-1):
        bx2=int((framewidth-1))
        bx1=int(bx2-width)
    if by2>(frameheight-1):
        by2=int((frameheight-1))
        by1=int(by2-width)
    box=(int(bx1),int(by1),int(bx2),int(by2))#old box
    subframe=image[int(by1):int(by2),int(bx1):int(bx2)]
    subframeB=subframe[:,:,0]
    
    LoPassBVal=int(SelctPtValB+halfx)
    if LoPassBVal>254:
        LoPassBVal=254
    HiPassBVal=int(SelctPtValB-halfx)
    if HiPassBVal<2:  #this sequence ends with inverted image because we track a 'dark' object on a bright background
        HiPassBVal=2  #first do HiPass shift down and then invert to add LoPass, this sequence matters!
    HiPass=subframeB-HiPassBVal#shift it down
    HiPass=np.clip(HiPass,0,255)#clip all neg values to 0
    LoPass=halfx-HiPass#invert image and shift up; all values more than x+val become zero
    BandPass=np.clip(LoPass,0,255)
    sumBx=np.sum(BandPass,axis=0)
    sumBy=np.sum(BandPass,axis=1)
    if sumBx.size > 1:
        sBxMax=np.argmax(sumBx)#pixel x location where min occurs
        if sumBy.size > 1:
            sByMax=np.argmax(sumBy)
            tpoint=(int(sBxMax+bx1),int(sByMax+by1))
    cv2.rectangle(image,(int(box[0]),int(box[1])),(int(box[2]),int(box[3])),(0,0,255),2)
    cv2.line(image,(int(tpoint[0]),0),(int(tpoint[0]),int(image.shape[0]-1)),(0,255,0),1)
    cv2.line(image,(0,int(tpoint[1])),(int(image.shape[1]-1),int(tpoint[1])),(0,255,0),1)
    cv2.imshow("Frame", image)#cv2.imshow("Frame", blue)
    cv2.imshow("Bandpass", BandPass)
    print(time.time()-start)
    key=cv2.waitKey(1) & 0xFF
    rawCapture.truncate(0)
    if key == ord("q"):
        break