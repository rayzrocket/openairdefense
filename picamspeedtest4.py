#CAPTURE_CONTINUOUS
#Try both PiRGBArray results in 53ms times
#PiArrayOutput(not worky)
#PiYUVArray results in 43ms times, not sure of images
#capture continuous method used
from picamera.array import PiRGBArray
from picamera.array import PiArrayOutput
from picamera.array import PiYUVArray
from picamera import PiCamera
import time
import cv2
import numpy as np
from matplotlib import pyplot as plt

camera = PiCamera()

framewidth = int(640)#int(96)#int(160)#int(320)#int(640)#int(1920)
frameheight = int(480)#int(64)#int(120)#int(240)#int(480)#int(1088)

camera.resolution =(framewidth,frameheight)
camera.framerate = 90
camera.sensor_mode = 7
time.sleep(2)
camera.shutter_speed = 2720 #int(camera.exposure_speed/4)
print(str(camera.shutter_speed))
camera.exposure_mode = 'off'
g = camera.awb_gains
camera.awb_mode = 'off'
camera.awb_gains = g
camera.drc_strength = 'off'
camera.flash_mode = 'off'
selfPiRGBCapture = PiRGBArray(camera, size=(framewidth,frameheight))
selfArrayCapture = PiArrayOutput(camera, size=(framewidth,frameheight))
selfPiYUVArray = PiYUVArray(camera, size=(framewidth,frameheight))

time.sleep(0.5)
  
i=int(0)
img = []
times = []
stime=time.time()
#for frame in camera.capture_continuous(selfPiRGBCapture, format='rgb', use_video_port=True):
#for frame in camera.capture_continuous(selfArrayCapture, format='rgb', use_video_port=True):
for frame in camera.capture_continuous(selfPiYUVArray, format='yuv', use_video_port=True):
    img.append(frame.array)
    times.append(time.time() - stime)
    #selfPiRGBCapture.truncate(0)#truncate to given number of bytes
    #selfArrayCapture.truncate(0)
    selfPiYUVArray.truncate(0)
    i=i+1
    if i==10:#if cv2.waitKey(50) == 27:
        break
    stime=time.time()
print(times)
for x in range(0,len(img)):
    cv2.imwrite('PiCamCapCont'+str(framewidth)+'x'+str(frameheight)+'_'+str(x)+'.jpg',img[x])