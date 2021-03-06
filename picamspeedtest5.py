#12ms Pi4b camera.capture(output,'raw', use_video_port=True)
#33ms Pi3b camera.capture(output,'raw', use_video_port=True)
import cv2
from picamera import PiCamera
import io
import time
import numpy as np
import os #for CPU temperature
from matplotlib import pyplot as mplt
from matplotlib import image as mimg
import gc

camera = PiCamera()
stream = io.BytesIO()

framewidth = int(640)#int(96)#int(160)#int(320)#int(640)#int(1920)
frameheight = int(480)#int(64)#int(120)#int(240)#int(480)#int(1088)
#when using 80x60, ..picamera/encoders.py:544: PiCameraResolutionRounded: frame size rounded up from 80x60 to 96x64
#frame size rounded up from 1920x1080 to 1920x1088

#----Function to measure CPU temp  use print(measure_temp())
def measure_temp():
        temp = os.popen("vcgencmd measure_temp").readline()
        return (temp.replace("temp=",""))
    
camera.resolution =(framewidth,frameheight)#(640,480)
camera.framerate = 90
camera.sensor_mode = 7
time.sleep(2)
camera.shutter_speed = 2720#2720 #int(camera.exposure_speed/4) #print(str(camera.shutter_speed))#because may hit max or min by software, check.
camera.exposure_mode = 'off'
g = camera.awb_gains
camera.awb_mode = 'off'
camera.awb_gains = g
camera.drc_strength = 'off'
camera.flash_mode = 'off'

img = [] #list for image store
times = [] #list for timing store
stime=time.time()

for n in range(1,42):
    output = np.empty((frameheight,framewidth, 3), dtype=np.uint8)#output recorded in list img(n)
    stime=time.time()
    camera.capture(output,'raw', use_video_port=True)
    times.append(time.time() - stime)#times used as list t(n)
    #do manual image processing here
    img.append(output)
#     gc.collect()
    
m=n#copied n to m here

ntimes=[]
for x in range(1,m):#record frames a jpg's into folder to review later
    ntimes.append(float("%0.4f" % (times[x-1])))#timesrnd=[round(a,3) for a in times]
    cv2.imwrite('PiCam'+str(framewidth)+'x'+str(frameheight)+'_'+str(x)+'.jpg',img[x])
print(ntimes, measure_temp(), "Last frame:  dims=", output.ndim, "shape=", output.shape, "dtype=", output.dtype)
# sample=img[8]#show a sample image on screen
# sample2=sample[:,:,0]#show a sample image on screen
# cv2.imshow("SamplE",sample2)#show a sample image on screen
# cv2.waitKey()#show a sample image on screen