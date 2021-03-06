#images don't move
#35ms camera.capture(stream,'yuv')  stream = open('image.data','w+b')
import cv2
from picamera import PiCamera
import io
import time
import numpy as np
import picamera

camera = PiCamera()
stream = io.BytesIO()
stream = open('image.data','w+b')

camera.resolution =(640,480)
camera.framerate = 90
camera.sensor_mode = 7
time.sleep(2)
# Now fix the values
camera.shutter_speed = 2720 #int(camera.exposure_speed/4)
print(str(camera.shutter_speed))
camera.exposure_mode = 'off'
g = camera.awb_gains
camera.awb_mode = 'off'
camera.awb_gains = g
camera.drc_strength = 'off'
camera.flash_mode = 'off'

i=int(0)
img = []
times = []
stime=time.time()
width=640
height=480
fwidth = (width+31)//32*32
fheight = (height+15)//16*16
temp=np.empty(0)

for n in range(0,10):
    stime=time.time()
    camera.capture(stream,'yuv', use_video_port=True)
    #data = np.fromstring(stream.getvalue(), dtype=np.uint8)
    stream.seek(0)#read from beginning of stream
    imagestr = np.fromfile(stream, dtype=np.uint8, count=fwidth*fheight)
    image = np.reshape(imagestr,(fheight,fwidth))
    #temp[:,:,0]=y[:,:]
    img.append(image)
    times.append(time.time() - stime)#times used as dictionary t(n)
print(times)
for n in range(0,len(img)):
    cv2.imwrite('picture'+str(n)+'.jpg',img[n])