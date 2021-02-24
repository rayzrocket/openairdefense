#33ms  camera.capture(output,'yuv', use_video_port=True)
import cv2
from picamera import PiCamera
import io
import time
import numpy as np

camera = PiCamera()
stream = io.BytesIO()

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

for n in range(0,10):
    stime=time.time()
    output = np.empty((480, 640, 3), dtype=np.uint8)#output used as dictionary img(n)
    camera.capture(output,'yuv', use_video_port=True)
    img.append(output)
    times.append(time.time() - stime)#times used as dictionary t(n)
print(times)
for n in range(0,len(img)):
    cv2.imwrite('picture'+str(n)+'.jpg',img[n])