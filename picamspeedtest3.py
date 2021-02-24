#manual capture results in about 134ms
import picamera
import time
import cv2
import numpy as np

with picamera.PiCamera(sensor_mode=7) as camera:
    camera.resolution =(640,480)
    camera.framerate=90
    camera.sensor_mode = 7
    time.sleep(2)#wait for agc to settle to record gain for reuse
    camera.shutter_speed=2720
    camera.exposure_mode = 'off'
    g = camera.awb_gains
    camera.awb_mode = 'off'
    camera.awb_gains = g
    times = []
    for n in range(1,6):#1 thru 5
        stime=time.time()
        if n==1:
            output1 = np.empty((480, 640, 3), dtype=np.uint8)#output used as dictionary img(n)
            camera.capture(output1, 'yuv')
            times.append(time.time() - stime)#times used as dictionary t(n)
        if n==2:
            output2 = np.empty((480, 640, 3), dtype=np.uint8)#output used as dictionary img(n)
            camera.capture(output2, 'yuv')
            times.append(time.time() - stime)#times used as dictionary t(n)
        if n==3:
            output3 = np.empty((480, 640, 3), dtype=np.uint8)#output used as dictionary img(n)
            camera.capture(output3, 'yuv')
            times.append(time.time() - stime)
        if n==4:
            output4 = np.empty((480, 640, 3), dtype=np.uint8)#output used as dictionary img(n)
            camera.capture(output4, 'yuv')
            times.append(time.time() - stime)
        if n==5:
            output5 = np.empty((480, 640, 3), dtype=np.uint8)#output used as dictionary img(n)
            camera.capture(output5, 'yuv')
            times.append(time.time() - stime)
    print(times)
#    cv2.imshow('out4',output4)
#with picamera.PiCamera(sensor_mode=7) as camera:
#    start=time.time()
#    for i, filename in enumerate(camera.capture_continuous('image{counter:02d}.jpg')):
#        print(filename)
#        print(time.time()-start)
#        if i==20:
#            break
#        start=time.time()
#        #average loop time is 220ms !!!!

#
#camera = PiCamera()
#camera.resolution =(640,480)
#camera.sensor_mode=7
#camera.framerate=90
#rawCapture = PiRGBArray(camera, size=(640,480))
#
#time.sleep(0.1)
#
#framewidth = int(640)
#frameheight = int(480)
#width=260
#tpoint=(int(framewidth/2), int(frameheight/2))
#
#SelctPtValB=15
#halfx=80
#  
#i=int(0)
#start = time.time()
#for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
#    #*******************************************************************************************
#    image=frame.array
#    cv2.imwrite('picture'+str(i)+'.jpg',image)
#    rawCapture.truncate(0)
#    i=i+1
#    #*******************************************************************************************
#    print(time.time()-start)
#    if cv2.waitKey(50) == 27:
#        break
#    start=time.time()