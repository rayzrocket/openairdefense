#123ms on best setup
import picamera
import picamera.array
import time
import numpy as np
import cv2

width=640
height=480
fwidth = (width+31)//32*32#only for rounding errors, same if 640 used
fheight = (height+15)//16*16#similar to fwidth
    
with picamera.PiCamera(sensor_mode=7) as camera:
    camera.resolution =(fwidth,fheight)
    camera.framerate=90
    camera.sensor_mode = 7
    time.sleep(2)#wait for agc to settle to record gain for reuse
    camera.shutter_speed=2720
    camera.exposure_mode = 'off'
    g = camera.awb_gains
    camera.awb_mode = 'off'
    camera.awb_gains = g
    times = []
    img=[]
    for n in range(0,10):#1 thru 5
        stime=time.time()
        ydata=np.empty((fheight,fwidth),dtype=np.uint8)
        try:
            camera.capture(ydata,'yuv')
        except IOError:
            pass
        ydata=ydata[:fheight,:fwidth]
        times.append(time.time() - stime)#times used as dictionary t(n)
        img.append(ydata)
print(times)
for n in range(0,len(img)):
    cv2.imwrite('Capture'+str(n)+'.jpg',img[n])
        
#         if n==2:
#             output2 = np.empty((480, 640, 3), dtype=np.uint8)#output used as dictionary img(n)
#             camera.capture(output2, 'yuv')
#             times.append(time.time() - stime)#times used as dictionary t(n)
#         if n==3:
#             output3 = np.empty((480, 640, 3), dtype=np.uint8)#output used as dictionary img(n)
#             camera.capture(output3, 'yuv')
#             times.append(time.time() - stime)
#         if n==4:
#             output4 = np.empty((480, 640, 3), dtype=np.uint8)#output used as dictionary img(n)
#             camera.capture(output4, 'yuv')
#             times.append(time.time() - stime)
#         if n==5:
#             output5 = np.empty((480, 640, 3), dtype=np.uint8)#output used as dictionary img(n)
#             camera.capture(output5, 'yuv')
#             times.append(time.time() - stime)
    
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