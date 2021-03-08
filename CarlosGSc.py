#https://gist.githubusercontent.com/CarlosGS/b8462a8a1cb69f55d8356cbb0f3a4d63/raw/bf83517820e12f882218db53b0b21c21015e41f7/raspberry_fast_capture.py
# Fast reading from the raspberry camera with Python, Numpy, and OpenCV
# Allows to process grayscale video up to 124 FPS (tested in Raspberry Zero Wifi with V2.1 camera)
#
# Made by @CarlosGS in May 2017
# Club de Robotica - Universidad Autonoma de Madrid
# http://crm.ii.uam.es/
# License: Public Domain, attribution appreciated

import cv2
import numpy as np
import subprocess as sp
import time
import atexit
import sys

frames = [] # stores the video sequence for the demo
max_frames = 300

N_frames = 0

# Video capture parameters
(w,h) = (1280,720)#(640,480)
bytesPerFrame = w * h
fps = 120 # setting to 250 will request the maximum framerate possible

# "raspividyuv" is the command that provides camera frames in YUV format
#  "--output -" specifies stdout as the output
#  "--timeout 0" specifies continuous video
#  "--luma" discards chroma channels, only luminance is sent through the pipeline
# see "raspividyuv --help" for more information on the parameters
#videoCmd = "raspividyuv -w "+str(w)+" -h "+str(h)+" --output - --timeout 0 --framerate "+str(fps)+" --luma --nopreview"
videoCmd = "raspividyuv -w "+str(w)+" -h "+str(h)+" --output - --timeout 0 --framerate "+str(fps)+" --luma --nopreview --mode 6 --exposure fixedfps"
#videoCmd = "raspivid -w "+str(w)+" -h "+str(h)+" -b 20000000 --output - --timeout 0 --framerate "+str(fps)+" --nopreview"
videoCmd = videoCmd.split() # Popen requires that each parameter is a separate string

cameraProcess = sp.Popen(videoCmd, stdout=sp.PIPE, stderr=sp.PIPE, bufsize=((w*h)+20)) # start the camera
atexit.register(cameraProcess.terminate) # this closes the camera process in case the python scripts exits unexpectedly

# wait for the first frame and discard it (only done to measure time more accurately)
times = []
sizeA=[]
sizeS=[]

for n in range(0,90):
    stime = time.time()
    rawStream = cameraProcess.stdout.read(bytesPerFrame)
#     print(sys.getsizeof(rawStream))
    cameraProcess.stdout.flush()
    if sys.getsizeof(rawStream) == ((w*h)+17):
        img=np.frombuffer(rawStream, dtype=np.uint8)
        img=img[0:bytesPerFrame]
        img.shape=(h,w)
        sizeS.append(sys.getsizeof(rawStream))
    #     rawStream = np.frombuffer(cameraProcess.stdout.read(76800))
        times.append(time.time() - stime)
        sizeA.append(img.size)
        frames.append(img)
print(times)
print("sizes:")
print(sizeS)
print(sizeA)

cameraProcess.terminate() # stop the camera

for nmbr in range(0,len(frames)):
#     start = time.time()
#     cv2.imwrite('Carlos'+str(n)+'.jpg',frames[n])
#     print(time.time()-start,' saveframe')#takes 24ms to write jpg
    if nmbr < 10:
        cv2.imwrite('out000'+str(nmbr)+'.jpg',frames[nmbr])
    elif nmbr<100:
        cv2.imwrite('out00'+str(nmbr)+'.jpg',frames[nmbr])
    else:
        cv2.imwrite('out0'+str(nmbr)+'.jpg',frames[nmbr])

