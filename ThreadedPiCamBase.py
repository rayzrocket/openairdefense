#20ms
import io
import time
import threading
import picamera
import picamera.array#!!add this for numpy image output
import cv2
from PIL import Image
import numpy as np
from os import system
import struct

# Create a pool of image processors
done = False
lock = threading.Lock()
pool = []
globalPicCounter = 0

class ImageProcessor(threading.Thread):
    def __init__(self):
        super(ImageProcessor, self).__init__()
        self.picNmbrMax = 10
        self.iostream = [io.BytesIO() for x in range(0, self.picNmbrMax)]#io.BytesIO()
        self.event = threading.Event()
        self.terminated = False
        self.start()
        self.streamIndex = 0
        self.imgNmbr = []

    def run(self):
        # This method runs in a separate thread
        global done
        global globalPicCounter
        global imgdata
        start=time.time()
#        ssize=921600
#        a=0
        while not self.terminated:
            # Wait for an image to be written to the stream
            if self.event.wait(1):
                globalPicCounter += 1
                tmp = globalPicCounter
                self.imgNmbr.append(tmp)        
                print ('taking an image '+str(tmp)+'  imgNmbr='+str(self.imgNmbr))
                # Reset the stream and event
                self.iostream[self.streamIndex].seek(0)#self.stream.seek(0)
                self.streamIndex += 1
#                    self.stream.truncate()#do not truncate because we want to keep the image to save off later
                print("streamIndex = "+str(self.streamIndex))
                if self.streamIndex >= self.picNmbrMax:
                    # Set done to True if you want the script to terminate
                    done = True
                self.event.clear()
                # Return ourselves to the pool
                with lock:
                    pool.append(self)
                print(time.time()-start, globalPicCounter)
                start=time.time()


def streams():
    print("start the program")
    time.sleep(0.5)
    while not done:
        with lock:
            if pool:
                processor = pool.pop()
            else:
                processor = None
        if processor:
            yield processor.iostream[processor.streamIndex]
            processor.event.set()
        else:
            # When the pool is starved, wait a while for it to refill
            print("**************************pool is starved**************************")
            time.sleep(0.01)

with picamera.PiCamera() as camera:
#    camera.start_preview()
    pool = [ImageProcessor() for n in range(2)]#4)]
    camera.resolution = (640,480)
    camera.framerate = 90
    camera.iso=300
#    camera.raw_format = 'rgb'
    time.sleep(2)#allow auto gain to settle
    camera.shutter_speed=camera.exposure_speed
    camera.exposure_mode='off'
    camera.awb_gains=7#0 to 8
#    camera.start_preview()
    time.sleep(1)
#    camera.capture_sequence(streams(),'jpeg',use_video_port=True)
    camera.capture(streams(),'yuv',use_video_port=True)

# Shut down the processors in an orderly fashion
while pool:
    with lock:
        processor = pool.pop()
    for index, nmbr in enumerate(processor.imgNmbr):
        try:
            print('save image for: index='+str(index)+'  nmbr='+str(nmbr))
            img = Image.open(processor.iostream[index])
            if nmbr < 10:
                img.save("out000" + str(nmbr) + ".jpg")
                #cv2.imwrite('out000'+str(nmbr)+'.jpg',img)
            elif nmbr<100:
                img.save("out00" + str(nmbr) + ".jpg")
                #cv2.imwrite('out00'+str(nmbr)+'.jpg',img)
            else:
                img.save("out0" + str(nmbr) + ".jpg")
                #cv2.imwrite('out0'+str(nmbr)+'.jpg',img)
        except:
            print("couldn't do it "+str(index))
    processor.terminated = True
    processor.join()
#    
#    try:
##                    globalPicCounter += 1
##                    tmp = globalPicCounter
##                    self.imgNmbr.append(tmp)
##                    
#
##                    
##                    try:
##                        a=int(0)
##                        start=time.time()
##                        for index, entry in enumerate(self.grid):
##                            a=int(1+a)
##                            self.iostream.seek(entry)
##                            start=time.time()
##    #                        imgdata=struct.unpack('B'*ssize,self.iostream[self.streamIndex].read(ssize))
##                            if struct.unpack('B', self.iostream.read(1))[0] > self.threshold:
##                                self.objPosX = self.indexMapX[index]
##                                self.objPosY = self.indexMapY[index]
##                            if a > 10:
##                                break
##                        print(str(time.time()-start)+" Time to check stream ***")
##                    except:
##                        print("can't read buffer")
###                        
#                    print ('taking an image '+str(tmp)+'  imgNmbr='+str(self.imgNmbr))
##                    start=time.time()
##                    for n in range(1,3000):
##                        x=n/7
#                
#                    # Read the image and do some processing on it
#
#                finally:
#                    # Reset the stream and event
#                    self.iostream[self.streamIndex].seek(0)#self.stream.seek(0)
#                    self.streamIndex += 1
##                    self.stream.truncate()#do not truncate because we want to keep the image to save off later
#                    print("streamIndex = "+str(self.streamIndex))
#                    if self.streamIndex >= self.picNmbrMax:
#                        # Set done to True if you want the script to terminate
#                        done = True
#                    self.event.clear()
#                    # Return ourselves to the pool
#                    with lock:
#                        pool.append(self)