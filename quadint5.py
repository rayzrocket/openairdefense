"""Quadcopter Interceptor project 
Press <esc> to quit.
"""
#recall: cv2.VideoCapture() does not work with with PiCamera on ribbon cable
#30 FPS on C920 Logitech webcam (this cam uses 15fps in low light automatically)
#6 to 14 ms with FullScale USB3A LWIR cam, no NUC, recall datasheet 14bit pixel arrays
#
#this is not using python enviroments #!/usr/bin/env python3
import cv2
import time
#from matplotlib import pyplot as plt
import os #for CPU temperature
import array #stock python array handling
import serial #serial port

readin = bytearray(25) #'bytearray'init the array bytes to read in
sendout = bytearray(25)
La = list(range(16)) #0 to 15 ; 16 positions
Ld = list(range(4)) #0 to 15 ; 16 positions
achannel = array.array('I',La) #initialize a basic python array as 'I' is unsigned integer, 2 bytes
digichannel = array.array('I',Ld) #initialize a basic python array
#mode = int #0 start / 1 user targeting / 2 auto track / 3 auto pilot
rstickhorz = int #right stick horz center bias (trim)
rstickvert = int #right stick vert center bias (trim)

# ---- Function definition for converting scales ------
def remap(unscaled, to_min, to_max, from_min, from_max):
    return (to_max-to_min)*(unscaled-from_min)/(from_max-from_min)+to_min

#----Function to measure CPU temp  use print(measure_temp())
def measure_temp():
        temp = os.popen("vcgencmd measure_temp").readline()
        return (temp.replace("temp=",""))

#---- Function Timer, returns current time
def how_long(start, activity):
    print('%s took %.3fs' % (activity, time.time()-start))
    return time.time()

def stop_loop(frame,cam):#break jumps us out of inner most loop and used in an if statement of the loop
    print(measure_temp())
    print(type(frame))
    print(frame.shape)
    print(frame.dtype)
    cv2.waitKey(100)
    cv2.destroyAllWindows()
    cam.release()
    
def parse_serin(readin):#Bitwise or is |, regular or is ||   #Bitwise and is &, regular and is &&
    achannel[0]  = ((readin[1]     | readin[2]<<8)                   & 0x07FF)
    achannel[1]  = ((readin[2]>>3  | readin[3]<<5)                   & 0x07FF)
    achannel[2]  = ((readin[3]>>6  | readin[4]<<2 | readin[5]<<10)   & 0x07FF)
    achannel[3]  = ((readin[5]>>1  | readin[6]<<7)                   & 0x07FF)
    achannel[4]  = ((readin[6]>>4  | readin[7]<<4)                   & 0x07FF)
    achannel[5]  = ((readin[7]>>7  | readin[8]<<1 | readin[9]<<9)    & 0x07FF)
    achannel[6]  = ((readin[9]>>2  | readin[10]<<6)                  & 0x07FF)
    achannel[7]  = ((readin[10]>>5 | readin[11]<<3)                  & 0x07FF)
    achannel[8]  = ((readin[12]    | readin[13]<<8)                  & 0x07FF)
    achannel[9]  = ((readin[13]>>3 | readin[14]<<5)                  & 0x07FF)
    achannel[10] = ((readin[14]>>6 | readin[15]<<2 | readin[16]<<10) & 0x07FF)
    achannel[11] = ((readin[16]>>1 | readin[17]<<7)                  & 0x07FF)
    achannel[12] = ((readin[17]>>4 | readin[18]<<4)                  & 0x07FF)
    achannel[13] = ((readin[18]>>7 | readin[19]<<1 |readin[20]<<9)   & 0x07FF)
    achannel[14] = ((readin[20]>>2 | readin[21]<<6)                  & 0x07FF)
    achannel[15] = ((readin[21]>>5 | readin[22]<<3)                  & 0x07FF)
#    if (readin[23] & (1<<0)):#0
#        digichannel[0] = 1
#    else:
#        digichannel[0] = 0
#    if(readin[23] & (1<<1)):#1
#        digichannel[1] = 1
#    else:
#        digichannel[1] = 0
#    if(readin[23] & (1<<2)):#2
#        digichannel[2] = 1
#    else:
#        digichannel[2] = 0
#    if(readin[23] & (1<<3)):#3
#        digichannel[3] = 1
#    else:
#        digichannel[3] = 0

def parse_serout(achannel,readin):#digichannel):
    sendout[0] = 0x0F #valid first byte to flight controller
    sendout[1] =  (achannel[0]  & 0x07FF) & 0xFF #07FF is 11111111111 to filter value to 11bit
    sendout[2] =  (((achannel[0]  & 0x07FF) >> 8)  | ((achannel[1] << 3) & 0x07FF)) & 0xFF
    sendout[3] =  (((achannel[1]  & 0x07FF) >> 5)  | ((achannel[2] << 6) & 0x07FF)) & 0xFF
    sendout[4] =  ((achannel[2]  & 0x07FF) >> 2) & 0xFF
    sendout[5] =  (((achannel[2]  & 0x07FF) >> 10) | ((achannel[3] << 1) & 0x07FF)) & 0xFF
    sendout[6] =  (((achannel[3]  & 0x07FF) >> 7)  | ((achannel[4] << 4) & 0x07FF)) & 0xFF
    sendout[7] =  (((achannel[4]  & 0x07FF) >> 4)  | ((achannel[5] << 7) & 0x07FF)) & 0xFF
    sendout[8] =  ((achannel[5]  & 0x07FF) >> 1) & 0xFF
    sendout[9] =  (((achannel[5]  & 0x07FF) >> 9)  | ((achannel[6] << 2) & 0x07FF)) & 0xFF
    sendout[10] = (((achannel[6]  & 0x07FF) >> 6)  | ((achannel[7] << 5) & 0x07FF)) & 0xFF
    sendout[11] = ((achannel[7]  & 0x07FF) >> 3) & 0xFF
    sendout[12] = (achannel[8]  & 0x07FF) & 0xFF
    sendout[13] = (((achannel[8]  & 0x07FF) >> 8)  | ((achannel[9] << 3) & 0x07FF)) & 0xFF
    sendout[14] = (((achannel[9]  & 0x07FF) >> 5)  | ((achannel[10] << 6) & 0x07FF)) & 0xFF
    sendout[15] = ((achannel[10] & 0x07FF) >> 2) & 0xFF
    sendout[16] = (((achannel[10] & 0x07FF) >> 10) | ((achannel[11] << 1) & 0x07FF)) & 0xFF
    sendout[17] = (((achannel[11] & 0x07FF) >> 7)  | ((achannel[12] << 4) & 0x07FF)) & 0xFF
    sendout[18] = (((achannel[12] & 0x07FF) >> 4)  | ((achannel[13] << 7) & 0x07FF)) & 0xFF
    sendout[19] = ((achannel[13] & 0x07FF) >> 1) & 0xFF
    sendout[20] = ((achannel[13] & 0x07FF) >> 9  | (achannel[14] & 0x07FF) << 2) & 0xFF
    sendout[21] = ((achannel[14] & 0x07FF) >> 6  | (achannel[15] & 0x07FF) << 5) & 0xFF
    sendout[22] = ((achannel[15] & 0x07FF) >> 3) & 0xFF
#    sendout[23]=0;#init digital channels byte to 0
#    if digichannel[0]==1:
#        sendout[23]=1
#    if digichannel[1]==1:
#        sendout[23]=sendout[23] | (1<<1)
#    if digichannel[2]==1:
#        sendout[23]=sendout[23] | (1<<2)
#    if digichannel[3]==1:
#        sendout[23]=sendout[23] | (1<<3)
#    sendout[23] = sendout[23] & 0xFF
    sendout[23]=readin[23];#no digichannels used on Radiolink AT9S
    sendout[24]=readin[24];#footer byte 0x00

def main():
#    mode=0#start mode with nothing targeted
    trackerinit=2#0=user has new targeting / 1=tracker running / 2=program start nothing happening
    ser = serial.Serial(#init serial
    port='/dev/ttyAMA0',
    baudrate=100000,
    bytesize=8,
    parity='E',
    stopbits=2,
    timeout=None,
    )
    cam = cv2.VideoCapture(0)#use USB cam
    time.sleep(0.5)#camera warmup for half second
    cam.set(cv2.CAP_PROP_CONVERT_RGB,1)#useing regular color webcam
    cam.set(cv2.CAP_PROP_FPS,60)#try to get 60fps from webcam
    cam.set(cv2.CAP_PROP_FRAME_WIDTH,640)#1280)#1920)#reasonable resolution for fast image processing
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT,480)#720)#1080)
    cv2.namedWindow("Show", cv2.WINDOW_NORMAL)#Declare the image window
    #cv2.setWindowProperty("Show",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)#fullscreen for FPV use
    ret_val, frame = cam.read() #must init & warm-up cam, this frame usually takes .250sec
    time.sleep(.25)
    pbox = (int(640/2),int(480/2),75,75)#centerpoint cx1 & cy1 and xwidth & ywidth
    bbox = (int(pbox[0]-(pbox[2]/2)),int(pbox[1]-(pbox[3]/2)),pbox[2],pbox[3])#init bounding box for tracker init; x1 / y1 / xwidth / ywidth
    for i in range(3):#Init right stick center for trimmed condition
        readin=ser.read(25)#take reading 3 times, use last one to elimnate any start up noise
        if readin[0]==0x0F:#bias is needed to be known when stick used for targeting
            parse_serin(readin)
            rstickhorz=achannel[0]#-993 # horz center bias
            rstickvert=achannel[1]#-1010 # vert center bias
            print(rstickhorz,rstickvert)
        else:
            print("0x0F not readin")
            rstickhorz=980#default value if assessment fails
            rstickvert=980
            print(rstickhorz,rstickvert)
        ser.reset_input_buffer()
    while True:#*******************************************
        ret_val, frame = cam.read() 
        readin=ser.read(25)#readin 25bytes or timeout
        if readin[0]==0x0F:#SBUS message start byte bits are LSB first, 11110000 is F0, but LSB First is 0F
            parse_serin(readin)
#            print(rstickhorz,rstickvert)
#            print([achannel[0],achannel[1],achannel[2],achannel[3],achannel[4],achannel[5],achannel[6],achannel[7],achannel[8],achannel[9],achannel[10],achannel[11],achannel[12],achannel[13],achannel[14],achannel[15]])
##            print([digichannel[0],digichannel[1],digichannel[2],digichannel[3]])#if digichannel[9]>800
##            print([readin[0],readin[1],readin[2],readin[3],readin[4],readin[5],readin[6],readin[7],readin[8],readin[9],readin[10],readin[11],readin[12],readin[13],readin[14],readin[15],readin[16],readin[17],readin[18],readin[19],readin[20],readin[21],readin[22],readin[23],readin[24]])
            if (achannel[9]>310):#&(mode==0):#'user targeting mode' switch B is ON from startup condition
#                mode=0
                trackerinit=0#init tracker each time user performs targeting
                x1=int(bbox[0]+((achannel[0]-rstickhorz)/15))
                y1=int(bbox[1]+((rstickvert-achannel[1])/15))
                width=int(achannel[7]/12)
                x2=x1+width
                y2=y1+width
                if x1<1:
                    x1=1
                if y1<1:
                    y1=1
                if x2>639:
                    x1=639-width
                if y2>479:
                    y1=479-width
                bbox = (int(x1),int(y1),int(width),int(width))
                cv2.rectangle(frame,(int(bbox[0]), int(bbox[1])),(int(bbox[0]+bbox[2]),int(bbox[1]+bbox[3])),(0,255,0),2)#topleft to bottomright
                cv2.putText(frame, "User Targeting", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
#                cv2.imshow("Show", frame)
#                parse_serout(achannel,readin)#digichannel)
#                ser.write([sendout[0],sendout[1],sendout[2],sendout[3],sendout[4],sendout[5],sendout[6],sendout[7],sendout[8],sendout[9],sendout[10],sendout[11],sendout[12],sendout[13],sendout[14],sendout[15],sendout[16],sendout[17],sendout[18],sendout[19],sendout[20],sendout[21],sendout[22],sendout[23],sendout[24]])
            if (achannel[9]<310):#&(mode==1):#'autotrack mode' switch B is OFF from Targeting condition
                if trackerinit==0:#OpenCV tracker.init; done each time user 'targets' new bbox start
                    tracker = cv2.TrackerMOSSE_create() #0.040s; fastest from opencv 3.4 img proc tests
                    ok = tracker.init(frame, bbox)#start tracker with first bbox
                    trackerinit=1
                if trackerinit==1:
                    ok, bbox = tracker.update(frame)#run tracker; ok=success or not
                    cv2.rectangle(frame,(int(bbox[0]), int(bbox[1])),(int(bbox[0]+bbox[2]),int(bbox[1]+bbox[3])),(255,0,0),2)#topleft to bottomright
                    if ok:
                        cv2.putText(frame, "Tracking", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
                    else:
                        cv2.putText(frame, "Fail Tracking", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
                if trackerinit==2:
#                    pbox = (int(640/2),int(480/2),75,75)#centerpoint cx1 & cy1 and xwidth & ywidth
#                    bbox = (int(pbox[0]-(pbox[2]/2)),int(pbox[1]-(pbox[3]/2)),pbox[2],pbox[3])#init bounding box for tracker init; x1 / y1 / xwidth / ywidth
                    cv2.rectangle(frame,(int(bbox[0]), int(bbox[1])),(int(bbox[0]+bbox[2]),int(bbox[1]+bbox[3])),(0,255,0),2)
                    cv2.putText(frame, "Start-Up", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            #
            parse_serout(achannel,readin)#digichannel)
            ser.write([sendout[0],sendout[1],sendout[2],sendout[3],sendout[4],sendout[5],sendout[6],sendout[7],sendout[8],sendout[9],sendout[10],sendout[11],sendout[12],sendout[13],sendout[14],sendout[15],sendout[16],sendout[17],sendout[18],sendout[19],sendout[20],sendout[21],sendout[22],sendout[23],sendout[24]])
        ser.reset_input_buffer()
#        time.sleep(0.1)
        cv2.imshow("Show", frame)# Display result
        if cv2.waitKey(1) == 27:# esc to quit
            stop_loop(frame,cam)
            break  


if __name__ == '__main__':
    main()
#python interpreter renames xyz.py program global __name__ to __main__
#if some other .py script uses thisscript.py as import thisscript.py then it's __name__ = actual name, not__main__
#
#
## --------- On Track Routine ----------
##            if cx < 370 and cx > 270:
##                print "On Track"
##
##            # --------- Steer Left Routine ----------
##            if cx <= 270:
##                LSteer = 270 - cx
##                SteerLeft = remap(LSteer, 0, 45, 1, 270)
##                print ("Turn Left: ", SteerLeft)  
