"""Quadcopter Interceptor project 
Use Raspivid (9ms)
Use basic peak mag in track frame for tracking
"""
#
#this is not using python enviroments #!/usr/bin/env python3
#
import cv2
import time      
from matplotlib import pyplot as plt #matlab like plotting
import os               #for CPU temperature
import array            #stock python array handling
import serial           #serial port
import numpy as np      #numpy array system
import subprocess as sp #for streaming frames
import atexit           #allows a function to be registered and called when python program exits
import sys              #system to allow reading the stream size in buffer
#setup SBUS and remote controller joysticks
readin = bytearray(25) #'bytearray'init the array bytes to read in
sendout = bytearray(25)
La = list(range(16)) #0 to 15 ; 16 positions
Ld = list(range(4)) #0 to 15 ; 16 positions
achannel = array.array('I',La) #initialize a basic python array as 'I' is unsigned integer, 2 bytes
digichannel = array.array('I',Ld) #initialize a basic python array
#mode = int #0 start / 1 user targeting / 2 auto track / 3 auto pilot
rstickhorz = int #right stick horz center bias (trim)
rstickvert = int #right stick vert center bias (trim)
#set up camera on mipi
framewidth = int(640)#(640,480) mode 7 for 120fps / (1280,720) mode 6 for 90fps
frameheight = int(480)
bytesPerFrame = framewidth * frameheight
fps = 120
# see "raspividyuv --help" for more information on the parameters
#videoCmd = "raspividyuv -w "+str(w)+" -h "+str(h)+" --output - --timeout 0 --framerate "+str(fps)+" --luma --nopreview"
videoCmd = "raspividyuv -w "+str(framewidth)+" -h "+str(frameheight)+" --output - --timeout 0 --framerate "+str(fps)+" --luma --nopreview --mode 7"
videoCmd = videoCmd.split() # Popen requires that each parameter is a separate string
cameraProcess = sp.Popen(videoCmd, stdout=sp.PIPE, stderr=sp.PIPE, bufsize=((framewidth*frameheight)+20)) # start the camera
atexit.register(cameraProcess.terminate) # this closes the camera process in case the python scripts exits unexpectedly
frame=np.empty((h,w),dtype=np.uint8)#init empty frame for cam_frame err handling purposes

#---Function to read the camera
def cam_read(frame):#args are public, no need to pass, pass the old frame
    rawStream = cameraProcess.stdout.read(bytesPerFrame)
#     print(sys.getsizeof(rawStream))
    cameraProcess.stdout.flush()#be sure to clear the buffer
    if sys.getsizeof(rawStream) == ((w*h)+17):#cam function check, expect to get 1 byte for every pixel pluse 17(?metadata?) bytes, 921617(1280,720)
        frame=np.frombuffer(rawStream, dtype=np.uint8)
        frame=frame[0:bytesPerFrame]#get frame, first time in this program
        frame.shape=(h,w)#turn linear array into 2D image array
    return frame

#----Function to measure CPU temp  use print(measure_temp())
def measure_temp():
        temp = os.popen("vcgencmd measure_temp").readline()
        return (temp.replace("temp=",""))
  
#----Function to close the program, loop exit
def stop_loop(frame,cameraProcess):#break jumps us out of inner most loop and used in an if statement of the loop
    print(measure_temp())
    print(type(frame))
    print(frame.shape)
    print(frame.dtype)
    cv2.destroyAllWindows()
    cameraProcess.terminate() # stop the camera

#----Mouse Click event handler
def return_mouse_click(event,xmouse,ymouse,flags,param):#special var return via global vars
    global tpoint
    global clickflag
    global x
    global y
    print("Mouse ",repr(event))
    if event == 1:#left mouse button depress
        clickflag=1
        tpoint[0]=xmouse
        tpoint[1]=ymouse
    return tpoint#,varBreak

#----Function to track target
def simple_tracker(tpoint,width,frame):#tpoint is [x,y] / width is frame width / frame is total camera pict
    global clickflag
    global SelctPtVal
#     width=300
    newwidth=width
    #
    #if Target is dark,low values, wrt background; then expect Target values to be low
#    start=time.time()
    halfwidth=width/2
    x1=int(tpoint[0]-(halfwidth))
    y1=int(tpoint[1]-(halfwidth))
    x2=int(tpoint[0]+(halfwidth))
    y2=int(tpoint[1]+(halfwidth))
    if x1<0:
        x1=int(0)
        x2=int(x1+width)
    if y1<0:
        y1=int(0)
        y2=int(y1+width)
    if x2>(w-1):
        x2=int((w-1))
        x1=int(x2-width)
    if y2>(h-1):
        y2=int((h-1))
        y1=int(y2-width)
    oldbox=(int(x1),int(y1),int(x2),int(y2))#old box
    subframe=frame[int(y1):int(y2),int(x1):int(x2)]#smaller tracking frame inside large image frame
#    print('subframe size = ',subframe.shape)
    if clickflag==1:#when user selects target, perform threshold calc
        pickhalfwidth=2
        SelctPtValfrm=frame[int(tpoint[1]-pickhalfwidth):int(tpoint[1]+pickhalfwidth),int(tpoint[0]-pickhalfwidth):int(tpoint[0]+pickhalfwidth)]
        SelctPtVal=np.average(SelctPtValfrm)
#         sv1=frame[tpoint[1]-1,tpoint[0]-1]
#         sv2=frame[tpoint[1]-1,tpoint[0]]
#         sv3=frame[tpoint[1]-1,tpoint[0]+1]
#         sv4=frame[tpoint[1],tpoint[0]-1]
#         sv5=frame[tpoint[1],tpoint[0]]
#         sv6=frame[tpoint[1],tpoint[0]+1]
#         sv7=frame[tpoint[1]+1,tpoint[0]-1]
#         sv8=frame[tpoint[1]+1,tpoint[0]]
#         sv9=frame[tpoint[1]+1,tpoint[0]+1]
#         SelctPtVal=(sv1+sv2+sv3+sv4+sv5+sv6+sv7+sv8+sv9)/9#applies to BandPass
        print('MouseClicked New SelctPtVal = ',SelctPtVal,' clickflag= ',clickflag)
        clickflag=0
#only good for dark objects less
    #BandPass processing (2ms)
    stime=time.time()
    x=40.00#pixel magnitude width
    halfx=x/2#use half magnitude to go up and down from user selected pixel target point
    LoPassVal=int(SelctPtVal+halfx)
    if LoPassVal>254:
        LoPassVal=254
    HiPassVal=int(SelctPtVal-halfx)
    if HiPassVal<2:  #this sequence ends with inverted image because we track a 'dark' object on a bright background
        HiPassVal=2  #first do HiPass shift down and then invert to add LoPass, this sequence matters!
    HiPass=subframe-HiPassVal#shift it down ; all values less than HiPassVal become negative numbers
    HiPass=np.clip(HiPass,0,255)#clip all neg values to 0
    LoPass=halfx-HiPass#invert image and shift up ; all values more than LoPassVal become negative numbers
    BandPass=np.clip(LoPass,0,255)#clip all neg values to 0
#     print('width = ',width,'  Bandpass size = ',BandPass.shape) #     print('Time= ',(time.time()-stime),'SelctPtVal = ',SelctPtVal,' clickflag= ',clickflag) #     print('BandPass MAX = ',np.amax(BandPass),'  BandPass min = ',np.amin(BandPass))
    #End of BandPass processing
#     BandPass=255-subframe#Invert image only if bypassing the BandPass
    sumx=np.sum(BandPass,axis=0)#sums all rows into one row, position in sum row is tpoint0
    sumy=np.sum(BandPass,axis=1)#sums all clms into one clm, position in sum clm is tpoint1
    if sumx.size > 1:
        sxMax=np.argmax(sumx)#pixel x location where min occurs
        if sumy.size > 1:
            syMax=np.argmax(sumy)
            tpoint=(int(sxMax+x1),int(syMax+y1))#tpoint is absolute to camera image frame
    x1=int(tpoint[0]-(halfwidth))
    y1=int(tpoint[1]-(halfwidth))
    x2=int(tpoint[0]+(halfwidth))
    y2=int(tpoint[1]+(halfwidth))
    if x1<0:
        x1=int(0)
        x2=int(x1+width)
    if y1<0:
        y1=int(0)
        y2=int(y1+width)
    if x2>(w-1):
        x2=int((w-1))
        x1=int(x2-width)
    if y2>(h-1):
        y2=int((h-1))
        y1=int(y2-width)
    box=(int(x1),int(y1),int(x2),int(y2))#new box
    return tpoint,newwidth,subframe,box,BandPass,sumx,sumy#subframe,subframeG,subframeR#bbox  
  
#----Function to read SBUS serial input
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
#    if (readin[23] & (1<<0)):#0 for radios with 'digichannels'
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

#----Function to send out SBUS Serial
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
#    sendout[23]=0;#init digital channels byte to 0, for 'digichannel' radios
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
#********************************************************************************************    
#Main Program
def main():
    global tpoint
    global SelctPtVal
    global clickflag

    clickflag=0#flag indicating mouse click or target selection
    loop=1#while loop control
    
    trackerinit=2#0=user has new targeting / 1=tracker running / 2=program start nothing happening
    rsh = np.empty(0) #right stick horz center bias (trim)
    rsv = np.empty(0) #right stick vert center bias (trim)
    
    ser = serial.Serial(#init serial
    port='/dev/ttyAMA0',
    baudrate=100000,
    bytesize=8,
    parity='E',
    stopbits=2,
    timeout=None,
    )
    #
    cv2.namedWindow("Show", cv2.WINDOW_NORMAL)#Declare the image window
    cv2.setMouseCallback('Show',return_mouse_click)#event driven hook to this window
    cv2.resizeWindow('Show',framewidth,frameheight)#640,480)
#    cv2.namedWindow("Showbbox", cv2.WINDOW_NORMAL)#
#    cv2.namedWindow("ShowDevFrame", cv2.WINDOW_NORMAL)#
##    cv2.setWindowProperty("Show",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)#fullscreen for FPV use
    #
    tpoint=[1,1]
    tpoint[0]=int(framewidth/2)#X graph axis center
    tpoint[1]=int(frameheight/2)#Y graph axis center
    targetwidth=100
    box=[1,1,1,1]#box is size set by user VrBKnob during targeting; but changes to targeting area box in track mode
    box[0]=int(tpoint[0]-targetwidth)#X1 for targeting box
    box[1]=int(tpoint[1]-targetwidth)#Y1
    box[2]=int(tpoint[0]+targetwidth)#X2 for targeting box
    box[3]=int(tpoint[1]+targetwidth)#Y2
    trfrmsize=350#pixel area being processed
    SelctPtVal=int(50)
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
    #
    frame=cam_read(frame)
    #
    time.sleep(.05)
    n=int(0)
    e=int(0)
    start=time.time()
    while loop==1:#*******************************************
        frame=cam_read(frame)#read frame from camera
        readin=ser.read(25)#readin 25bytes or timeout, every 13.2ms for R6DSM, message frame takes 3ms
        if readin[0]==0x0F:#Valid SBUS message start byte bits are LSB first, 11110000 is F0, but LSB First is 0F
            parse_serin(readin)
            if (achannel[9]>310):#F switch On 'user targeting mode'
                rsh=int(achannel[0]-rstickhorz)#incorp cal into horz and vert on right stick reading
                rsv=int(rstickvert-achannel[1])
                if abs(rsh) > 5 :#Horizontal move only if greater than 
                    tpoint[0]=int(tpoint[0]+(rsh/15))#increment tpoint position
                    if tpoint[0] < 1:
                        tpoint[0] = int(1)
                    if tpoint[0] > framewidth:
                        tpoint[0]=int(framewidth-1)
                if abs(rsv) > 5 :#Vertical move only if greater than
                    tpoint[1]=int(tpoint[1]+(rsv/15))#increment tpoint position
                    if tpoint[1] < 1:
                        tpoint[1] = int(1)
                    if tpoint[1] > frameheight:
                        tpoint[1]=int(frameheight-1)
                width=int((achannel[7]-300)/2)#set track box size using knob
                print("width = ",achannel[7])
                x1=int(tpoint[0])-width
                if x1<1:
                    x1=1
                y1=int(tpoint[1])-width
                if y1<1:
                    y1=1
                x2=int(tpoint[0])+width
                if x2>framewidth:
                    x2=int(framewidth-1)
                y2=int(tpoint[1])+width
                if y2>frameheight:
                    y2=int(frameheight-1)
                try:
                    tgtfrm=frame[int(y1):int(y2),int(x1):int(x2)]
                    SelctPtVal=np.average(tgtfrm)
                finally:
                    SelctPtVal=int(50)
                cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)#topleft to bottomright
                cv2.line(frame,(int(tpoint[0]),0),(int(tpoint[0]),int(frame.shape[0]-1)),(0,255,0),1)
                cv2.line(frame,(0,int(tpoint[1])),(int(frame.shape[1]-1),int(tpoint[1])),(0,255,0),1)
                cv2.putText(frame, "Targeting", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            if (achannel[9]<310):#'Flight Mode' USER switch F is OFF (with B off) from Targeting condition
                trfrmsize=int((achannel[7]-50)/3)
                tpoint,subframe,box,BandPass,sumBx,sumBy= simple_tracker(tpoint,trfrmsize,frame)
                cv2.rectangle(frame,(int(box[0]), int(box[1])),(int(box[2]),int(box[3])),(0,0,255),2)#topleft to bottomright
                cv2.line(frame,(int(tpoint[0]),0),(int(tpoint[0]),int(frame.shape[0]-1)),(0,0,255),1)
                cv2.line(frame,(0,int(tpoint[1])),(int(frame.shape[1]-1),int(tpoint[1])),(0,0,255),1)
                if (achannel[8]>310):#'Flight Mode' AUTOPILOT switch B is On (with F OFF)
                    cv2.putText(frame, "Auto Pilot", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
                    #place autopilot controls here (RstkHorz Roll A [0] / RstkVert Pitch E [1] / LstkVert Thrtl [2] / LstkHoz Yaw [3]
                else:#User Flight Mode
                    cv2.putText(frame, "Controlled Flight", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            parse_serout(achannel,readin)#digichannel)
            ser.write([sendout[0],sendout[1],sendout[2],sendout[3],sendout[4],sendout[5],sendout[6],sendout[7],sendout[8],sendout[9],sendout[10],sendout[11],sendout[12],sendout[13],sendout[14],sendout[15],sendout[16],sendout[17],sendout[18],sendout[19],sendout[20],sendout[21],sendout[22],sendout[23],sendout[24]])
        ser.reset_input_buffer()
        n=n+1
        if n>2:#show image on screen 
            n=0
            frameshow=frame.copy()#must copy to overcome assignment error
            frameshow[int(box[1]):int(box[3]),int(box[0]):int(box[2])]=BandPass#box=(int(x1),int(y1),int(x2),int(y2))#new box
            cv2.imshow("Show", frameshow)# Display result
            if cv2.waitKey(50) == 27:# esc to quit; this .waitKey consumes at least 22ms time
                stop_loop(frame,cameraProcess)
                break
        print(time.time()-start,' LOOP TIME')
        start=time.time()
#
#
if __name__ == '__main__':
    main()