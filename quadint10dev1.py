"""Quadcopter Interceptor project 


"""
#
#tracks using only 'blue' minimum x and y magnitude point
#ccblobfinder works, but not used in tracking
#
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
import numpy as np
import subprocess#for setting cam properties at shell level

readin = bytearray(25) #'bytearray'init the array bytes to read in
sendout = bytearray(25)
La = list(range(16)) #0 to 15 ; 16 positions
Ld = list(range(4)) #0 to 15 ; 16 positions
achannel = array.array('I',La) #initialize a basic python array as 'I' is unsigned integer, 2 bytes
digichannel = array.array('I',Ld) #initialize a basic python array
#mode = int #0 start / 1 user targeting / 2 auto track / 3 auto pilot
rstickhorz = int #right stick horz center bias (trim)
rstickvert = int #right stick vert center bias (trim)

framewidth = int(640)#int(1920)#int(640)
frameheight = int(480)#int(1080)#int(480)

#----Function to measure CPU temp  use print(measure_temp())
def measure_temp():
        temp = os.popen("vcgencmd measure_temp").readline()
        return (temp.replace("temp=",""))

#---- Function Timer, returns current time
def how_long(start, activity):
    print('%s time %.3fs' % (activity, time.time()-start))
    return time.time()

def stop_loop(frame,cam):#break jumps us out of inner most loop and used in an if statement of the loop
    print(measure_temp())
    print(type(frame))
    print(frame.shape)
    print(frame.dtype)
#    cv2.waitKey(100)
    cam.release()
    cv2.destroyAllWindows()
    cv2.destroyAllWindows()
    
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


def simple_tracker(tpoint,trfrmsize,frame):#tpoint is [x,y] / width is frame width / frame is total camera pict
    global clickflag
    global SelctPtValB
#    trfrmsize=350#pixel area being processed makes timing difference
    x=40.00#pixel magnitude width
    halfx=x/2#use half magnitude to go up and down from user selected pixel target point
    
    #if Target is dark,low values, wrt background; then expect Target values to be low
#    start=time.time()
    bx1=int(tpoint[0]-(trfrmsize/2))
    by1=int(tpoint[1]-(trfrmsize/2))
    bx2=int(tpoint[0]+(trfrmsize/2))
    by2=int(tpoint[1]+(trfrmsize/2))
    if bx1<0:
        bx1=int(1)
        bx2=int(bx1+trfrmsize)
    if by1<0:
        by1=int(1)
        by2=int(by1+trfrmsize)
    if bx2>(framewidth-1):
        bx2=int((framewidth-1))
        bx1=int(bx2-trfrmsize)
    if by2>(frameheight-1):
        by2=int((frameheight-1))
        by1=int(by2-trfrmsize)
    box=[int(bx1),int(by1),int(bx2),int(by2)]#old box
    subframe=frame[int(by1):int(by2),int(bx1):int(bx2)]
    subframeB=subframe[:,:,0]
    subframeG=subframe[:,:,1]
    subframeR=subframe[:,:,2]
#only good for dark objects less
    LoPassBVal=int(SelctPtValB+halfx)
    if LoPassBVal>254:
        LoPassBVal=254
    HiPassBVal=int(SelctPtValB-halfx)
    if HiPassBVal<2:  #this sequence ends with inverted image because we track a 'dark' object on a bright background
        HiPassBVal=2  #first do HiPass shift down and then invert to add LoPass, this sequence matters!
    HiPass=subframeB-HiPassBVal#shift it down
    HiPass=np.clip(HiPass,0,255)#clip all neg values to 0
    LoPass=halfx-HiPass#invert image and shift up; all values more than x+val become zero
    BandPass=np.clip(LoPass,0,255)
#    print('width = ',width)
#    print('SelctPtValB = ',SelctPtValB)
#    print('BandPass MAX = ',np.amax(BandPass))
#    print('BandPass min = ',np.amin(BandPass))
#    print('Bandpass size = ',BandPass.shape)
    sumBx=np.sum(BandPass,axis=0)#sums columns into a single row
    sumBy=np.sum(BandPass,axis=1)#sums rows into a single column
    if sumBx.size > 1:
        sBxMax=np.argmax(sumBx)#pixel x location where min occurs
        if sumBy.size > 1:
            sByMax=np.argmax(sumBy)
            tpoint=[int(sBxMax+bx1),int(sByMax+by1)]
    return tpoint,subframe,box,BandPass,sumBx,sumBy#subframe,subframeB,subframeG,subframeR#bbox


def main():
    global tpoint
    global SelctPtValB
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
    
    cam_props = {'brightness': 128,#brightness (int)    : min=0 max=255 step=1 default=-8193 value=128
                 'contrast': 128,#contrast (int)    : min=0 max=255 step=1 default=57343 value=128
                 'saturation': 128,#saturation (int)    : min=0 max=255 step=1 default=57343 value=128
                 'white_balance_temperature_auto': 0,#white_balance_temperature_auto (bool)   : default=1 value=1
                 'gain': 50,#gain (int)    : min=0 max=255 step=1 default=57343 value=0
                 'power_line_frequency': 2,#power_line_frequency (menu)   : min=0 max=2 default=2 value=2
                 'white_balance_temperature': 4000,#white_balance_temperature (int)    : min=2000 max=6500 step=1 default=57343 value=4000 flags=inactive
                 'sharpness': 128,#sharpness (int)    : min=0 max=255 step=1 default=57343 value=128
                 'backlight_compensation': 0,#backlight_compensation (int)    : min=0 max=1 step=1 default=57343 value=0
                 'exposure_auto': 1,#exposure_auto (menu)   : min=0 max=3 default=0 value=1 is manual
                 'exposure_absolute': 300,#exposure_absolute (int)    : min=3 max=2047 step=1 default=250 value=250 flags=inactive
                 'exposure_auto_priority': 1,#exposure_auto_priority (bool)   : default=0 value=1
                 'pan_absolute': 0,#pan_absolute (int)    : min=-36000 max=36000 step=3600 default=0 value=0
                 'tilt_absolute': 0,#tilt_absolute (int)    : min=-36000 max=36000 step=3600 default=0 value=0
                 'focus_absolute': 100,#250 is near; 0 is far;focus_absolute (int)    : min=0 max=250 step=5 default=8189 value=0 flags=inactive
                 'focus_auto': 0,#focus_auto (bool)   : default=1 value=1
                 'zoom_absolute': 100}#zoom_absolute (int)    : min=100 max=500 step=1 default=57343 value=100
    for key in cam_props:
        subprocess.call(['v4l2-ctl -d /dev/video0 -c {}={}'.format(key, str(cam_props[key]))], shell=True)
    subprocess.call(['v4l2-ctl -d /dev/video0 -l'], shell=True)
    time.sleep(0.5)
    #Do it again to make sure it the camera gets it
    for key in cam_props:
        subprocess.call(['v4l2-ctl -d /dev/video0 -c {}={}'.format(key, str(cam_props[key]))], shell=True)
    subprocess.call(['v4l2-ctl -d /dev/video0 -l'], shell=True)
    time.sleep(0.5)
    
    cam = cv2.VideoCapture(0)#use USB cam
    time.sleep(0.5)#camera warmup for half second

#    cam.set(cv2.CAP_PROP_CONVERT_RGB,1)#useing regular color webcam
#    cam.set(cv2.CAP_PROP_FPS,60)#try to get 60fps from webcam
    cam.set(cv2.CAP_PROP_FRAME_WIDTH,framewidth)#1280)#1920)#reasonable resolution for fast image processing
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT,frameheight)#720)#1080)
    
    cv2.namedWindow("Show", cv2.WINDOW_NORMAL)#Declare the image window
    cv2.setMouseCallback('Show',return_mouse_click)#event driven hook to this window
    cv2.resizeWindow('Show',framewidth,frameheight)#640,480)
#    cv2.namedWindow("Showbbox", cv2.WINDOW_NORMAL)#
#    cv2.namedWindow("ShowDevFrame", cv2.WINDOW_NORMAL)#
##    cv2.setWindowProperty("Show",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)#fullscreen for FPV use
    ret_val, frame = cam.read() #must init & warm-up cam, this frame usually takes .250sec
    time.sleep(.25)
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
    SelctPtValB=int(50)
    SelctPtValG=int(127)
    SelctPtValR=int(127)
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
    n=int(0)
    e=int(0)
    start=time.time()
    while loop==1:#*******************************************
        ret_val, frame = cam.read() #read frame from camera
        readin=ser.read(25)#readin 25bytes or timeout
        if readin[0]==0x0F:#Valid SBUS message start byte bits are LSB first, 11110000 is F0, but LSB First is 0F
            parse_serin(readin)
            if (achannel[9]>310):#F switch On 'user targeting mode'
                rsh=int(achannel[0]-rstickhorz)
                rsv=int(rstickvert-achannel[1])
                if abs(rsh) > 5 :#Horizontal
                    tpoint[0]=int(tpoint[0]+(rsh/15))
                    if tpoint[0] < 1:
                        tpoint[0] = int(1)
                    if tpoint[0] > framewidth:
                        tpoint[0]=int(framewidth-1)
                if abs(rsv) > 5 :#Vertical
                    tpoint[1]=int(tpoint[1]+(rsv/15))
                    if tpoint[1] < 1:
                        tpoint[1] = int(1)
                    if tpoint[1] > frameheight:
                        tpoint[1]=int(frameheight-1)
                width=int((achannel[7]-300)/2)
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
                    SelctPtValB=np.average(tgtfrm)
                finally:
                    SelctPtValB=int(50)
                cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)#topleft to bottomright
                cv2.line(frame,(int(tpoint[0]),0),(int(tpoint[0]),int(frame.shape[0]-1)),(0,255,0),1)
                cv2.line(frame,(0,int(tpoint[1])),(int(frame.shape[1]-1),int(tpoint[1])),(0,255,0),1)
                cv2.putText(frame, "Targeting", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            if (achannel[9]<310):#'user flight mode' switch F is OFF from Targeting condition
                trfrmsize=int((achannel[7]-50)/3)
                tpoint,subframe,box,BandPass,sumBx,sumBy= simple_tracker(tpoint,trfrmsize,frame)
                cv2.rectangle(frame,(int(box[0]), int(box[1])),(int(box[2]),int(box[3])),(0,0,255),2)#topleft to bottomright
                cv2.line(frame,(int(tpoint[0]),0),(int(tpoint[0]),int(frame.shape[0]-1)),(0,0,255),1)
                cv2.line(frame,(0,int(tpoint[1])),(int(frame.shape[1]-1),int(tpoint[1])),(0,0,255),1)
                if (achannel[8]>310):#B switch On with F OFF
                    cv2.putText(frame, "Auto Pilot", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
                    #place autopilot controls here (RstkHorz Roll A [0] / RstkVert Pitch E [1] / LstkVert Thrtl [2] / LstkHoz Yaw [3]
                else:#User Flight Mode
                    cv2.putText(frame, "Controlled Flight", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            parse_serout(achannel,readin)#digichannel)
            ser.write([sendout[0],sendout[1],sendout[2],sendout[3],sendout[4],sendout[5],sendout[6],sendout[7],sendout[8],sendout[9],sendout[10],sendout[11],sendout[12],sendout[13],sendout[14],sendout[15],sendout[16],sendout[17],sendout[18],sendout[19],sendout[20],sendout[21],sendout[22],sendout[23],sendout[24]])
        ser.reset_input_buffer()
        n=n+1
        if n>2:
            n=0
            cv2.imshow("Show", frame)# Display result
            if cv2.waitKey(50) == 27:# esc to quit; this .waitKey consumes at least 22ms time
                stop_loop(frame,cam)
                break
        print(time.time()-start,' LOOP TIME')
        start=time.time()
#        print(np.amax(BandPass),' BandPass max val')
#        print(np.amin(BandPass),' BandPass min val')
if __name__ == '__main__':
    main()
