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
from matplotlib import pyplot as plt
import os #for CPU temperature
import array #stock python array handling
import serial #serial port
import numpy as np
import subprocess#for setting cam properties at shell level
#from BlobFinder2 import ccblobfinder

readin = bytearray(25) #'bytearray'init the array bytes to read in
sendout = bytearray(25)
La = list(range(16)) #0 to 15 ; 16 positions
Ld = list(range(4)) #0 to 15 ; 16 positions
achannel = array.array('I',La) #initialize a basic python array as 'I' is unsigned integer, 2 bytes
digichannel = array.array('I',Ld) #initialize a basic python array

rstickhorz = int
stickvert = int

framewidth = int(640)
frameheight = int(480)
#640x480 is max resolution for max frame rate of about 100fps

# ---- Function definition for converting scales ------
def remap(unscaled, to_min, to_max, from_min, from_max):
    return (to_max-to_min)*(unscaled-from_min)/(from_max-from_min)+to_min

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
#    cv2.destroyAllWindows()
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
    sendout[23]=readin[23];#no digichannels used on Radiolink AT9S
    sendout[24]=readin[24];#footer byte 0x00

def return_mouse_click(event,x,y,flags,param):#special var return via global vars
    global tpoint
    global clickflag
    global showblob
    print("Mouse ",repr(event))
#    if event == 7:#stop the program
#        varBreak=1
    if event == 7:#left mouse button DoubleClick
        showblob=1
        clickflag=0
    if event == 1:#left mouse button depress
        if showblob!=1:
            tpoint=(x,y)
            clickflag=1
        else:
            clickflag=0
#        tbox=(int(t0),int(t1),int(t2),int(t3))
        return tpoint#,varBreak


def simple_tracker(tpoint,width,frame):
    global clickflag
    global SelctPtValB
    #if Target is dark,low values, wrt background; then expect Target values to be low
#    start=time.time()
    bx1=int(tpoint[0]-(width/2))
    by1=int(tpoint[1]-(width/2))
    bx2=int(tpoint[0]+(width/2))
    by2=int(tpoint[1]+(width/2))
    if bx1<0:
        bx1=int(0)
        bx2=int(bx1+width)
    if by1<0:
        by1=int(0)
        by2=int(by1+width)
    if bx2>(framewidth-1):
        bx2=int((framewidth-1))
        bx1=int(bx2-width)
    if by2>(frameheight-1):
        by2=int((frameheight-1))
        by1=int(by2-width)
    box=(int(bx1),int(by1),int(bx2),int(by2))#old box
    subframe=frame[int(by1):int(by2),int(bx1):int(bx2)]
    subframeB=subframe[:,:,0]
    subframeG=subframe[:,:,1]
    subframeR=subframe[:,:,2]
#    print('subframeB size = ',subframeB.shape)
#    subframe=(subframe[:,:,0]+subframe[:,:,1]+subframe[:,:,2])/3#rgb to gray
    if clickflag==1:#when user selects target, perform threshold calc
        svB1=frame[tpoint[1]-1,tpoint[0]-1,0]
        svB2=frame[tpoint[1]-1,tpoint[0],0]
        svB3=frame[tpoint[1]-1,tpoint[0]+1,0]
        svB4=frame[tpoint[1],tpoint[0]-1,0]
        svB5=frame[tpoint[1],tpoint[0],0]
        svB6=frame[tpoint[1],tpoint[0]+1,0]
        svB7=frame[tpoint[1]+1,tpoint[0]-1,0]
        svB8=frame[tpoint[1]+1,tpoint[0],0]
        svB9=frame[tpoint[1]+1,tpoint[0]+1,0]
        SelctPtValB=(svB1+svB2+svB3+svB4+svB5+svB6+svB7+svB8+svB9)/9
        width=40
    x=40.00
    halfx=x/2
#only good for dark objects less
    LoPassBVal=SelctPtValB+halfx
    if LoPassBVal>254:
        LoPassBVal=254
    HiPassBVal=SelctPtValB-halfx
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
    sumBx=np.sum(BandPass,axis=0)
    sumBy=np.sum(BandPass,axis=1)
    wtemp=0
    newwidth=width
    print('ClickFlag ',clickflag)
    print('SelctPtValB = ',SelctPtValB)
    if clickflag==0:
        sumBxnz=np.nonzero(sumBx)
        if int(sumBxnz[0].shape[0])>1:#at least two values for min and max
            wtemp=int(np.max(sumBxnz)-np.min(sumBxnz))
        sumBynz=np.nonzero(sumBy)
        if int(sumBynz[0].shape[0])>1:    
            sumBynzWidth=int(np.max(sumBynz)-np.min(sumBynz))
            if sumBynzWidth>wtemp:
                wtemp=sumBynzWidth
        if wtemp>0:
            newwidth=wtemp*4
            if newwidth>300:
                newwidth=300
            elif newwidth<40:
                newwidth=40
    else:
        clickflag=0
        print('Reset ClickFlag ',clickflag)
    if sumBx.size > 1:
        sBxMax=np.argmax(sumBx)#pixel x location where min occurs
        if sumBy.size > 1:
            sByMax=np.argmax(sumBy)
            tpoint=(int(sBxMax+bx1),int(sByMax+by1))
    return tpoint,newwidth,subframe,box,BandPass,sumBx,sumBy#subframe,subframeB,subframeG,subframeR#bbox


def ccblobfinder(ccframe):#CONNECTED COMPONENTS BLOB DETECTOR
#    fig=plt.figure()
    #ccframe=worked image where pixels being replaced by markers of blobs as neg numbers, -1 is first blob, -2 is 2nd blob, etc..
    print(np.amax(ccframe),' ccframe max val')
    print(np.amin(ccframe),' ccframe min val')
    rf=(ccframe.shape[1])#
    cf=(ccframe.shape[0])
    ccframe[:,0]=0#set edges of ccframe to zeros because these are not used
    ccframe[:,cf-1]=0#.shape includes 0 position, so must -1 for actual last position in matrix
    ccframe[0,:]=0
    ccframe[rf-1,:]=0
    ccsumx=np.sum(ccframe,axis=1)#sum rows to get x
    ccsumy=np.sum(ccframe,axis=0)#sum cols to get y
#    plt.subplot(121)
#    plt.plot(ccsumx)
#    plt.subplot(122)
#    plt.plot(ccsumy)
#    plt.show()
    n=int(-1)#starting blob group
    m=int(-100)
    blob=np.zeros((rf,cf,100), dtype=int)# upto 99 blobs
    for r in range(1,(rf-2)):#fix algo later to handle edges
        if (ccsumx[r])!=0:
            for c in range(1,(cf-2)):   #use neg numbrs for labeling pixels in ccframe, yet be able to keep orig values
                if ccframe[r,c]>0:      #if the new pixel visited is >0, fresh new foreground pixel (it has not been visited before nor it's a zero val)
                    n=n+1 #We are in a new blob, so increment the blob value index, 'n' is the name of the blob being marked
                    ncurrent=n
                    m=m+1
                    print('increment n to ',n,'  increment m to ',m)
                    if m==0:
                        print('blob count exceeds 99')
                        break
                    blob[r,c,n]=255#ccframe[r,c]#record the pixel for this blob, use n or 'size(blob,3)' value is the latest index of 3rd dim in blob
                    print(r,' x ',c,' new N blob mark to ',n, ' and m=',m)
                    ccframe[r,c]=m#we will mark each pixel with a neg value corresponding to the blob marking name, but a negative val of the blob name
                if ccframe[r,c]<0:#if current pixel is not equal to zero then let's search it's neighbors, the above has already set the current blob label value
                    #Look East
                    if ccframe[r,c+1] < m:#if it has been labeled in prior group...
                        priorlabel=int(ccframe[r,c+1]+99)#cc algo is a raster scan, so we only need this check here for anti-spoofing
                        blob[:,:,ncurrent]=blob[:,:,ncurrent]+blob[:,:,priorlabel]#merge the two connected blobs
                        blob[:,:,priorlabel]=0
                        print('prior group r,c+1 relabeled')
                        print('ccframe[r,c+1] value=',ccframe[r,c+1],'  move ',priorlabel,'=priorlabel  ',ncurrent,'=current label')
                    elif ccframe[r,c+1] > 0:#greater than zero value
                        blob[r,c+1,ncurrent]=255#ccframe[r,c+1]#must be either positive val or current neg val; place the pixel value into the blob array 
                        ccframe[r,c+1]=m#apply current pixel 'label'(neg value) to this neighbor
                        print(r,' x ',c,' Look East record to n ',ncurrent)
                    else: pass
                    #Look South East
                    if ccframe[r+1,c+1] < m:#if it has been labeled in prior group...
                        priorlabel=int(ccframe[r+1,c+1]+99)#cc algo is a raster scan, so we only need this check here for anti-spoofing
                        blob[:,:,ncurrent]=blob[:,:,ncurrent]+blob[:,:,priorlabel]#merge the two connected blobs
                        blob[:,:,priorlabel]=0
                        print('prior group r+1,c+1 relabeled')
                        print('ccframe[r+1,c+1] value=',ccframe[r+1,c+1],'  move ',priorlabel,'=priorlabel  ',ncurrent,'=current label')
                    elif ccframe[r+1,c+1] > 0:#greater than zero value
                        blob[r+1,c+1,ncurrent]=255#ccframe[r,c+1]#must be either positive val or current neg val; place the pixel value into the blob array 
                        ccframe[r+1,c+1]=m#apply current pixel 'label'(neg value) to this neighbor
                        print(r,' x ',c,' Look South East record to n ',ncurrent)
                    else: pass
                    #Look South
                    if ccframe[r+1,c] < m:#if it has been labeled in prior group...
                        priorlabel=int(ccframe[r+1,c]+99)#cc algo is a raster scan, so we only need this check here for anti-spoofing
                        blob[:,:,ncurrent]=blob[:,:,ncurrent]+blob[:,:,priorlabel]#merge the two connected blobs
                        blob[:,:,priorlabel]=0
                        print('ccframe[r+1,c] value=',ccframe[r+1,c],'  move ','prior group r+1,c relabeled')
                        print(priorlabel,'=priorlabel  ',ncurrent,'=current label')
                    elif ccframe[r+1,c] > 0:#greater than zero value
                        blob[r+1,c,ncurrent]=255#ccframe[r,c+1]#must be either positive val or current neg val; place the pixel value into the blob array 
                        ccframe[r+1,c]=m#apply current pixel 'label'(neg value) to this neighbor
                        print(r,' x ',c,' Look South record to n ',ncurrent)
                    else: pass
                    #Look South West
                    if ccframe[r+1,c-1] < m:#if it has been labeled in prior group...
                        priorlabel=int(ccframe[r+1,c-1]+99)#cc algo is a raster scan, so we only need this check here for anti-spoofing
                        blob[:,:,ncurrent]=blob[:,:,ncurrent]+blob[:,:,priorlabel]#merge the two connected blobs
                        blob[:,:,priorlabel]=0
                        print('prior group r+1,c-1 relabeled')
                        print('ccframe[r+1,c-1] value=',ccframe[r+1,c-1],'  move ',priorlabel,'=priorlabel  ',ncurrent,'=current label')
                    elif ccframe[r+1,c-1] > 0:#greater than zero value
                        blob[r+1,c-1,ncurrent]=255#ccframe[r,c+1]#must be either positive val or current neg val; place the pixel value into the blob array 
                        ccframe[r+1,c-1]=m#apply current pixel 'label'(neg value) to this neighbor
                        print(r,' x ',c,' Look South East record to n ',ncurrent)
                    else: pass
        if m==0:
            print('blob count exceeds 99')
            break
    x=0#handles index lockup when blob(:,:,index) is reduced for removing the empty matrices in the below
    blob2=np.zeros((rf,cf,0), dtype=int)# blob2 init
    blobcount=0
    if n==-1: #handles when there are NO blobs found
        ccframe2=0#return value as empty array allowing to check using "not ccframe2" in main program
        blob2=0#return value as empty array allowing to check using "not blob2" in main program
    else:
        print(n+1,' total blobs generated enter rearrange loop')
        ccframe2=np.zeros((rf,cf))
        for i in range(0,(n+1)): #remove any blank blob dims and resort
            i=i-x#must do this way because python cannot mutate 3rd dim to pass nonzero array to blob2
            print('for loop count i=',i,' and decrementer x=',x)
            if np.any(blob[:,:,i]):#check for non zero values
                ccframe2=ccframe2+blob[:,:,i]
                print('orig nonzero blob #',i+x,' added to ccframe2 and ',blobcount)
                blob2=np.uint8(np.dstack((blob2,blob[:,:,i])))
                name="CCBlobfinder"+str(blobcount)
                cv2.namedWindow(name, cv2.WINDOW_NORMAL)#
                cv2.imshow(name,blob2[:,:,blobcount])
                blobcount=blobcount+1
            else:
                blob=np.delete(blob,i,axis=2)#destroy this 'i' blob frame
                print('blob #',i,' destroyed')
                x=x+1
    return blob2,ccframe2


def main():
#    global tbox#required for mouse call function def return_mouse_click
    global tpoint
    global SelctPtValB
    global clickflag
    global showblob
#    global varBreak
    showblob=0
    clickflag=0#flag indicating mouse click or target selection
    loop=1#while loop control
    trackerinit=2#0=user has new targeting / 1=tracker running / 2=program start nothing happening
    params = cv2.SimpleBlobDetector_Params
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
    fig=plt.figure()
    start=time.time()
    cam_props = {'brightness': 128,#brightness (int)    : min=0 max=255 step=1 default=-8193 value=128
                 'contrast': 128,#contrast (int)    : min=0 max=255 step=1 default=57343 value=128
                 'saturation': 128,#saturation (int)    : min=0 max=255 step=1 default=57343 value=128
                 'white_balance_temperature_auto': 0,#white_balance_temperature_auto (bool)   : default=1 value=1
                 'gain': 0,#gain (int)    : min=0 max=255 step=1 default=57343 value=0
                 'power_line_frequency': 0,#power_line_frequency (menu)   : min=0 max=2 default=2 value=2
                 'white_balance_temperature': 4000,#white_balance_temperature (int)    : min=2000 max=6500 step=1 default=57343 value=4000 flags=inactive
                 'sharpness': 128,#sharpness (int)    : min=0 max=255 step=1 default=57343 value=128
                 'backlight_compensation': 0,#backlight_compensation (int)    : min=0 max=1 step=1 default=57343 value=0
                 'exposure_auto': 1,#exposure_auto (menu)   : min=0 max=3 default=0 value=3
                 'exposure_absolute': 300,#exposure_absolute (int)    : min=3 max=2047 step=1 default=250 value=250 flags=inactive
                 'exposure_auto_priority': 0,#exposure_auto_priority (bool)   : default=0 value=1
                 'pan_absolute': 0,#pan_absolute (int)    : min=-36000 max=36000 step=3600 default=0 value=0
                 'tilt_absolute': 0,#tilt_absolute (int)    : min=-36000 max=36000 step=3600 default=0 value=0
                 'focus_absolute': 100,#250 is near; 0 is far;focus_absolute (int)    : min=0 max=250 step=5 default=8189 value=0 flags=inactive
                 'focus_auto': 0,#focus_auto (bool)   : default=1 value=1
                 'zoom_absolute': 100}#zoom_absolute (int)    : min=100 max=500 step=1 default=57343 value=100
    for key in cam_props:
        subprocess.call(['v4l2-ctl -d /dev/video0 -c {}={}'.format(key, str(cam_props[key]))], shell=True)
    start=how_long(start,'subprocess ')
    #
    subprocess.call(['v4l2-ctl -d /dev/video0 -l'], shell=True)
    #
    cam = cv2.VideoCapture(0)#use USB cam
    time.sleep(0.5)#camera warmup for half second
    #
#    cam.set(cv2.CAP_PROP_FPS,120)#try to get 60fps from webcam
#    cam.set(cv2.CAP_PROP_FRAME_WIDTH,framewidth)#1280)#1920)#reasonable resolution for fast image processing
#    cam.set(cv2.CAP_PROP_FRAME_HEIGHT,frameheight)#720)#1080)
    #
    cv2.namedWindow("Show", cv2.WINDOW_NORMAL)#Declare the image window
    cv2.setMouseCallback('Show',return_mouse_click)#event driven hook to this window
    cv2.namedWindow("Showbbox", cv2.WINDOW_NORMAL)#
    cv2.namedWindow("ShowDevFrame", cv2.WINDOW_NORMAL)#
    cv2.namedWindow("ccframe2 plotted", cv2.WINDOW_NORMAL)#
#    cv2.setWindowProperty("Show",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)#fullscreen for FPV use
    ret_val, frame = cam.read() #must init & warm-up cam, this frame usually takes .250sec
    time.sleep(.25)
    tpoint=(int(framewidth/2), int(frameheight/2))
    width=60
    SelctPtValB=int(127)
    SelctPtValG=int(127)
    SelctPtValR=int(127)
    n=int(0)
    start=time.time()
    while loop==1:#*******************************************
#        start=time.time()
        ret_val, frame = cam.read() #read frame from camera
        tpoint,newwidth,subframe,box,BandPass,sumBx,sumBy= simple_tracker(tpoint,width,frame)#
        cv2.rectangle(frame,(int(box[0]),int(box[1])),(int(box[2]),int(box[3])),(0,0,255),2)
        cv2.line(frame,(int(tpoint[0]),0),(int(tpoint[0]),int(frame.shape[0]-1)),(0,255,0),1)
        cv2.line(frame,(0,int(tpoint[1])),(int(frame.shape[1]-1),int(tpoint[1])),(0,255,0),1)
        cv2.imshow("Show", frame)# Display result
        cv2.imshow("Showbbox", subframe)# Display result
        cv2.imshow("ShowDevFrame", BandPass)
        print(np.amax(BandPass),' BandPass max val')
        print(np.amin(BandPass),' BandPass min val')
        if showblob==1:#double click on target
            clickflag=0
            showblob=0
            blob2,ccframe2=ccblobfinder(BandPass)
            if np.any(ccframe2):
                cv2.imshow("ccframe2 plotted", ccframe2)
        width=newwidth
        stop=time.time()
        print(int(1/(stop-start)),' Hz')
        if cv2.waitKey(50) == 27:# esc to quit; this .waitKey consumes at least 22ms time
                stop_loop(frame,cam)
                break
        start=time.time()

if __name__ == '__main__':
    main()
