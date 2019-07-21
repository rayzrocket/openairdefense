#based on code near bottom of this page:
#    https://picamera.readthedocs.io/en/release-0.6/recipes.html

import time
import picamera

frames = 20

with picamera.PiCamera() as camera:
#    camera.sensor_mode=7
    camera.resolution = (640,480)
    camera.framerate = 90
    camera.iso=300
    time.sleep(2)#allow auto gain to settle
    camera.shutter_speed=camera.exposure_speed
    camera.exposure_mode='off'
    camera.awb_gains=7
    # Give the camera some warm-up time
    time.sleep(2)
    start = time.time()
    camera.capture_sequence([
        'image%02d.jpg' % i
        for i in range(frames)
        ], use_video_port=True)
    finish = time.time()
print('Captured %d frames at %.2ffps' % (
    frames,
    frames / (finish - start)))