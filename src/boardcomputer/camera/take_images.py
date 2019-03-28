import datetime
import io
import time

import picamera
from PIL import Image

with picamera.PiCamera() as camera:
    # Set the camera's resolution to VGA @40fps and give it a couple
    # of seconds to measure exposure etc.
    # camera.rotation = 180

    #
    camera.resolution = (1280, 960)
    camera.brightness = 55
    camera.exposure_mode = 'off'
    camera.shutter_speed = 10000
    camera.framerate = 2
    time.sleep(4)

    # Set up 40 in-memory streams
    outputs = [io.BytesIO() for i in range(30)]
    print(datetime.datetime.now())
    start = time.time()
    camera.capture_sequence(outputs, 'jpeg', use_video_port=True)
    finish = time.time()
    print(datetime.datetime.now())
    # How fast were we?
    print('Captured 10 images at %.2ffps' % (40 / (finish - start)))

    i = 0
    for output in outputs:
        i = i + 1
        img = Image.open(output, mode='r')

        img.save(("/home/pi/pren/src/boardcomputer/camera/img/img" + str(i) + ".jpg"),
                 format='jpeg')
