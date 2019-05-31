'''
This script is used to gather images for the dataset.
'''
import os
import sys
sys.path.insert(0, '/home/pi/boardcomputer')

import cv2
import time
import numpy as np
import multiprocessing as mp
from fsm.config import BaseConfig
from picamera import PiCamera

print(os.getcwd())

base_config = BaseConfig()

# PyCharm might not update the config file because it's being a fucking cunt.
# therefore set values in this file for now.
camera = PiCamera()
camera.exposure_mode = base_config.CAMERA_EXPOSUREMODE
camera.resolution = base_config.CAMERA_RESOLUTION
camera.brightness = 55
camera.framerate = 15
camera.shutter_speed = 3100
camera.rotation = base_config.CAMERA_ROTATION
camera.iso = 1200
time.sleep(2)

images = []

start_image_id = 0
num_images = 1000

count = start_image_id
max_count = start_image_id + num_images

while count < max_count:
    color_image = np.empty((camera.resolution[1], camera.resolution[0], 3), dtype=np.uint8)
    camera.capture(color_image, format='bgr', use_video_port=True)
    images.append(color_image)
    count = count + 1
    print("images captured: " + str(count) + "/" + str(max_count))

count = start_image_id
for image in images:
    cv2.imwrite("/home/pi/boardcomputer/image_analysis/dataset/img-0-" + str(count) + ".jpg", image)
    print("images saved: " + str(count) + "/" + str(max_count))
    count = count + 1
