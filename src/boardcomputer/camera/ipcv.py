import base64
import time

import cv2
# import matplotlib.pyplot as plt
import numpy as np
import socketio
from picamera import PiCamera

from fsm.config import BaseConfig

config = BaseConfig()

_camera = PiCamera()
_camera.resolution = config.CAMERA_RESOLUTION
_camera.brightness = config.CAMERA_BRIGHTNESS
_camera.framerate = config.CAMERA_FRAMERATE
_camera.shutter_speed = config.CAMERA_SHUTTERSPEED
_camera.exposure_mode = config.CAMERA_EXPOSUREMODE
_camera.rotation = config.CAMERA_ROTATION
# _camera.color_effects = (128 , 128)
_camera.iso = config.CAMERA_ISO

time.sleep(2)

_socketio = socketio.Client()
_socketio.connect('http://localhost:5000')


def gaussian_kernel(size, sigma=1):
    size = int(size) // 2
    x, y = np.mgrid[-size:size + 1, -size:size + 1]
    normal = 1 / (2.0 * np.pi * sigma ** 2)
    g = np.exp(-((x ** 2 + y ** 2) / (2.0 * sigma ** 2))) * normal

    return g


def showPic(img):
    image = cv2.imencode(".jpeg", img)[1]

    b64_bytes = base64.b64encode(image)
    b64_str = b64_bytes.decode()

    _socketio.emit('pictureSet', b64_str)


def showPicSmall(img):
    image = cv2.imencode(".jpeg", img)[1]

    b64_bytes = base64.b64encode(image)
    b64_str = b64_bytes.decode()

    _socketio.emit('smallPictureSet', b64_str)


def image_resize(image, long_edge=32, inter=cv2.INTER_AREA):
    # initialize the dimensions of the image to be resized and
    # grab the image size
    dim = None
    (h, w) = image.shape[:2]

    # check to see if the width is None
    if h > w:
        # calculate the ratio of the height and construct the
        # dimensions
        r = long_edge / float(h)
        dim = (int(w * r), long_edge)

    # otherwise, the height is None
    else:
        # calculate the ratio of the width and construct the
        # dimensions
        r = long_edge / float(w)
        dim = (long_edge, int(h * r))

    # resize the image
    final = np.zeros((long_edge, long_edge))

    resized = cv2.resize(image, dim, interpolation=inter)

    final[0:resized.shape[0], 0:resized.shape[1]] = resized
    # return the resized image
    return final


def analyse_image(image, ratioBoundaries: [], ratioAreaBoundaries: []):
    nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(image,
                                                                               connectivity=8)
    min_size = 100
    max_size = 3000

    extracted_img_size = 64

    img2 = np.zeros((output.shape))

    components = []
    for j in range(1, nb_components):

        ratioSize = stats[j][3] / stats[j][2]

        long_edge = stats[j][2] if stats[j][2] > stats[j][3] else stats[j][3]
        ratioArea = stats[j][4] / long_edge ** 2

        if min_size <= stats[j][4] <= max_size and ratioBoundaries[0] < ratioSize < ratioBoundaries[
            1] and ratioAreaBoundaries[0] < ratioArea < ratioAreaBoundaries[1]:

            img2[output == j] = 255

            bounding_box_pos_x = stats[j][1]
            bounding_box_pos_y = stats[j][0]

            bounding_box_len_x = stats[j][3]
            bounding_box_len_y = stats[j][2]

            component = output[bounding_box_pos_x:bounding_box_pos_x + bounding_box_len_x,
                        bounding_box_pos_y:bounding_box_pos_y + bounding_box_len_y]

            component[component == j] = 255

            offset_x = int((long_edge - bounding_box_len_x) / 2)
            offset_y = int((long_edge - bounding_box_len_y) / 2)

            new_x = bounding_box_pos_x - offset_x if bounding_box_pos_x - offset_x > 0 else 0
            new_y = bounding_box_pos_y - offset_y if bounding_box_pos_y - offset_y > 0 else 0

            if new_x + long_edge < output.shape[0]:

                if new_y + long_edge < output.shape[1]:
                    subPicture = img[new_x:new_x + long_edge, new_y:new_y + long_edge]
                else:
                    subPicture = img[new_x:new_x + long_edge,
                                 output.shape[1] - long_edge:output.shape[1]]

            else:
                if new_y + long_edge < output.shape[1]:
                    subPicture = img[output.shape[0] - long_edge:output.shape[0] + long_edge,
                                 new_y:new_y + long_edge]
                else:
                    subPicture = img[output.shape[0] - long_edge:output.shape[0] + long_edge,
                                 output.shape[1] - long_edge:output.shape[1]]

            component = image_resize(component.astype(np.uint8), extracted_img_size,
                                     cv2.INTER_NEAREST)
            subPicture = cv2.resize(subPicture.astype(np.uint8),
                                    (extracted_img_size, extracted_img_size))
            component = cv2.resize(component.astype(np.uint8), (64, 64),
                                   interpolation=cv2.INTER_AREA)

            components.append((component, subPicture))

    return components, img2


def image_preprocessing(img):
    img_equalized = cv2.equalizeHist(img)
    img_brigh = cv2.add(-120, img_equalized)

    return img_brigh


raw_img = np.empty((config.CAMERA_RESOLUTION[1], config.CAMERA_RESOLUTION[0], 3), dtype=np.uint8)

while True:
    # for i in range(1, 1500):

    _camera.capture(raw_img, format='bgr', use_video_port=True)
    # print(str(i))
    # path1 = '/media/kevinr/rootfs/home/pi/image/img1/image{}.jpg'.format(i)
    # path2 = './tmp/img/img{}.jpg'.format(i)
    # img = cv2.imread(path1, 0)
    # img = cv2.resize(img, None, fx=0.75, fy=0.75, interpolation=cv2.INTER_AREA)

    img, _, _ = cv2.split(raw_img)

    # kernel = np.ones((3, 3), np.uint8)
    # img_filter = cv2.dilate(img, kernel, iterations = 1)

    # kernel = np.ones((3, 3), np.uint8)
    # img_filter = cv2.erode(img, kernel, iterations=1)
    # img_filter = cv2.fastNlMeansDenoising(img, 10, 7,21)

    # ret, thresh1 = cv2.threshold(img,220,255,cv2.THRESH_BINARY_INV)

    img_processed = image_preprocessing(img)

    ret, thresh1 = cv2.threshold(img_processed, 20, 255, cv2.THRESH_BINARY)
    # ret, thresh1 = cv2.threshold(img, 25, 255, cv2.THRESH_BINARY)
    components1, img1 = analyse_image(thresh1, (0.5, 5), (0.05, 0.7))

    # kernel = np.ones((3, 3), np.uint8)
    # img2_filter = cv2.erode(img, kernel, iterations=1)

    ret, thresh2 = cv2.threshold(img_processed, 20, 255, cv2.THRESH_BINARY_INV)
    # ret, thresh2 = cv2.threshold(cv2.bitwise_not(img), 15, 255, cv2.THRESH_BINARY)
    components2, img2 = analyse_image(thresh2, (0.5, 6), (0.05, 0.7))

    components = components1 + components2
    # sizes = stats[1:, -1]; nb_components = nb_components - 1

    img3 = cv2.bitwise_or(img1, img2)

    alpha = 0.5
    beta = (1.0 - alpha)
    dst = cv2.addWeighted(img, alpha, img3.astype(np.uint8), beta, 0.0)
    showPic(dst)
    # showPic(thresh2)

    for comp in components:
        showPicSmall(comp[1])
        # time.sleep(0.05)
        # input("C?")

# time.sleep(0.01)

_socketio.disconnect()
