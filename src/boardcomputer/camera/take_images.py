import base64
import io
import threading
import time
from time import sleep

import socketio
from picamera import PiCamera
from picamera.array import PiRGBArray

sio = socketio.Client()
sio.connect('http://localhost:5000')

with PiCamera() as camera:
    # Set the camera's resolution to VGA @40fps and give it a couple
    # of seconds to measure exposure etc.
    # camera.rotation = 180handler.write_message(msg)

    #
    camera.resolution = (960, 640)
    camera.brightness = 50
    camera.framerate = 15
    camera.shutter_speed = 100000
    camera.exposure_mode = "off"
    camera.rotation = 0

    rawCapture = PiRGBArray(camera, size=(960, 640))

    time.sleep(0.1)

    for frame in camera.capture_continuous(rawCapture, format='jpg', use_video_port=True):
        image = frame.array

        rawCapture.truncate(0)
        print("Picture")

        b64_bytes = base64.b64encode(image)
        b64_str = b64_bytes.decode()

        sio.emit('pictureSet', str(data_uri))
        sleep(0.02)

        break

sio.disconnect()


class imageToWebserverThread:

    def __init__(self, buffer: io.BytesIO):

        self.buffer = buffer

        self._stopped = False

        self.thread1 = threading.Thread(target=self.read_thread)
        self.thread1.start()

    def thread(self):
        while not self._stopped:
            if self.buffer.readable():
                pass

    def stop(self):
        self._stopped = True
        self.thread1.join()
