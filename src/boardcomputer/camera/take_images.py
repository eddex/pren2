import base64
import io
import threading
import time
from time import sleep

import picamera
import socketio

sio = socketio.Client()
sio.connect('http://localhost:5000')

with picamera.PiCamera() as camera:
    # Set the camera's resolution to VGA @40fps and give it a couple
    # of seconds to measure exposure etc.
    # camera.rotation = 180handler.write_message(msg)

    #
    camera.resolution = (960, 640)
    camera.brightness = 50
    camera.shutter_speed = 100000
    camera.exposure_mode = "off"
    camera.framerate = 8
    camera.rotation = 0
    time.sleep(1)

    # Set up 40 in-memory streams
    i = 0

    stream = io.BytesIO()
    for picture in camera.capture_continuous(stream, format='jpeg', use_video_port=True):
        print("Picture")
        stream.truncate()
        stream.seek(0)

        data_uri = base64.b64encode(stream.read()).decode('ASCII')
        stream.seek(0)

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
