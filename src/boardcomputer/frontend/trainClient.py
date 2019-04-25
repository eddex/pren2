import base64
import io

import socketio
from PIL import Image

sio = socketio.Client()
sio.connect('http://localhost:8080')

"""
@sio.on('connect')
def on_connect():
    print('connection established')

@sio.on('my message')
def on_message(data):
    print('message received with ', data)
    sio.emit('my response', {'response': 'my response'})

@sio.on('disconnect')
def on_disconnect():
    print('disconnected from server')
"""

for i in range(50):
    pic = "../camera/tmp/img2/img" + str(i + 1) + ".jpg"
    im = Image.open(pic, 'r')
    buffer = io.BytesIO()
    im.save(buffer, format='jpeg')
    buffer.seek(0)
    data_uri = base64.b64encode(buffer.read()).decode('ascii')

    sio.emit('pictureSet', data_uri)

sio.disconnect()
