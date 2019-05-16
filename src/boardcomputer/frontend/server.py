import socketio
from aiohttp import web

# HTTP Server Configuration

sio = socketio.AsyncServer()
app = web.Application()
sio.attach(app)


async def index(request):
    with open('html/index.html') as f:
        return web.Response(text=f.read(), content_type='text/html')


@sio.on('message')
async def handleMessage(socketID, msg):
    print('Message: ' + msg)
    await sio.emit('message', msg)


@sio.on('connect')
async def connection(socketID, wsgiEnviroment):
    print("Client Connected")
    await sio.emit('message', "User Connected")


@sio.on('disconnect')
async def disconnect(socketID):
    print("Client Disconnected")
    await sio.emit('message', "User Disconnected")


@sio.on('pictureSet')
async def pictureBroadcast(socketID, base64: str):
    print("Received and broadcasted Picture")
    await sio.emit('pictureBroadcast', base64)


@sio.on('smallPictureSet')
async def smallPictureBroadcast(socketID, base64: str):
    print("Received and broadcasted Picture")
    await sio.emit('smallPictureBroadcast', base64)


app.router.add_get('/', index)
app.router.add_static('/', path='./html/')

if __name__ == '__main__':
    web.run_app(app, host='0.0.0.0', port=5000)
