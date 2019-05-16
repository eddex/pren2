import socketio


class WebStream:

    def __init__(self, url: str):
        """
        Interface for Logging on Web Server

        Args:
            url: URL of Webserver 'http://192.168.10.1:5000'
        """
        self.sio = socketio.Client()
        self.sio.connect('http://192.168.10.1:5000')

    def write(self, msg):
        self.sio.emit('message', msg)

    def flush(self):
        pass

    def close(self):
        self.sio.disconnect()


class WebLoggerStream:

    def __init__(self, socket):
        """
        Interface for Logging on Web Server

        Args:
            socket: URL of Webserver 'http://192.168.10.1:5000'
        """
        self.sio = socket

    def write(self, msg):
        self.sio.emit('message', msg)

    def flush(self):
        pass

    def close(self):
        self.sio.disconnect()
