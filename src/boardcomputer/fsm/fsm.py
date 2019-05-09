import logging
import queue
import sys
from enum import Enum

import serial

from frontend.WebStream import WebStream
from fsm import config
from fsm.config import BaseConfig

sys.path.append(config.BaseConfig.APP_IMPORT_PATH)

from uart_handler.ticp_handler import _TICPToSerialAdapter
from uart_handler.ticp_handler import _SerialInterfaceMock
from uart_handler.ticp_handler import TICPHandler


class TrainManagementSystemStates(Enum):
    pass


class FSM:

    def __init__(self, config: BaseConfig):
        """TODO"""
        self.config = config
        self._log = None
        self._serial = None
        self.ticp_handler = None
        self._camera = None

        self._init_logging()
        self._init_uart()
        self._init_camera()

        self.rounds_to_drive = config.RUN_NUMBEROFROUNDS

    def _init_logging(self):
        self.log = logging.getLogger()
        self.log.setLevel(self.config.LOG_LEVEL)

        if self.config.LOG_ENABLE_WEB_LOGGING:
            webstream = WebStream(self.config.LOG_WEBSERVER_ADDRESS)
            web_logger = logging.StreamHandler(webstream)
            web_logger.setLevel(self.config.LOG_LEVEL)
            web_logger.terminator = ''

            formatter = logging.Formatter('%(asctime)s %(name)-12s: %(levelname)-8s %(message)s')
            web_logger.setFormatter(formatter)

            self.log.addHandler(web_logger)

    def _init_uart(self):

        self.log.info("Start Initialize TICP Interface")
        if not self.config.TICP_ENABLE_DEBUG:
            self.serial = serial.Serial(port=self.config.SERIAL_PORT,
                                        baudrate=self.config.SERIAL_BAUD,
                                        parity=self.config.SERIAL_PARITY,
                                        stopbits=self.config.SERIAL_STOPBIT)

        else:
            self.log.info("TICP Debug Mode enabled")
            self.serial = _SerialInterfaceMock(queue.Queue(100))

        adapter = _TICPToSerialAdapter(self.serial)
        self.ticp_handler = TICPHandler(adapter)

        self.log.info("TICP Interface initialized")

    def _init_camera(self):
        self.log.info("Start Initialize Camera:")

        # TODO Define Camera Module
        # self._camera = PiCamera()
        self._camera.resolution = config.CAMERA_RESOLUTION
        self._camera.brightness = config.CAMERA_BRIGHTNESS
        self._camera.framerate = config.CAMERA_FRAMERATE
        self._camera.shutter_speed = config.CAMERA_SHUTTERSPEED
        self._camera.exposure_mode = config.CAMERA_EXPOSURMODE
        self._camera.rotation = config.CAMERA_ROTATION
        self._camera.iso = config.CAMERA_ISO

        self.log.info("Camera initialized")

    def _init_webservice(self):
        pass
        # TODO

    def run(self):
        pass
        # TODO


if __name__ == "__main__":

    environment = sys.argv[1] if len(sys.argv) > 2 else 'dev'

    if environment == 'dev':
        pass

    elif environment == 'test':
        pass

    elif environment == 'prod':
        pass

    else:
        raise ValueError('Invalide enviroment name')
