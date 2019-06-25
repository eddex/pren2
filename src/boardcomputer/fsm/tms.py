import base64
import logging
import queue
import sys
import threading
from collections import Counter
from enum import Enum
from time import sleep, time

import cv2
import numpy as np
import serial
import socketio

from frontend.WebStream import WebLoggerStream
from fsm.config import BaseConfig, DevelopmentConfig, TestConfig, ProductionConfig
from fsm.signals import SignalType, Signal
from image_analysis.image_analysis_multithreading import ImageAnalyzer 
from uart_handler.ticp_handler import FSMStateStatusListenerInterface
from uart_handler.ticp_handler import TICPHandler
from uart_handler.ticp_handler import _SerialInterfaceMock
from uart_handler.ticp_handler import _TICPToSerialAdapter
from uart_handler.ticp_message import TICPMessageAllCommandData

sys.path.append(BaseConfig.APP_IMPORT_PATH)


class MCFSMStates(Enum):
    STARTUP = 0
    WURFEL_ERKENNEN = 1
    SERVO_RUNTER = 2
    SERVO_RAUF = 3
    WURFEL_KONTROLLE = 4
    STARTPOSITION = 5
    SCHNELLFAHRT = 6
    BREMSEN = 7
    FINALES_HALTESIGNAL = 8
    HALTESIGNAL_ANFART = 9
    HALTESIGNAL_STOPPEN = 10
    STOP = 11


class TrainManagementSystemStates(Enum):
    pass


class TrainManagementSystem(FSMStateStatusListenerInterface):

    def __init__(self, configuration: BaseConfig):

        self.config = configuration
        self.serial_lock = threading.Lock()
        self._log = None
        self._serial = None
        self._socketio = None
        self.ticp_handler = None
        self._image_amalyzer = ImageAnalyzer()
        self._image_amalyzer.initialize()

        self._init_logging_local()

        if self.config.LOG_ENABLE_WEB_LOGGING:
            try:
                self._init_socketio()
                self._init_logging_webserver()

            except:
                self.log.info("Unable to connect to Webservice. Logging local instead")

        self._init_uart()

        if self.config.INIT_CAMERA_IN_TMS:
            self._camera = None
            self._init_camera()


        self._init_image_analyzer()

        self._current_mcu_fsm = MCFSMStates(0)
        self.round_counter = 0
        self.rounds_to_drive = self.config.RUN_NUMBEROFROUNDS
        self.info_signals = []
        self.running = True

    def run(self):

        self._initialize()

        while self.running or self._current_mcu_fsm != MCFSMStates.STOP:
            pass


    def _initialize(self):
        self.log.info("Train Inizialized")
        msg = TICPMessageAllCommandData.from_values(start_signal=True, round_counter=0,
                                                    stop_signal=False)
        self.ticp_handler.write_message(msg)
        self.log.info("Startsignal send to Microcontroler")

    def _signal_recognition(self):

        self.log.info("Start with Signal recognition")

        t_start_signal_detected = time()

        while self.round_counter < self.rounds_to_drive:

            signal = self._image_amalyzer.detect_signal()

            if signal is None:
                self.log.info("No Signal detected")

            elif signal.signal_type is SignalType.INFO:
                self.log.info(
                    "Info Signal with number {} was deteced and logged".format(signal.number))
                self.info_signals.append(signal)

            elif signal.signal_type is SignalType.STOP:
                self.log.info("Stop Signal with number {} was deteced".format(signal.number))

            elif signal.signal_type is SignalType.START:
                self.log.info("Start Signal detected")
                t_detection = time()
                dt = t_detection - t_start_signal_detected

                if dt > self.config.START_SIGNAL_DETECTION_COOLDOWN:
                    self.log.info("Round Counter was incrementet")
                    self.round_counter += 1
                    t_start_signal_detected = time()
                else:
                    self.log.info("Signal deteted during Cooldown and therefore no actiion taken")


            # Send updated Status to MC
            msg = TICPMessageAllCommandData.from_values(start_signal=True,
                                                        round_counter=self.round_counter,
                                                        stop_signal=False)
            self.ticp_handler.write_message(msg)

        self._searching_stop_signal()

    def _searching_stop_signal(self):

        self.log.info("Start searching for stop signal")
        self._image_amalyzer.search_high_signals = False
        # Returns the Stop Signal which was detected most
        info_signal = Counter(self.info_signals).most_common()[0][0]

        self.log.info("Stopping as soon as {} Signal is detected".format(info_signal.number))

        stop_signal_detected = False

        while not stop_signal_detected:

            signal = self._image_amalyzer.detect_signal()

            if signal is None:
                self.log.info("No Signal detected")

            elif signal.signal_type is SignalType.STOP:

                if signal.number is info_signal.number:
                    stop_signal_detected = True

        msg = TICPMessageAllCommandData.from_values(start_signal=True,
                                                    round_counter=self.round_counter,
                                                    stop_signal=True)

        for _ in range(0,10):
            self.ticp_handler.write_message(msg)

        self.shutdown()

    def shutdown(self):
        self.ticp_handler.stop_handler()
        self.running = False

    def notify_fsm_state_change(self, newFSM: int):
        self._current_mcu_fsm = MCFSMStates(newFSM)

        if self._current_mcu_fsm is MCFSMStates.SCHNELLFAHRT:
            self._signal_recognition()

    def _upload_image(self, image) -> None:

        image = cv2.imencode(".jpg", image)[1]
        b64_bytes = base64.b64encode(image)
        b64_str = b64_bytes.decode()

        try:
            self._socketio.emit('pictureSet', b64_str)
        except:
            self.log.info("Unable to upload image")

    def _init_logging_local(self):
        self.log = logging.getLogger()
        self.log.setLevel(self.config.LOG_LEVEL)

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
        self.ticp_handler.subscribe_fsm_state_change(self)

        self.log.info("TICP Interface initialized")

    def _init_image_analyzer(self):
        self._image_amalyzer = ImageAnalyzer()
        self._image_amalyzer.initialize()

    def _init_camera(self):
        self.log.info("Start Initialize Camera:")

        # TODO Define Camera Module
        # self._camera = PiCamera()
        self._camera.resolution = self.config.CAMERA_RESOLUTION
        self._camera.brightness = self.config.CAMERA_BRIGHTNESS
        self._camera.framerate = self.config.CAMERA_FRAMERATE
        self._camera.shutter_speed = self.config.CAMERA_SHUTTERSPEED
        self._camera.exposure_mode = self.config.CAMERA_EXPOSUREMODE
        self._camera.rotation = self.config.CAMERA_ROTATION
        self._camera.iso = self.config.CAMERA_ISO

        sleep(2)

        self.log.info("Camera initialized")

    def _init_socketio(self):
        self.log.info("Start Initializing SocketIO Client:")

        self._socketio = socketio.Client()
        self._socketio.connect(self.config.LOG_WEBSERVER_ADDRESS)

        self.log.info("Start Initializing SocketIO Client:")

    def _init_logging_webserver(self):
        stream = WebLoggerStream(self._socketio)
        web_logger = logging.StreamHandler(stream)
        web_logger.setLevel(self.config.LOG_LEVEL)
        web_logger.terminator = ''

        formatter = logging.Formatter('%(asctime)s %(name)-12s: %(levelname)-8s %(message)s')
        web_logger.setFormatter(formatter)

        self.log.addHandler(web_logger)


if __name__ == "__main__":

    environment = sys.argv[1] if len(sys.argv) > 2 else 'dev'

    if environment == 'dev':
        tms = TrainManagementSystem(DevelopmentConfig())
        tms.run()

    elif environment == 'test':
        tms = TrainManagementSystem(TestConfig())
        tms.run()

    elif environment == 'prod':
        tms = TrainManagementSystem(ProductionConfig())
        tms.run()

    else:
        raise ValueError('Invalide enviroment name')
