try:
    import serial
except:
    serial = None
    print('INFO: Can\'t import module \'serial\' on dev computer. Import is ignored.')

# ---------------------------------------------------------------------------
#   Level related stuff
# ---------------------------------------------------------------------------
#
# Default levels and level names, these can be replaced with any positive set
# of values having corresponding names. There is a pseudo-level, NOTSET, which
# is only really there as a lower limit for user-defined levels. Handlers and
# loggers are initialized with NOTSET so that they will log all messages, even
# at user-defined levels.
#

CRITICAL = 50
FATAL = CRITICAL
ERROR = 40
WARNING = 30
WARN = WARNING
INFO = 20
DEBUG = 10
NOTSET = 0


class SoundConfig:

    HUMAN_OUTPUT_ENABLE = True
    HUMAN_SOUNDFILE_BASEPATH = './soundfiles/'
    HUMAN_SOUNDFILE_PATHS = {
        0: '0.mp3',
        1: '1.mp3',
        2: '2.mp3',
        3: '3.mp3',
        4: '4.mp3',
        5: '5.mp3',
        6: '6.mp3',
        7: '7.mp3',
        8: '8.mp3',
        9: '9.mp3'
    }

    BEEP_SOUNDFILE_BASEPATH = './soundfiles/'
    BEEP_SOUNDFILE_PATH = 'beep-08b.mp3'

    PCM_SETTING = '100%'

class BaseImageAnalysisCPUConfig:
    DEVICE = 'CPU'
    MODEL_NAME = 'frozen_darknet_yolov3_tiny_model_signals_CPU'
    DRAW_RECTANGLES = True


class BaseImageAnalysisNCS2Config:
    DEVICE = 'MYRIAD'
    MODEL_NAME = 'frozen_darknet_yolov3_tiny_model_signals_MYRIAD'
    DRAW_RECTANGLES = False


class BaseCameraConfig:
    CAMERA_RESOLUTION = (416, 416)
    CAMERA_BRIGHTNESS = 55  # 0 - 100, default 50
    CAMERA_FRAMERATE = 15
    CAMERA_SHUTTERSPEED = 3100
    CAMERA_EXPOSUREMODE = 'off'
    CAMERA_ROTATION = 0
    CAMERA_ISO = 1200  # 0 = auto, 1600 = max


class BaseUARTConfig:
    SERIAL_PORT = '/dev/serial0'
    SERIAL_BAUD = 19200
    SERIAL_STOPBIT = 1
    if (serial != None):
        SERIAL_PARITY = serial.PARITY_NONE


class BaseConfig(BaseCameraConfig, BaseUARTConfig, SoundConfig):

    APP_NAME = 'TrainManagementSystem'
    APP_IMPORT_PATH = './'

    LOG_WEBSERVER_ADDRESS = 'http://192.168.10.1:5000'

    LOG_ENABLE_WEB_LOGGING = None
    LOG_ENABLE_WEB_PICTURE_LOGGING = None

    """ 
    If set to True: Data written to the TICP Interface will be loopbacked over memory without 
    using the Hardware Interface
    """
    TICP_ENABLE_DEBUG = None

    LOG_LEVEL = None

    RUN_NUMBEROFROUNDS = 2

    # Cooldown after Detecting a Start Signal
    START_SIGNAL_DETECTION_COOLDOWN = 4
    INIT_CAMERA_IN_TMS = False


class DevelopmentConfig(BaseConfig):
    LOG_LEVEL = DEBUG
    LOG_ENABLE_WEB_LOGGING = True
    LOG_ENABLE_WEB_PICTURE_LOGGING = True

    TICP_ENABLE_DEBUG = True


class TestConfig(BaseConfig):
    LOG_LEVEL = DEBUG
    LOG_ENABLE_WEB_LOGGING = True
    LOG_ENABLE_WEB_PICTURE_LOGGING = True

    TICP_ENABLE_DEBUG = False


class ProductionConfig(BaseConfig):
    LOG_LEVEL = INFO
    LOG_ENABLE_WEB_LOGGING = True
    LOG_ENABLE_WEB_PICTURE_LOGGING = False
    TICP_ENABLE_DEBUG = False
