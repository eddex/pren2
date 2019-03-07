import logging
import logging.handlers
import datetime


def setup_logging():
    """
    setup the python logging framework
    the logging module is a singleton: other modules can use logging.info(...) directly after this setup
    see https://stackoverflow.com/questions/5505187/single-logger-in-python-project
    Documentation: https://docs.python.org/2/library/logging.handlers.html
    """
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.DEBUG)

    socket_handler = logging.handlers.SocketHandler('localhost', logging.handlers.DEFAULT_TCP_LOGGING_PORT)
    root_logger.addHandler(socket_handler)

    formatter = logging.Formatter('%(asctime)s [%(levelname)s] %(module)s: %(message)s', datefmt='%d.%m.%Y %H:%M:%S')
    filename = datetime.datetime.now().strftime('%d-%b-%Y_%H.%M.%S.log')
    file_handler = logging.FileHandler(filename, mode='a')
    file_handler.setFormatter(formatter)
    root_logger.addHandler(file_handler)
