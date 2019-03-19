import logging
import logging.handlers

from log.log import setup_logging


def test_log_setup_level_debug():
    setup_logging()
    logger = logging.getLogger()
    assert logger.level == logging.DEBUG


def test_log_setup_socket_handler_configured():
    setup_logging()
    logger = logging.getLogger()
    socket_handler = [
        h for h in logger.handlers
        if isinstance(h, logging.handlers.SocketHandler)
    ]
    assert socket_handler is not None


def test_log_setup_file_handler_configured():
    setup_logging()
    logger = logging.getLogger()
    file_handler = [
        h for h in logger.handlers
        if isinstance(h, logging.FileHandler)
    ]
    assert file_handler is not None
