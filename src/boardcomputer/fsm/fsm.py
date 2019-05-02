import logging
import sys


# sys.path.append('/opt/settings')
# import config


class FSM:

    def __init__(self):
        """TODO"""
        self.log = logging.getLogger()

    def init_logging(self, logLevel=logging.INFO):
        self.log.setLevel(logLevel)


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
