import logging
from logging import handlers

LOG_FILENAME = '../log/exercise.log'
FORMAT = '%(asctime)s %(name)-12s %(levelname)-8s %(message)s'
DATEFMT='%y-%m-%d %H:%M:%S'

class MyLogger(object):
    def __init__(self):
        logging.basicConfig(format=FORMAT, datefmt=DATEFMT, filename=LOG_FILENAME, filemode='a')
        self.logger = logging.getLogger()
        self.logger.setLevel(logging.INFO)
        handler = handlers.TimedRotatingFileHandler(LOG_FILENAME, when='d', interval=1)
        self.logger.addHandler(handler)