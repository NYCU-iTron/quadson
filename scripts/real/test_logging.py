from common.logging_config import setup_logging
import logging

if __name__ == '__main__':
    setup_logging()
    logging.getLogger(__name__)
    logging.debug('This is a debug message.')
    logging.info('This is an info message.')
    logging.warning('This is a warning message.')
    logging.error('This is an error message.')
    logging.critical('This is a critical message.')
