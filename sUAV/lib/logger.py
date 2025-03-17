# TCP Socket
# Import the necessary libraries 
import logging
from sUAV.lib.constants import *
import os
from datetime import datetime
import time
from logging.handlers import RotatingFileHandler

class Logger():
    """Class for handling the logging while the system is running"""

    def __init__(self, app_name="sUAV_System", log_dir=LOG_DIR):
        """Initialization function that sets up the logging and logger"""
        
        self.app_name = app_name
        self.log_dir = log_dir

        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        timestamp = datetime.now().strftime("%Y%m%d")
        current_time = str(time.time())
        self.log_file = os.path.join(log_dir, f"{app_name}_{timestamp}_{current_time}.log")

        self.logger = logging.getLogger(app_name)
        self.logger.setLevel(logging.DEBUG)

        if self.logger.handlers:
            self.logger.handlers.clear()

        file_formatter = logging.Formatter(
            '%(asctime)s - %(levelname)s - [%(funcName)s] - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        console_formatter = logging.Formatter(
            '%(levelname)s - %(message)s'
        )
        file_handler = RotatingFileHandler(
            self.log_file,
            maxBytes=5*1024*1024,  # 5MB
            backupCount=5
        )
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(file_formatter)

        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_handler.setFormatter(console_formatter)

        self.logger.addHandler(file_handler)
        self.logger.addHandler(console_handler)

    def debug(self, message):
        """Log debug level message"""
        if LOGGING:
            self.logger.debug(message)

    def info(self, message):
        """Log info level message"""
        if LOGGING:
            self.logger.info(message)

    def warning(self, message):
        """Log warning level message"""
        if LOGGING:
            self.logger.warning(message)
    
    def error(self, message):
        """Log error level message"""
        if LOGGING:
            self.logger.error(message)
    
    def critical(self, message):
        """Log critical level message"""
        if LOGGING:
            self.logger.critical(message)
        