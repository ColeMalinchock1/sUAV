# Import the necessary libraries 
import logging
from sUAV.lib.constants import *
import os
from datetime import datetime
import time
from logging.handlers import RotatingFileHandler
import socket
import json

class UDPHandler(logging.Handler):
    """Handler for sending log messages over UDP"""
    
    def __init__(self, host, port):
        """Initialize the UDP handler with target host and port"""
        logging.Handler.__init__(self)
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    def emit(self, record):
        """Send the log record over UDP"""
        try:
            # Format the log message
            msg = self.format(record)
            
            # Create a JSON structure with the log info
            log_data = {
                "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "level": record.levelname,
                "function": record.funcName,
                "message": record.getMessage()
            }
            
            # Convert to JSON string and encode to bytes
            json_msg = json.dumps(log_data).encode('utf-8')
            
            # Send the message over UDP
            self.sock.sendto(json_msg, (self.host, self.port))
        except Exception:
            # If any error occurs, just continue (no retry, buffer, etc.)
            self.handleError(record)

class Logger():
    """Class for handling the logging while the system is running"""

    def __init__(self, app_name="sUAV_System", log_dir=LOG_DIR, udp_host=None, udp_port=None):
        """Initialization function that sets up the logging and logger"""
        
        self.app_name = app_name
        self.log_dir = log_dir
        self.udp_host = udp_host
        self.udp_port = udp_port

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
        
        # Add UDP handler if host and port are provided
        if udp_host and udp_port:
            udp_formatter = logging.Formatter(
                '%(asctime)s - %(levelname)s - [%(funcName)s] - %(message)s',
                datefmt='%Y-%m-%d %H:%M:%S'
            )
            udp_handler = UDPHandler(udp_host, udp_port)
            udp_handler.setLevel(logging.INFO)  # Set the appropriate level
            udp_handler.setFormatter(udp_formatter)
            self.logger.addHandler(udp_handler)

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

    def add_udp_handler(self, host, port, log_level=logging.INFO):
        """Add a UDP handler to the logger after initialization"""
        udp_formatter = logging.Formatter(
            '%(asctime)s - %(levelname)s - [%(funcName)s] - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        udp_handler = UDPHandler(host, port)
        udp_handler.setLevel(log_level)
        udp_handler.setFormatter(udp_formatter)
        self.logger.addHandler(udp_handler)
        self.udp_host = host
        self.udp_port = port