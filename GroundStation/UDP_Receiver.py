#!/usr/bin/env python3
"""
UDP Log Receiver for Drone Telemetry
------------------------------------
This script listens for UDP log messages from the drone and displays them
in a formatted way on the console.

Usage:
    python udp_log_receiver.py [--port PORT] [--save] [--logfile FILENAME]

Options:
    --port PORT       UDP port to listen on (default: 9999)
    --save            Save received logs to a file
    --logfile FILE    Specify log file name (default: drone_logs_TIMESTAMP.log)
"""

import socket
import json
import argparse
import datetime
import os
import signal
import sys
import time
from colorama import init, Fore, Style

# Initialize colorama for cross-platform colored terminal output
init()

# Color mapping for different log levels
LOG_COLORS = {
    "DEBUG": Fore.CYAN,
    "INFO": Fore.GREEN,
    "WARNING": Fore.YELLOW,
    "ERROR": Fore.RED,
    "CRITICAL": Fore.RED + Style.BRIGHT
}

class UDPLogReceiver:
    def __init__(self, port=9999, save_logs=False, log_file=None):
        """Initialize the UDP log receiver"""
        self.port = port
        self.save_logs = save_logs
        self.socket = None
        self.log_file = log_file
        self.log_fd = None
        self.running = False
        self.messages_received = 0
        self.start_time = None
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self.handle_shutdown)
        signal.signal(signal.SIGTERM, self.handle_shutdown)
        
    def setup(self):
        """Setup the UDP socket and log file"""
        # Create a UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('0.0.0.0', self.port))
        self.socket.settimeout(1.0)  # 1 second timeout for checking shutdown condition
        
        print(f"{Fore.CYAN}[*] Listening for drone logs on UDP port {self.port}{Style.RESET_ALL}")
        print(f"{Fore.CYAN}[*] Press Ctrl+C to stop{Style.RESET_ALL}")
        
        # Setup log file if needed
        if self.save_logs:
            if not self.log_file:
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                self.log_file = f"drone_logs_{timestamp}.log"
            
            log_dir = os.path.dirname(self.log_file)
            if log_dir and not os.path.exists(log_dir):
                os.makedirs(log_dir)
                
            self.log_fd = open(self.log_file, 'w')
            print(f"{Fore.CYAN}[*] Saving logs to {self.log_file}{Style.RESET_ALL}")
    
    def handle_shutdown(self, signum, frame):
        """Handle shutdown signals"""
        self.running = False
        
    def print_stats(self):
        """Print receiver statistics"""
        if self.start_time:
            duration = time.time() - self.start_time
            print(f"\n{Fore.CYAN}[*] Summary:{Style.RESET_ALL}")
            print(f"    Messages received: {self.messages_received}")
            print(f"    Duration: {duration:.2f} seconds")
            print(f"    Avg rate: {self.messages_received / duration:.2f} msgs/sec")
        
    def run(self):
        """Run the receiver loop"""
        self.setup()
        self.running = True
        self.start_time = time.time()
        
        try:
            while self.running:
                try:
                    # Receive data (up to 8KB buffer)
                    data, addr = self.socket.recvfrom(8192)
                    self.messages_received += 1
                    
                    # Try to decode and parse the JSON data
                    try:
                        log_data = json.loads(data.decode('utf-8'))
                        self.display_log(log_data, addr)
                    except json.JSONDecodeError:
                        # If not valid JSON, display raw data
                        print(f"{Fore.RED}[!] Received non-JSON data: {data.decode('utf-8', errors='replace')}{Style.RESET_ALL}")
                    except UnicodeDecodeError:
                        # If not valid UTF-8, display hex data
                        print(f"{Fore.RED}[!] Received binary data from {addr[0]}:{addr[1]}{Style.RESET_ALL}")
                
                except socket.timeout:
                    # This is just to check the running flag periodically
                    continue
                except Exception as e:
                    print(f"{Fore.RED}[!] Error receiving data: {e}{Style.RESET_ALL}")
        
        finally:
            # Clean up
            if self.socket:
                self.socket.close()
            if self.log_fd:
                self.log_fd.close()
            self.print_stats()
            
    def display_log(self, log_data, addr):
        """Display a log message and save it if needed"""
        # Extract log fields with defaults
        timestamp = log_data.get('timestamp', datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        level = log_data.get('level', 'INFO')
        function = log_data.get('function', 'unknown')
        message = log_data.get('message', 'No message')
        
        # Get color for log level
        color = LOG_COLORS.get(level, Fore.WHITE)
        
        # Format the log line for display
        formatted_log = f"{color}{timestamp} - {level:<8} - [{function}] - {message}{Style.RESET_ALL}"
        print(formatted_log)
        
        # Write to log file if saving is enabled
        if self.save_logs and self.log_fd:
            # Write plain text (no color codes) to the file
            log_line = f"{timestamp} - {level} - [{function}] - {message} (from {addr[0]})\n"
            self.log_fd.write(log_line)
            self.log_fd.flush()

def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='UDP Log Receiver for Drone')
    parser.add_argument('--port', type=int, default=9999, help='UDP port to listen on')
    parser.add_argument('--save', action='store_true', help='Save logs to file')
    parser.add_argument('--logfile', type=str, help='Log file name')
    return parser.parse_args()

if __name__ == "__main__":
    args = parse_args()
    receiver = UDPLogReceiver(
        port=args.port, 
        save_logs=args.save, 
        log_file=args.logfile
    )
    receiver.run()