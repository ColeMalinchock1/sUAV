#!/usr/bin/env python3
import serial
import time

class CyglidarConfig:
    def __init__(self, port='/dev/ttyUSB0'):
        self.port = port
        self.serial_port = None
        
        # Command sequences
        self.STOP_CMD = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x01, 0x00, 0x77]  # Stop data stream
        self.SENSITIVITY = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x11, 0x64, 0x77]  # Sensitivity setting
        self.START_CMD = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x02, 0x00, 0x77]  # Start scanning
        
    def connect(self):
        """Connect to device at 115200 baud."""
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
                
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=115200,  # Fixed at 115200
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
                write_timeout=0.1
            )
            
            # Clear buffers
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            print(f"Connected to Cyglidar on {self.port} at 115200 baud")
            return True
            
        except Exception as e:
            print(f"Connection error: {str(e)}")
            return False
            
    def _send_command(self, cmd, desc="command", retries=3):
        """Send command and get response."""
        if not self.serial_port or not self.serial_port.is_open:
            return None
            
        for attempt in range(retries):
            try:
                # Clear buffers
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
                
                print(f"\nSending {desc} (Attempt {attempt + 1}/{retries})")
                print(f"Command: {[hex(x) for x in cmd]}")
                
                # Send command bytes with small delays
                for byte in cmd:
                    self.serial_port.write(bytes([byte]))
                    time.sleep(0.002)  # 2ms between bytes
                    
                self.serial_port.flush()
                time.sleep(0.1)
                
                # Read response
                response = self.serial_port.read(32)
                print(f"Response ({len(response)} bytes): {[hex(x) for x in response]}")
                
                if response and any(b != 0 for b in response):
                    return response
                    
            except Exception as e:
                print(f"Command error: {str(e)}")
                time.sleep(0.2)
                
        return None
        
    def configure_sensitivity(self):
        """Configure device sensitivity."""
        try:
            # 1. Stop current scanning
            print("\nStopping scan...")
            self._send_command(self.STOP_CMD, "stop")
            time.sleep(0.2)
            
            # 2. Send sensitivity configuration
            print("\nSetting sensitivity...")
            self._send_command(self.SENSITIVITY, "sensitivity")
            time.sleep(0.2)
            
            # 3. Restart scanning
            print("\nRestarting scan...")
            self._send_command(self.START_CMD, "start")
            
            return True
            
        except Exception as e:
            print(f"Configuration error: {str(e)}")
            return False
        
    def disconnect(self):
        """Clean disconnect."""
        if self.serial_port and self.serial_port.is_open:
            try:
                self._send_command(self.START_CMD, "final start")
                time.sleep(0.1)
                self.serial_port.close()
                print("Disconnected from Cyglidar")
            except Exception as e:
                print(f"Disconnect error: {str(e)}")

def main():
    config = CyglidarConfig()
    try:
        if config.connect():
            config.configure_sensitivity()
    finally:
        config.disconnect()

if __name__ == "__main__":
    main()