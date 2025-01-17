#!/usr/bin/env python3
import serial
import time

class CyglidarConfig:
    def __init__(self, port='/dev/ttyUSB0'):
        self.port = port
        self.serial_port = None
        
        # Command sequences
        self.STOP_CMD = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x01, 0x00, 0x77]  # Stop data stream
        # Setting sensitivity to minimum (0x01) to test if configuration works
        self.SENSITIVITY = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x11, 0x01, 0x77]  # Extreme low sensitivity
        self.START_CMD = [0x5A, 0x77, 0xFF, 0x02, 0x00, 0x02, 0x00, 0x77]  # Start scanning
        
    def configure_sensitivity(self):
        """Configure device sensitivity with verification."""
        try:
            print("\nAttempting extreme low sensitivity configuration...")
            
            # Multiple attempts with verification
            for i in range(5):  # Increased to 5 attempts
                print(f"\nConfiguration attempt {i+1}/5")
                
                # 1. Stop current scanning
                print("\nStopping scan...")
                stop_response = self._send_command(self.STOP_CMD, "stop")
                time.sleep(0.5)  # Increased delay
                
                # 2. Send sensitivity configuration
                print("\nSetting sensitivity to minimum (0x01)...")
                sens_response = self._send_command(self.SENSITIVITY, "sensitivity")
                time.sleep(0.5)  # Increased delay
                
                # Print detailed response information
                if sens_response:
                    print(f"Detailed sensitivity response: {[hex(x) for x in sens_response]}")
                
                # 3. Restart scanning
                print("\nRestarting scan...")
                start_response = self._send_command(self.START_CMD, "start")
                time.sleep(0.5)  # Increased delay
                
                if sens_response:
                    print("Configuration attempt completed with response")
                else:
                    print("No response received for sensitivity configuration")
            
            return True
            
        except Exception as e:
            print(f"Configuration error: {str(e)}")
            return False