#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import curses

class KeyboardControllerNode():
    
    def __init__(self):
        self.stdscr = curses.initscr()
        curses.noecho()  # Don't echo keystrokes
        curses.cbreak()  # React to keys instantly without requiring Enter
        self.stdscr.keypad(True)  # Enable keypad mode

        # Initialize mode and direction
        self.mode = "None"
        self.direction = "None"
          
        self.main()

    def handle_keyboard(self):
        while True:
            try:
                key = self.stdscr.getch()
                
                # Mode controls
                if key == ord('a'):
                    self.mode = "Auto"
                elif key == ord('g'):
                    self.mode = "Guided"
                elif key == ord('n'):
                    self.mode = "None"
                
                # Direction controls
                elif key == curses.KEY_LEFT:
                    self.direction = "Left"
                elif key == curses.KEY_RIGHT:
                    self.direction = "Right"
                elif key == curses.KEY_UP:
                    self.direction = "Forward"
                elif key == curses.KEY_DOWN:
                    self.direction = "Backward"
                elif key == ord('o'):
                    self.direction = "Hold"
                
                # Reset direction when key is released
                else:
                    self.direction = "None"

            except Exception as e:
                self.cleanup()
                break

    def cleanup(self):
        # Restore terminal settings
        curses.nocbreak()
        self.stdscr.keypad(False)
        curses.echo()
        curses.endwin()

    def main(self, args=None):
        rclpy.init(args=args)
        node = Node("keyboard_controller_node")

        # Publishers for mode and direction
        mode_pub = node.create_publisher(String, "/controller/mode", 1)
        direction_pub = node.create_publisher(String, "/controller/direction", 1)

        # Start ROS2 spin in a separate thread
        ros_thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
        ros_thread.start()

        # Start keyboard handling in a separate thread
        keyboard_thread = threading.Thread(target=self.handle_keyboard, daemon=True)
        keyboard_thread.start()

        rate = node.create_rate(20, node.get_clock())
            
        while rclpy.ok():
            try:
                # Publish messages
                mode_msg = String()
                mode_msg.data = self.mode

                direction_msg = String()
                direction_msg.data = self.direction

                mode_pub.publish(mode_msg)
                direction_pub.publish(direction_msg)
                
                # Update display
                self.stdscr.clear()
                self.stdscr.addstr(1, 25, 'Keyboard Controller Node')
                self.stdscr.addstr(2, 25, f'Mode: {self.mode}')
                self.stdscr.addstr(3, 25, f'Direction: {self.direction}')
                self.stdscr.addstr(5, 25, 'Controls:')
                self.stdscr.addstr(6, 25, 'A: Auto  G: Guided  N: None')
                self.stdscr.addstr(7, 25, 'Arrow keys: Direction control')
                self.stdscr.refresh()
                
                rate.sleep()
                
            except KeyboardInterrupt:
                self.cleanup()
                break
        
        rclpy.shutdown()

def start():
    KeyboardControllerNode()

if __name__ == '__main__':
    start()