#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import threading

global test_data
test_data = 0

def callback(data):
    global test_data
    test_data = data.data
                
def main(args=None):
    global test_data

    rclpy.init(args=args)
    node = Node("sub_test_node")
    node.create_subscription(Int64, "test_topic", callback, 1)

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(20, node.get_clock())

    while rclpy.ok():
        print("Received data: " + str(test_data))
        rate.sleep()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()