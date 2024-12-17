#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import threading
                
def main(args=None):

    rclpy.init(args=args)
    node = Node("pub_test_node")
    
    pub_test = node.create_publisher(Int64, "test_topic", 1)
    
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(20, node.get_clock())
    i = 0
    while rclpy.ok():
        m = Int64()
        log = Int64()
        m.data = i
        pub_test.publish(m)
        print("Sending: " + str(i))
        i += 1

        rate.sleep()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()