"""
MAVROS SETUP: https://github.com/mavlink/mavros/blob/ros2/mavros/README.md

Use test script for trying to intercept waypoints to see if you can get the current waypoints 
listed after sending the pixhawk waypoints from mission planner
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import WaypointPull
from mavros_msgs.msg import Waypoint

class GetWaypointsNode(Node):
    def __init__(self):
        super().__init__('get_waypoints_node')
        self.client = self.create_client(WaypointPull, '/mavros/mission/pull')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.timer = self.create_timer(5.0, self.timer_callback)

    def timer_callback(self):
        request = WaypointPull.Request()
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Number of waypoints: {response.wp_received}")
                for wp in response.waypoints:
                    self.get_logger().info(f"Waypoint: {wp}")
            else:
                self.get_logger().info("Failed to pull waypoints")
        except Exception as e:
            self.get_logger().info(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    get_waypoints_node = GetWaypointsNode()
    rclpy.spin(get_waypoints_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
