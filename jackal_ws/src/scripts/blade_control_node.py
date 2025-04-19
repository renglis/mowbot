#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float64MultiArray

class BladeControlNode(Node):
    def __init__(self):
        super().__init__('blade_control_node')

        # Subscribe to the ultrasonic sensor topic
        self.sub = self.create_subscription(
            Range,
            '/ultrasonic',
            self.range_callback,
            10
        )

        # Publish to the blade velocity controller
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/blade_velocity_controller/commands',
            10
        )

        # Threshold and velocity settings
        self.threshold = 0.04          # meters
        self.spin_velocity = 50.0      # rad/s when object detected
        self.stop_velocity = 0.0       # rad/s when clear

    def range_callback(self, msg):
        velocity = self.spin_velocity if msg.range < self.threshold else self.stop_velocity
        self.pub.publish(Float64MultiArray(data=[velocity]))

def main(args=None):
    rclpy.init(args=args)
    node = BladeControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
