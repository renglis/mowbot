#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Range
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudToRange(Node):
    def __init__(self):
        super().__init__('pc2_to_range')
        self.sub = self.create_subscription(
            PointCloud2, '/gazebo_ros_range/out', self.cb,  10)
        self.pub = self.create_publisher(Range, '/ultrasonic', 10)

        # set these once
        self.range_msg = Range()
        self.range_msg.radiation_type = Range.ULTRASOUND
        self.range_msg.field_of_view = 0.05
        self.range_msg.min_range = 0.02
        self.range_msg.max_range = 4.0

    def cb(self, pc2_msg):
        pts = list(pc2.read_points(pc2_msg, field_names=('x',), skip_nans=True))
        if not pts:
            return
        self.range_msg.header = pc2_msg.header
        self.range_msg.range = float(pts[0][0])
        self.pub.publish(self.range_msg)

def main():
    rclpy.init()
    n = PointCloudToRange()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__=='__main__':
    main()
