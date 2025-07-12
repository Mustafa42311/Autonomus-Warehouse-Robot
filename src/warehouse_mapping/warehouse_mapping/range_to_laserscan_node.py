#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math

class LaserScanMergerNode(Node):
    def __init__(self):
        super().__init__('laserscan_merger_node')

        # Parameters
        self.declare_parameter('scan_topic', '/sonar/scan')
        self.declare_parameter('scan_frame_id', 'base_link')
        self.declare_parameter('num_scan_points', 360)
        self.declare_parameter('scan_frequency', 10.0)
        self.declare_parameter('range_min', 0.03)
        self.declare_parameter('range_max', 3.0)

        # Get parameters
        self.scan_topic = self.get_parameter('scan_topic').value
        self.scan_frame_id = self.get_parameter('scan_frame_id').value
        self.num_scan_points = self.get_parameter('num_scan_points').value
        self.scan_frequency = self.get_parameter('scan_frequency').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value

        # A dictionary to hold the latest reading from each sensor
        self.sensor_readings = {}

        # The angles of your sensors relative to the robot's forward direction (X-axis)
        self.sensor_angles = {
            'sonar/front_left': -0.34753,
            'sonar/front_right': 0.3506,
        }

        # QoS Profile for LaserScan data
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher for the final, combined LaserScan message
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)

        # Subscribers for each single-beam LaserScan topic
        self.create_subscription(
            LaserScan, 'sonar/front_left', lambda msg: self.laserscan_callback(msg, 'sonar/front_left'), qos_profile)
        self.create_subscription(
            LaserScan, 'sonar/front_right', lambda msg: self.laserscan_callback(msg, 'sonar/front_right'), qos_profile)

        # Timer to publish the combined LaserScan at a fixed rate
        self.timer = self.create_timer(1.0 / self.scan_frequency, self.publish_scan)

        self.get_logger().info('LaserScan Merger for 2 sensors started.')

    def laserscan_callback(self, msg, sensor_id):
        """Store the latest reading from a sensor. The reading is in the first element of the ranges array."""
        if msg.ranges:
            self.sensor_readings[sensor_id] = msg.ranges[0]

    def publish_scan(self):
        """Construct and publish the combined LaserScan message."""
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.scan_frame_id

        scan_msg.angle_min = -math.pi
        scan_msg.angle_max = math.pi
        scan_msg.angle_increment = (2 * math.pi) / self.num_scan_points
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 1.0 / self.scan_frequency
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max

        # Initialize ranges with 'inf'
        scan_msg.ranges = [float('inf')] * self.num_scan_points

        # Fill in the scan data from our sensor readings
        for sensor_id, reading in self.sensor_readings.items():
            if sensor_id in self.sensor_angles:
                angle = self.sensor_angles[sensor_id]
                index = int((angle - scan_msg.angle_min) / scan_msg.angle_increment)
                
                # Unlike a real sonar, our simulated beam is very thin.
                # We can still "paint" a small arc to make it more visible in RViz.
                arc_width_points = int((0.1 / scan_msg.angle_increment) / 2) # Paint a ~6 degree arc

                for i in range(index - arc_width_points, index + arc_width_points + 1):
                    if 0 <= i < self.num_scan_points:
                        scan_msg.ranges[i] = reading

        self.scan_pub.publish(scan_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanMergerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()