#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # CORRECTED TOPIC NAME
        # The topic name now matches the output of `ros2 topic list`
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera_node/image_raw/compressed', # Corrected topic
            self.listener_callback,
            10)
        self.get_logger().info('Image subscriber node has been started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received compressed image with format: {msg.format}')

# --- MAKE SURE THE FOLLOWING CODE IS NOT INDENTED ---
def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
