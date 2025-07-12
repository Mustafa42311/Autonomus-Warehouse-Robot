# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from rclpy.constants import S_TO_NS
# from rclpy.time import Time
# from std_msgs.msg import Float64MultiArray
# from geometry_msgs.msg import TwistStamped
# from sensor_msgs.msg import JointState
# from nav_msgs.msg import Odometry
# import numpy as np
# from tf2_ros import TransformBroadcaster
# from geometry_msgs.msg import TransformStamped
# import math
# from tf_transformations import quaternion_from_euler


# class RailSimpleController(Node):

#     def __init__(self):
#         super().__init__("rail_simple_controller")
        
#         # Rail-specific parameters
#         self.declare_parameter("rail_joint_name", "rail_plate_joint")
#         self.declare_parameter("rail_velocity_limit", 1.0)
#         self.declare_parameter("rail_position_limit_lower", 0.0)
#         self.declare_parameter("rail_position_limit_upper", 0.3)
        
#         self.rail_joint_name_ = self.get_parameter("rail_joint_name").get_parameter_value().string_value
#         self.rail_velocity_limit_ = self.get_parameter("rail_velocity_limit").get_parameter_value().double_value
#         self.rail_position_limit_lower_ = self.get_parameter("rail_position_limit_lower").get_parameter_value().double_value
#         self.rail_position_limit_upper_ = self.get_parameter("rail_position_limit_upper").get_parameter_value().double_value

#         self.get_logger().info(f"Controlling rail joint: {self.rail_joint_name_}")
#         self.get_logger().info(f"Rail velocity limit: {self.rail_velocity_limit_}")
#         self.get_logger().info(f"Rail position limits: [{self.rail_position_limit_lower_}, {self.rail_position_limit_upper_}]")

#         # State variables
#         self.rail_prev_pos_ = 0.0
#         self.rail_position_ = 0.0
#         self.rail_velocity_ = 0.0
        
#         # Publishers and Subscribers
#         self.rail_cmd_pub_ = self.create_publisher(
#             Float64MultiArray, 
#             "rail_position_controller/commands", 
#             10
#         )
        
#         self.vel_sub_ = self.create_subscription(
#             TwistStamped, 
#             "rail_controller/cmd_vel", 
#             self.velCallback, 
#             10
#         )
        
#         self.joint_sub_ = self.create_subscription(
#             JointState,
#             "joint_states", 
#             self.jointCallback, 
#             10
#         )
        
#         self.odom_pub_ = self.create_publisher(
#             Odometry, 
#             "rail_controller/odom", 
#             10
#         )

#         # Fill the Odometry message with invariant parameters
#         self.odom_msg_ = Odometry()
#         self.odom_msg_.header.frame_id = "odom"
#         self.odom_msg_.child_frame_id = "rail_base_link"
#         # Fixed orientation for rail movement (no rotation)
#         self.odom_msg_.pose.pose.orientation.x = 0.0
#         self.odom_msg_.pose.pose.orientation.y = 0.0
#         self.odom_msg_.pose.pose.orientation.z = 0.0
#         self.odom_msg_.pose.pose.orientation.w = 1.0

#         # Fill the TF message
#         self.br_ = TransformBroadcaster(self)
#         self.transform_stamped_ = TransformStamped()
#         self.transform_stamped_.header.frame_id = "odom"
#         self.transform_stamped_.child_frame_id = "rail_base_link"
        
#         # Fixed orientation quaternion (no rotation)
#         q = quaternion_from_euler(0, 0, 0)
#         self.transform_stamped_.transform.rotation.x = q[0]
#         self.transform_stamped_.transform.rotation.y = q[1]
#         self.transform_stamped_.transform.rotation.z = q[2]
#         self.transform_stamped_.transform.rotation.w = q[3]

#         self.prev_time_ = self.get_clock().now()
        
#         self.get_logger().info("Rail Simple Controller initialized successfully")

#     def velCallback(self, msg):
#         """
#         Convert linear velocity command to rail position command
#         For rail movement, we use the linear.y component since the rail moves along Y-axis
#         """
#         # Extract Y-axis linear velocity (rail moves along Y-axis)
#         desired_velocity = msg.twist.linear.y
        
#         # Apply velocity limits
#         if abs(desired_velocity) > self.rail_velocity_limit_:
#             desired_velocity = math.copysign(self.rail_velocity_limit_, desired_velocity)
            
#         # Check position limits to prevent exceeding boundaries
#         predicted_position = self.rail_position_ + desired_velocity * 0.1  # Rough prediction
#         if predicted_position < self.rail_position_limit_lower_:
#             desired_velocity = max(0.0, desired_velocity)  # Only allow positive velocity
#         elif predicted_position > self.rail_position_limit_upper_:
#             desired_velocity = min(0.0, desired_velocity)  # Only allow negative velocity

#         # Create command message
#         rail_cmd_msg = Float64MultiArray()
#         rail_cmd_msg.data = [desired_velocity]

#         self.rail_cmd_pub_.publish(rail_cmd_msg)
        
#         # Log for debugging
#         self.get_logger().debug(f"Rail velocity command: {desired_velocity}")

#     def jointCallback(self, msg):
#         """
#         Process joint state feedback and publish odometry
#         """
#         # Find our rail joint in the joint state message
#         rail_joint_index = -1
#         try:
#             rail_joint_index = msg.name.index(self.rail_joint_name_)
#         except ValueError:
#             self.get_logger().warn(f"Rail joint '{self.rail_joint_name_}' not found in joint states")
#             return

#         # Get current position
#         current_position = msg.position[rail_joint_index]
        
#         # Calculate time difference
#         current_time = Time.from_msg(msg.header.stamp)
#         dt = current_time - self.prev_time_
#         dt_seconds = dt.nanoseconds / S_TO_NS
        
#         # Calculate velocity
#         if dt_seconds > 0:
#             dp = current_position - self.rail_prev_pos_
#             self.rail_velocity_ = dp / dt_seconds
#         else:
#             self.rail_velocity_ = 0.0

#         # Update state
#         self.rail_position_ = current_position
#         self.rail_prev_pos_ = current_position
#         self.prev_time_ = current_time

#         # Compose and publish the odometry message
#         self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        
#         # Position: rail moves along Y-axis, X and Z remain 0
#         self.odom_msg_.pose.pose.position.x = 0.0
#         self.odom_msg_.pose.pose.position.y = self.rail_position_
#         self.odom_msg_.pose.pose.position.z = 0.0
        
#         # Velocity: only Y-axis linear velocity
#         self.odom_msg_.twist.twist.linear.x = 0.0
#         self.odom_msg_.twist.twist.linear.y = self.rail_velocity_
#         self.odom_msg_.twist.twist.linear.z = 0.0
#         self.odom_msg_.twist.twist.angular.x = 0.0
#         self.odom_msg_.twist.twist.angular.y = 0.0
#         self.odom_msg_.twist.twist.angular.z = 0.0
        
#         self.odom_pub_.publish(self.odom_msg_)

#         # Publish TF transform
#         self.transform_stamped_.transform.translation.x = 0.0
#         self.transform_stamped_.transform.translation.y = self.rail_position_
#         self.transform_stamped_.transform.translation.z = 0.0
#         self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        
#         self.br_.sendTransform(self.transform_stamped_)
        
#         # Log current state for debugging
#         self.get_logger().debug(f"Rail position: {self.rail_position_:.3f}, velocity: {self.rail_velocity_:.3f}")


# def main():
#     rclpy.init()

#     rail_simple_controller = RailSimpleController()
#     rclpy.spin(rail_simple_controller)
    
#     rail_simple_controller.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()