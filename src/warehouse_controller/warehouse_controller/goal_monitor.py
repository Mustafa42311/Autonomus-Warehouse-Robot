#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Float64MultiArray
from action_msgs.msg import GoalStatus, GoalStatusArray
import time


class GoalMonitor(Node):
    def __init__(self):
        super().__init__('goal_monitor')
        
        # Publisher to send goal trigger command to hardware interface
        self.goal_trigger_publisher = self.create_publisher(
            Float64MultiArray, 
            '/goal_signal_controller/commands', 
            10
        )
        
        # Subscribe to the navigate_to_pose action status
        self.status_subscriber = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.status_callback,
            10
        )
        
        # Keep track of processed goals to avoid duplicate triggers
        self.processed_goals = set()
        
        self.get_logger().info("Goal Monitor Node initialized")
        self.get_logger().info("Monitoring /navigate_to_pose/_action/status for goal completion")

    def status_callback(self, msg):
        """Callback for goal status updates"""
        try:
            for goal_status in msg.status_list:
                goal_id = goal_status.goal_info.goal_id.uuid
                status = goal_status.status
                
                # Convert UUID bytes to string for easier handling
                goal_id_str = ''.join(format(byte, '02x') for byte in goal_id)
                
                # Check if goal just succeeded and hasn't been processed
                if (status == GoalStatus.STATUS_SUCCEEDED and 
                    goal_id_str not in self.processed_goals):
                    
                    self.get_logger().info(
                        f"Navigation goal {goal_id_str[:8]}... succeeded! Sending trigger to Arduino..."
                    )
                    
                    # Send goal trigger command
                    self.send_goal_trigger()
                    
                    # Mark this goal as processed
                    self.processed_goals.add(goal_id_str)
                
                # Clean up old processed goals to prevent memory buildup
                elif status in [GoalStatus.STATUS_SUCCEEDED, 
                              GoalStatus.STATUS_ABORTED, 
                              GoalStatus.STATUS_CANCELED]:
                    
                    # Remove from processed set if it's in a terminal state
                    self.processed_goals.discard(goal_id_str)
                    
        except Exception as e:
            self.get_logger().error(f"Error processing goal status: {str(e)}")

    def send_goal_trigger(self):
        """Send trigger signal to hardware interface"""
        try:
            msg = Float64MultiArray()
            msg.data = [1.0]  # Trigger the goal signal
            
            self.goal_trigger_publisher.publish(msg)
            
            self.get_logger().info("Goal trigger signal (1.0) sent to hardware interface")
            
            # Also log to verify the topic is being published
            self.get_logger().info("Published to: /goal_signal_controller/commands")
            
        except Exception as e:
            self.get_logger().error(f"Failed to send goal trigger: {str(e)}")

    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        self.get_logger().info("Shutting down Goal Monitor Node")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        goal_monitor = GoalMonitor()
        rclpy.spin(goal_monitor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in goal monitor: {e}")
    finally:
        try:
            goal_monitor.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()