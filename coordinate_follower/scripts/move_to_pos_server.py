#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from coordinate_follower.action import MoveToPos
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

class MoveToPosServerNode(Node):
    def __init__(self):
        super().__init__("move_to_pos")
        self.get_logger().info("MoveToPosServerNode initialized.")
        self.cb_group = ReentrantCallbackGroup()

        # Current pose in the odom frame
        self.current_x = 0.0
        self.current_y = 0.0
        self.yaw = 0.0

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10, callback_group=self.cb_group)
        
        self.move_to_pos_server_ = ActionServer(
            self,
            MoveToPos,
            "move_to_pos",
            execute_callback=self.execute_callback,
            callback_group=self.cb_group
        )

        # Controller parameters
        self.linear_speed = 1.0  # Reduced speed for better control
        self.angular_speed = 1.0
        self.linear_tolerance = 0.05  # 5cm tolerance
        self.angular_tolerance = 0.05  # ~3 degrees

    def odom_callback(self, msg: Odometry):
        # Odometry position is relative to starting position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Executing goal: ({goal_handle.request.x}, {goal_handle.request.y})")
        feedback = MoveToPos.Feedback()
        result = MoveToPos.Result()

        target_x = goal_handle.request.x
        target_y = goal_handle.request.y

        # Control loop parameters
        control_rate = self.create_rate(10)  # 10Hz
        timer = None

        try:
            while rclpy.ok():
                # Check for cancelation
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info("Goal canceled")
                    result.success = False
                    return result

                # Calculate errors
                dx = target_x - self.current_x
                dy = target_y - self.current_y
                distance = math.hypot(dx, dy)
                target_yaw = math.atan2(dy, dx)
                yaw_error = target_yaw - self.yaw
                
                # Normalize angle to [-pi, pi]
                yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

                # Update feedback
                feedback.x = self.current_x
                feedback.y = self.current_y
                feedback.distance_from_goal = distance
                goal_handle.publish_feedback(feedback)

                # Check if goal is reached
                if distance < self.linear_tolerance:
                    self.get_logger().info("Goal reached")
                    self.cmd_vel_pub.publish(Twist())  # Stop the robot
                    result.success = True
                    goal_handle.succeed()
                    return result

                # Create and send command
                twist = Twist()
                
                # First align with target
                if abs(yaw_error) > self.angular_tolerance:
                    twist.angular.z = self.angular_speed * (1.0 if yaw_error > 0 else -1.0)
                else:
                    # Then move forward
                    twist.linear.x = self.linear_speed if distance>0.5 else 0.5*self.linear_speed # Reduce speed as we approach target

                self.cmd_vel_pub.publish(twist)
                control_rate.sleep()

        except Exception as e:
            self.get_logger().error(f"Exception in execute_callback: {e}")
            result.success = False
            goal_handle.abort()
            return result
        finally:
            # Ensure we stop the robot
            self.cmd_vel_pub.publish(Twist())
            if timer is not None:
                timer.cancel()

        result.success = False
        return result

def main(args=None):
    rclpy.init(args=args)
    node = MoveToPosServerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
