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

        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.yaw = 0.0

        self.get_logger().info("Setting up publishers and subscribers.")

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10, callback_group=self.cb_group)
        
        self.get_logger().info("Creating action server.")

        self.move_to_pos_server_ = ActionServer(
            self,
            MoveToPos,
            "move_to_pos",
            execute_callback=self.execute_callback,
            callback_group=self.cb_group
        )

        self.linear_speed = 1.0
        self.angular_speed = 1.0
        self.get_logger().info("Server node ready.")

    def odom_callback(self, msg: Odometry):
        self.current_linear_x = msg.pose.pose.position.x
        self.current_linear_y = msg.pose.pose.position.y
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        self.roll, self.pitch, self.yaw = euler_from_quaternion([x, y, z, w])

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Received goal: ({goal_handle.request.x}, {goal_handle.request.y})")
        feedback = MoveToPos.Feedback()
        result = MoveToPos.Result()

        target_x = goal_handle.request.x
        target_y = goal_handle.request.y

        goal_active = True

        self.get_logger().info("Initialized per-goal variables.")

        def control_loop():
            nonlocal goal_active
            try:
                if not goal_active:
                    return

                if goal_handle.is_cancel_requested:
                    self.get_logger().info("Goal canceled by client.")
                    self.cmd_vel_pub.publish(Twist())
                    goal_handle.canceled()
                    goal_active = False
                    timer.cancel()
                    return

                delta_x = target_x - self.current_linear_x
                delta_y = target_y - self.current_linear_y
                distance = math.hypot(delta_x, delta_y)

                feedback.x = self.current_linear_x
                feedback.y = self.current_linear_y
                feedback.distance_from_goal = distance
                goal_handle.publish_feedback(feedback)

                twist = Twist()
                target_theta = math.atan2(delta_y, delta_x)
                delta_theta = target_theta - self.yaw
                delta_theta = math.atan2(math.sin(delta_theta), math.cos(delta_theta))

                if distance < 0.05:
                    self.cmd_vel_pub.publish(Twist())
                    result.success = True
                    self.get_logger().info("Goal reached! Returning result.")
                    goal_handle.succeed()
                    goal_active = False
                    timer.cancel()
                    return

                if abs(delta_theta) > 0.1:
                    twist.angular.z = self.angular_speed
                    twist.linear.x = 0.0

                else:
                    twist.angular.z = 0.0
                    twist.linear.x = self.linear_speed

                self.cmd_vel_pub.publish(twist)
            except Exception as e:
                self.get_logger().error(f"Exception in control_loop: {e}")
                goal_active = False
                try:
                    timer.cancel()
                except Exception:
                    pass

        self.get_logger().info("Creating timer for control loop.")

        timer = self.create_timer(
            0.02, control_loop, callback_group=self.cb_group)
        
        self.get_logger().info("Timer started.")

        while goal_active and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().info("Goal execution loop finished.")

        try:
            timer.cancel()
            self.get_logger().info("Timer cleanup after goal.")
        except Exception:
            self.get_logger().error("Timer cleanup after goal failed.")

        self.get_logger().info(f"Returning result: {result}")
        return result

def main(args=None):
    rclpy.init(args=args)
    node = MoveToPosServerNode()
    node.get_logger().info("Node created. Adding to executor.")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    node.get_logger().info("Executor started.")
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down.")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info("Shutdown complete.")

if __name__ == "__main__":
    main()
