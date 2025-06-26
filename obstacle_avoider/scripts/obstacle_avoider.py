#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ObstacleAvoider(Node):

    def __init__(self):
        super().__init__('obstacle_avoider')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.get_logger().info("Obstacle Avoider node started")

        self.vision=15

        self.linear_speed = 1.5
        self.angular_speed = -7.5
        self.obstacle_distance_threshold = 0.5  # meters

        self.timer = self.create_timer(0.1, self.set_velocity)
        self.obstacle_in_front = False

    def scan_callback(self,msg:LaserScan):

        ''' 
        Callback function receives a LaserScan message (`msg`) which contains ranges
        a list of distance measurements from
        representing the distance from the robot to obstacles at different angles.
        The index in anges corresponds to a float angle in radians times 100
        starting from `angle_min` incremented by `angle_increment` for each step.
        '''

        distance_ranges = msg.ranges[-15:]+msg.ranges[:15]

        # Filter out invalid ranges (NaN or inf)
        distance_ranges= [r for r in distance_ranges if r > 0.0 and r < float('inf')]

        if not distance_ranges:
            self.get_logger().warn("No valid LaserScan data in front!")
            self.obstacle_in_front = False
            return
        else:
            self.get_logger().info(f"Min distance ahead: {min(distance_ranges):.2f}")
            self.obstacle_in_front = min(distance_ranges) < self.obstacle_distance_threshold
      

        pass

    def set_velocity(self):
        twist = Twist()
        if self.obstacle_in_front:
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
        else:
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)
        






def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()