
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

        self.linear_speed = 0.2
        self.angular_speed = -0.5
        self.obstacle_distance_threshold = 0.5  # meters

        self.timer = self.create_timer(0.1, self.control_loop)
        self.obstacle_in_front = False

    def scan_callback(self,msg):
        pass

    def set_velocity(self):
        twist = Twist()
        if self.obstacle_in_front:
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
        else:
            twist.linear.x = self.forward_speed
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