#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircleNode(Node):

    def __init__(self):
        super().__init__("draw_circle")
        self.cmd_vel_pub_=self.create_publisher(Twist,"/cmd_vel",10)
        self.timer=self.create_timer(0.5,self.send_velocity_command)
        self.get_logger().info("Draw circle node has been started")

        # Declare parameters with default values
        self.declare_parameter("linear_x", 2.0)
        self.declare_parameter("linear_y", 0.0)
        self.declare_parameter("angular_z", 1.0)



    def send_velocity_command(self):

        #in case we wish to publish new velocity components
        linear_x = self.get_parameter("linear_x").get_parameter_value().double_value
        linear_y = self.get_parameter("linear_y").get_parameter_value().double_value
        angular_z = self.get_parameter("angular_z").get_parameter_value().double_value

        msg=Twist()
        msg.linear.x=linear_x
        msg.linear.y=linear_y
        msg.angular.z=angular_z
        self.cmd_vel_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node=DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()