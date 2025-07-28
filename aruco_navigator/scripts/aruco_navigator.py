#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge
import numpy as np
import math
from tf_transformations import euler_from_quaternion
import time
from rclpy.duration import Duration


class ArucoNavigator(Node):
    def __init__(self):
        super().__init__("aruco_navigator")

        self.tag_in_front = None    #checks if the tag is in front

        self.linear_speed = 1.0
        self.angular_speed = 1.0
        self.turn_duration=math.pi/(2*self.angular_speed)

        self.check=True  

        self.image_sub = self.create_subscription(
            Image, '/leo1/camera/image_raw', self.read_image, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.distance_threshold = 1.0

        self.twist_pub = self.create_publisher(
            Twist, '/leo1/cmd_vel', 10)
        

        self.is_turning=False

        self.angle_turned_prev=None    #angle turned in the previous rotation, to be needed for course correction
        self.yaw_start=None    #the value of yaw exactly when the bot starts to turn
        self.yaw=None         #current yaw
    
        self.yaw_ref=None     #yaw at the start of the simulation as a base reference (considering yaw along x axis to be 0.0 rad)

        self.is_aligned=None  #to check if the bot is perfectly aligned to its path 
        
        self.turn_timer=self.create_timer(0.005,self.turn_bot)

    def odom_callback(self, msg: Odometry):
        # Odometry position is relative to starting position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _,_ , self.yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        if self.yaw_ref is None:
            self.yaw_ref=self.yaw     #intialising self.yaw_ref for once

    def read_image(self, msg: Image):
      

        twist=Twist()

        bridge = CvBridge()
        self.cam_feed = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.tag_in_front, distance = identify_aruco(self.cam_feed)
        
        if self.angle_turned_prev is None:
            self.angle_turned_prev=1.571   #assume in the start that our bot performed a complete 90 degree rotation get in the initial state
        if self.yaw_start is None:
            self.yaw_start=self.yaw-self.yaw_ref     

        if self.is_aligned is None:
            self.is_aligned=True        #assuning the bot is aligned perfectly at the start
        else:
            self.is_aligned=True if abs((1.571-self.angle_turned_prev))<0.005 else False

        if (self.tag_in_front is not None or not self.is_aligned):

            if (distance>0.998 and self.is_aligned):
                if self.is_aligned:
                    twist.linear.x=self.linear_speed
                    twist.angular.z=0.0
                    self.get_logger().info(f"distance from tag={distance}, current yaw={self.yaw-self.yaw_ref} ")
                else:
                    twist.linear.x=0.0
                    twist.angular.z=(1.571-self.angle_turned_prev)*2.0 
                    self.angle_turned_prev=angle_diff(self.yaw-self.yaw_ref,self.yaw_start)
                    self.get_logger().info(f"current yaw={self.yaw-self.yaw_ref} error={1.571-self.angle_turned_prev} from start={self.angle_turned_prev}")
            
            elif self.check and distance<0.998: 
                   self.yaw_start=self.yaw-self.yaw_ref
                   self.direction= 1 if self.tag_in_front==1 else -1
                   self.is_turning=True
                        
                        
            self.twist_pub.publish(twist)
        else:
            self.get_logger().info("no tag in front")
            self.twist_pub.publish(Twist())

    def turn_bot(self):
        if self.is_turning:

            self.check=False  

            angle_turned = angle_diff(self.yaw-self.yaw_ref, self.yaw_start) * self.direction
            self.get_logger().info(f"current yaw={self.yaw-self.yaw_ref} angle turned: {angle_turned} from start={self.yaw_start}")

            if (angle_turned<1.568):

                twist=Twist()
                twist.linear.x = 0.0
                if (angle_turned<1.53):

                    twist.angular.z=self.angular_speed*self.direction 
                   
                #elif (1.55<angle_turned<1.65):
                    #twist.angular.z=self.angular_speed*self.direction*0.005
                   

                else: #1.55<angle_turned<1.565:
                    twist.angular.z=self.angular_speed*self.direction*0.4
                #else: 
                    #twist.angular.z=self.angular_speed*self.direction*(1.571-angle_turned)

                self.twist_pub.publish(twist)

            else:
                #self.get_clock().sleep_for(Duration(seconds=3.0))  #waiting for the bot to stop from its inertia
                    

                self.check=True
                self.is_turning=False
                self.angle_turned_prev=angle_turned

def identify_aruco(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        marker_length = 0.1778
        camera_matrix = np.array([
            [228.83581161499023, 0.0, 320.0],
            [0.0, 228.83581638336182, 240.0],
            [0.0, 0.0, 1.0]
        ])
        dist_coeffs = np.array([-0.279817, 0.060321, 0.00031, 0.0, 0.000487])
        _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_length, camera_matrix, dist_coeffs)

        # Find the tag (0 or 1) with the smallest absolute X and Y (most centered)
        best_index = None
        min_off_axis = None
        best_id = None
        best_distance = None

        for i, marker_id in enumerate(ids.flatten()):
            if marker_id in [0, 1]:
                x_offset = tvecs[i][0][0]  # X in camera frame (left-right)
                y_offset = tvecs[i][0][1]  # Y in camera frame (up-down)
                off_axis = abs(x_offset) + abs(y_offset)  # How far from straight ahead
                if (min_off_axis is None) or (off_axis < min_off_axis):
                    min_off_axis = off_axis
                    best_index = i
                    best_id = int(marker_id)
                    best_distance = tvecs[i][0][2]  # Z in camera frame (forward)

        if best_id is not None:
            return best_id, best_distance
        else:
            return None, None
    return None, None


def angle_diff(a, b):
    """Return smallest signed difference between two angles, result in [-pi, pi]."""
    d = a - b
    while d < -math.pi:
        d += 2 * math.pi
    while d > math.pi:
        d -= 2 * math.pi
    return d


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNavigator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
