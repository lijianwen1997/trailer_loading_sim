#!/usr/bin/env python3
import cv2
import numpy as np
# ROS2 packages
import rclpy
import matplotlib.pyplot as plt
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from linc_msgs.msg import TorqeedoCmdThrustStamped, TorqeedoCmdPosStamped, TorqeedoCmdThrust, TorqeedoCmdPos, TorqeedoCmdStamped
from nav_msgs.msg import Path, Odometry
from scipy.spatial.transform import Rotation

from sensor_msgs.msg import Joy
import math

class ImagePublisher(Node):

    def __init__(self):
        super().__init__('motor_stoper')
        # Get global parameters

        # Get topic names from config file
        left_thruster_topic_name = '/wamv/thrusters/left/thrust'

        right_thruster_topic_name = '/wamv/thrusters/right/thrust'

        # Defin subscriptions and publishers

        self.left_thruster_publisher = self.create_publisher(Float64, left_thruster_topic_name, 10)
        self.right_thruster_publisher = self.create_publisher(Float64, right_thruster_topic_name, 10)
        self.left_thruster_subscriber = self.create_subscription(Float64, left_thruster_topic_name,
                                                                 self.left_motor_callback, 10)
        self.right_thruster_subscriber = self.create_subscription(Float64,
                                                                  right_thruster_topic_name,
                                                                  self.right_motor_callback, 10)

        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.state_id = 0
        self.cnt = 0
        self.cnt_joy = 0

    def joy_callback(self, msg):
        self.cnt_joy +=1


    def left_motor_callback(self, msg):
        # Convert ROS2 Image message to OpenCV image
        self.cnt +=1
    def right_motor_callback(self, msg):
        # Convert ROS2 Image message to OpenCV image
        self.cnt +=1

    def timer_callback(self):
        if self.cnt==0 or self.cnt_joy==0:

            # Set motor speed to zero and publish
            left_thrust_msg = Float64()
            right_thrust_msg = Float64()
            left_thrust_msg.data = 0.0
            right_thrust_msg.data = 0.0

            # Publish to the motors
            self.left_thruster_publisher.publish(left_thrust_msg)
            self.right_thruster_publisher.publish(right_thrust_msg)
        else:
            self.cnt = 0



def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        image_publisher.get_logger().info("Keyboard interrupt received.")

    finally:


        rclpy.shutdown()

if __name__ == '__main__':
    main()