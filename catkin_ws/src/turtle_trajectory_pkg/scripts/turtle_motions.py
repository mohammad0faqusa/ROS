#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from math import pi

cmd_vel_pub = None  # This will be initialized in the main script

def move_forward(distance):
    """Move the turtle forward by a given distance."""
    velocity_msg = Twist()
    velocity_msg.linear.x = 1.0  # Adjust speed as needed
    rate = rospy.Rate(10)  # 10 Hz

    for _ in range(int(distance * 10)):  # Simulate time for the distance
        if rospy.get_param('/stop_operation', False):
            rospy.loginfo("Operation stopped by user.")
            return
        cmd_vel_pub.publish(velocity_msg)
        rate.sleep()

    velocity_msg.linear.x = 0.0
    cmd_vel_pub.publish(velocity_msg)

def rotate_angle(angle_deg):
    """Rotate the turtle by a given angle in degrees."""
    velocity_msg = Twist()
    velocity_msg.angular.z = pi / 2  # 90 degrees/sec
    rate = rospy.Rate(10)

    duration = abs(angle_deg) / 90.0  # Duration in seconds
    for _ in range(int(duration * 10)):
        if rospy.get_param('/stop_operation', False):
            rospy.loginfo("Operation stopped by user.")
            return
        cmd_vel_pub.publish(velocity_msg)
        rate.sleep()

    velocity_msg.angular.z = 0.0
    cmd_vel_pub.publish(velocity_msg)

def move_square(side_length):
    """Move in a square trajectory."""
    for _ in range(4):
        move_forward(side_length)
        rotate_angle(90)

def move_triangle(side_length):
    """Move in a triangular trajectory."""
    for _ in range(3):
        move_forward(side_length)
        rotate_angle(120)

def move_circular(radius):
    """Move in a circular trajectory."""
    velocity_msg = Twist()
    velocity_msg.linear.x = 1.0
    velocity_msg.angular.z = velocity_msg.linear.x / radius
    rate = rospy.Rate(10)

    for _ in range(int(2 * pi * radius * 10)):
        if rospy.get_param('/stop_operation', False):
            rospy.loginfo("Operation stopped by user.")
            return
        cmd_vel_pub.publish(velocity_msg)
        rate.sleep()

    velocity_msg.linear.x = 0.0
    velocity_msg.angular.z = 0.0
    cmd_vel_pub.publish(velocity_msg)

# Add similar functions for spiral, point-to-point, and zigzag.
