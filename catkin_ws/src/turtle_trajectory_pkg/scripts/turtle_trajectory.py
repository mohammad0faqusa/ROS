#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# Globals
cmd_vel_pub = None
current_pose = None

def pose_callback(pose):
    """Callback function to update the current pose of the turtle."""
    global current_pose
    current_pose = pose

def move_forward(distance, speed=1.0):
    """Move the turtle forward a specified distance."""
    global cmd_vel_pub, current_pose
    velocity_msg = Twist()
    rate = rospy.Rate(10)  # 10 Hz

    start_x, start_y = current_pose.x, current_pose.y
    velocity_msg.linear.x = speed

    while not rospy.is_shutdown():
        distance_moved = math.sqrt((current_pose.x - start_x) ** 2 +
                                   (current_pose.y - start_y) ** 2)
        if distance_moved >= distance:
            break
        cmd_vel_pub.publish(velocity_msg)
        rate.sleep()

    velocity_msg.linear.x = 0
    cmd_vel_pub.publish(velocity_msg)

def rotate_angle(angle_deg, angular_speed=30.0):
    """Rotate the turtle by a specified angle."""
    global cmd_vel_pub, current_pose
    velocity_msg = Twist()
    rate = rospy.Rate(10)  # 10 Hz

    angle_rad = math.radians(angle_deg)
    start_theta = current_pose.theta
    velocity_msg.angular.z = math.radians(angular_speed) if angle_rad > 0 else -math.radians(angular_speed)

    while not rospy.is_shutdown():
        current_angle = abs(current_pose.theta - start_theta)
        if current_angle >= abs(angle_rad):
            break
        cmd_vel_pub.publish(velocity_msg)
        rate.sleep()

    velocity_msg.angular.z = 0
    cmd_vel_pub.publish(velocity_msg)

def move_square(edge_length):
    """Move the turtle in a square trajectory."""
    for _ in range(4):  # A square has four sides
        move_forward(edge_length)
        rotate_angle(90)  # Rotate 90 degrees for each corner
    rospy.loginfo("Completed square trajectory")

def move_triangle(side_length):
    """Move the turtle in a triangle trajectory."""
    for _ in range(3):  # A triangle has three sides
        move_forward(side_length)
        rotate_angle(120)  # Rotate 120 degrees for each corner
    rospy.loginfo("Completed triangle trajectory")

def move_circular(radius, linear_speed=1.0):
    """Move the turtle in a circular trajectory."""
    global cmd_vel_pub
    velocity_msg = Twist()

    angular_speed = linear_speed / radius  # Calculate angular speed
    velocity_msg.linear.x = linear_speed
    velocity_msg.angular.z = angular_speed

    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.Time.now().to_sec()

    # Run the loop for one complete circle
    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()
        elapsed_time = current_time - start_time
        if elapsed_time >= (2 * math.pi * radius / linear_speed):  # Time to complete a circle
            break
        cmd_vel_pub.publish(velocity_msg)
        rate.sleep()

    # Stop the turtle
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    cmd_vel_pub.publish(velocity_msg)

    rospy.loginfo("Completed circular trajectory")

def move_spiral(dr_increment=0.1, linear_speed=1.0):
    """
    Move the turtle in a spiral trajectory.
    :param dr_increment: Increment of radius per loop iteration.
    :param linear_speed: Base linear speed of the turtle.
    """
    global cmd_vel_pub
    velocity_msg = Twist()

    rate = rospy.Rate(10)  # 10 Hz
    radius = 0.5  # Start with a small radius

    start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()
        elapsed_time = current_time - start_time

        if elapsed_time > 20:  # Limit the spiral duration (e.g., 20 seconds)
            break

        angular_speed = linear_speed / radius
        velocity_msg.linear.x = linear_speed
        velocity_msg.angular.z = angular_speed

        cmd_vel_pub.publish(velocity_msg)
        radius += dr_increment  # Increment radius
        rate.sleep()

    # Stop the turtle
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    cmd_vel_pub.publish(velocity_msg)

    rospy.loginfo("Completed spiral trajectory")


def main_menu():
    """Display the main menu and handle user input."""
    while not rospy.is_shutdown():
        print("\nSelect a motion trajectory:")
        print("1. Square")
        print("2. Triangle")
        print("3. Circular")
        print("4. Spiral")
        print("5. Point to Point")
        print("6. Zigzag")
        print("7. Exit")

        try:
            choice = int(input("Enter your choice (1-7): "))
            if choice == 1:
                rospy.loginfo("Square trajectory selected")
                rospy.loginfo("Square trajectory selected")
                edge_length = float(input("Enter the side length of the square: "))
                move_square(edge_length)
            
            elif choice == 2:
                rospy.loginfo("Triangle trajectory selected")
                side_length = float(input("Enter the side length of the triangle: "))
                move_triangle(side_length)

            elif choice == 3:
                rospy.loginfo("Circular trajectory selected")
                radius = float(input("Enter the radius of the circle: "))
                move_circular(radius)

            elif choice == 4:
                rospy.loginfo("Spiral trajectory selected")
                dr_increment = float(input("Enter the radius increment per step: "))
                move_spiral(dr_increment)
                
            elif choice == 5:
                rospy.loginfo("Point-to-Point trajectory selected")
                # Placeholder for move_point_to_point()
            elif choice == 6:
                rospy.loginfo("Zigzag trajectory selected")
                # Placeholder for move_zigzag()
            elif choice == 7:
                rospy.loginfo("Exiting Turtle Trajectory Node.")
                break
            else:
                print("Invalid choice. Please enter a number between 1 and 7.")
        except ValueError:
            print("Invalid input. Please enter a valid number.")

def main():
    global cmd_vel_pub

    rospy.init_node('turtle_trajectory', anonymous=True)
    rospy.loginfo("Turtle Trajectory Node Initialized")

    # Initialize the publisher for cmd_vel
    cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    # Initialize the subscriber for pose updates
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

    rospy.sleep(1)  # Wait for connections to establish
    main_menu()


if __name__ == '__main__':
    main()
