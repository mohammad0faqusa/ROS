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
                # Placeholder for move_triangle()
            elif choice == 3:
                rospy.loginfo("Circular trajectory selected")
                # Placeholder for move_circular()
            elif choice == 4:
                rospy.loginfo("Spiral trajectory selected")
                # Placeholder for move_spiral()
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
