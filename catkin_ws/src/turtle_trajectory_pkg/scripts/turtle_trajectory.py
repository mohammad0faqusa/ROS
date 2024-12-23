#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import select
import sys

# Globals
cmd_vel_pub = None
current_pose = None

# Define boundaries
X_MIN = 0.0
X_MAX = 11.0
Y_MIN = 0.0
Y_MAX = 11.0

def check_for_pause_key():
    """
    Check if the user has pressed 'p' to pause the trajectory and return to the menu.
    :return: True if 'p' is pressed, False otherwise.
    """
    # print("Press 'p' to pause and return to the menu.")
    i, _, _ = select.select([sys.stdin], [], [], 0.1)  # Wait for input for 0.1 seconds
    if i:
        key = sys.stdin.read(1)
        if key == 'p':
            rospy.loginfo("Pause key detected. Returning to menu.")
            return True
    return False

def is_within_bounds(x, y):
    """Check if the given (x, y) position is within the turtlesim boundaries."""
    return X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX

def pose_callback(pose):
    """Callback function to update the current pose of the turtle."""
    global current_pose
    current_pose = pose

def move_forward(distance, speed=1.0):
    """Move the turtle forward a specified distance, ensuring it stays within bounds."""
    global cmd_vel_pub, current_pose
    velocity_msg = Twist()
    rate = rospy.Rate(10)  # 10 Hz

    start_x, start_y = current_pose.x, current_pose.y
    velocity_msg.linear.x = speed

    while not rospy.is_shutdown():
        if check_for_pause_key():
            break  # Exit the loop if 'p' is pressed

        # Calculate the distance moved
        distance_moved = math.sqrt((current_pose.x - start_x) ** 2 +
                                   (current_pose.y - start_y) ** 2)

        # Check if the turtle is within bounds
        if not is_within_bounds(current_pose.x, current_pose.y):
            rospy.logwarn("Turtle is out of bounds! Stopping forward movement.")
            break

        # Stop if the specified distance is covered
        if distance_moved >= distance:
            break

        cmd_vel_pub.publish(velocity_msg)
        rate.sleep()

    # Stop the turtle
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

def move_spiral(dr_increment=0.1, linear_speed=1.0):
    """
    Move the turtle in a spiral trajectory.
    :param dr_increment: Increment of radius per loop iteration.
    :param linear_speed: Base linear speed of the turtle.
    """
    global cmd_vel_pub, current_pose
    velocity_msg = Twist()

    # Safety check to avoid dividing by zero if the radius gets too small.
    # Also ensure the turtle won't spiral inward past radius = 0.
    radius = max(0.5, 0.01)  # Start with a small but non-zero radius

    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        if check_for_pause_key():
            break  # Exit the loop if 'p' is pressed

        current_time = rospy.Time.now().to_sec()
        elapsed_time = current_time - start_time

        # Limit the spiral duration, e.g. 20 seconds (adjust as needed)
        if elapsed_time > 20:
            rospy.loginfo("Reached time limit for spiral; stopping.")
            break

        # Calculate angular speed at the current radius
        angular_speed = linear_speed / radius if radius != 0 else 0

        # Prepare velocity message
        velocity_msg.linear.x = linear_speed
        velocity_msg.angular.z = angular_speed

        # Check if the current position is within bounds
        if not is_within_bounds(current_pose.x, current_pose.y):
            rospy.logwarn("Turtle is out of bounds! Stopping spiral.")
            break

        # Publish velocity to move in a spiral
        cmd_vel_pub.publish(velocity_msg)

        # Increment radius after each loop to expand the spiral
        radius += dr_increment

        # Safety check: prevent radius from dropping below a tiny threshold
        if radius < 0.01:
            rospy.logwarn("Radius has become too small! Adjusting to safe minimum.")
            radius = 0.01

        rate.sleep()

    # Stop the turtle after completing or interrupting the spiral
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    cmd_vel_pub.publish(velocity_msg)

    rospy.loginfo("Completed spiral trajectory (or stopped due to bounds/time).")


def move_circular(radius, linear_speed=1.0):
    """
    Move the turtle in a circular trajectory, ensuring it stays within bounds.
    :param radius: Radius of the circle.
    :param linear_speed: Speed of the turtle.
    """
    global cmd_vel_pub, current_pose
    velocity_msg = Twist()

    # Calculate angular speed based on the radius
    angular_speed = linear_speed / radius
    velocity_msg.linear.x = linear_speed
    velocity_msg.angular.z = angular_speed

    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        if check_for_pause_key():
            break  # Exit the loop if 'p' is pressed

        # Calculate elapsed time
        current_time = rospy.Time.now().to_sec()
        elapsed_time = current_time - start_time

        # Check if one complete circle is completed
        if elapsed_time >= (2 * math.pi * radius / linear_speed):
            break

        # Check if the current position is within bounds
        if not is_within_bounds(current_pose.x, current_pose.y):
            rospy.logwarn("Turtle is out of bounds! Stopping circular trajectory.")
            break

        # Publish velocity to continue circular motion
        cmd_vel_pub.publish(velocity_msg)
        rate.sleep()

    # Stop the turtle after completing the circle or going out of bounds
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    cmd_vel_pub.publish(velocity_msg)

    rospy.loginfo("Completed circular trajectory")

def move_point_to_point(target_x, target_y, linear_speed=1.0):
    """
    Move the turtle to a specific (x, y) point.
    :param target_x: Target x-coordinate.
    :param target_y: Target y-coordinate.
    :param linear_speed: Speed of movement.
    """
    if not is_within_bounds(target_x, target_y):
        rospy.logwarn(f"Target point ({target_x}, {target_y}) is out of bounds!")
        return

    global cmd_vel_pub, current_pose
    velocity_msg = Twist()
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        if check_for_pause_key():
            break  # Exit the loop if 'p' is pressed

        # Calculate distance and angle to the target
        distance = math.sqrt((target_x - current_pose.x) ** 2 + (target_y - current_pose.y) ** 2)
        angle_to_target = math.atan2(target_y - current_pose.y, target_x - current_pose.x)

        # Adjust angular velocity to align with the target
        velocity_msg.angular.z = 4 * (angle_to_target - current_pose.theta)

        # Move forward if close to alignment
        velocity_msg.linear.x = linear_speed if abs(angle_to_target - current_pose.theta) < 0.1 else 0

        cmd_vel_pub.publish(velocity_msg)

        if distance < 0.1:  # Stop if close to the target
            break

        rate.sleep()

    # Stop the turtle
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    cmd_vel_pub.publish(velocity_msg)

    rospy.loginfo(f"Reached target point ({target_x}, {target_y})")

def move_zigzag(zig_length, zig_angle, zig_count):
    """
    Move the turtle in a zigzag trajectory.
    :param zig_length: Length of each zig or zag.
    :param zig_angle: Angle between zig and zag.
    :param zig_count: Total number of zigs (and zags).
    """
    for i in range(zig_count):
        move_forward(zig_length)
        if i % 2 == 0:
            rotate_angle(zig_angle)  # Turn for zig
        else:
            rotate_angle(-zig_angle)  # Turn for zag

    rospy.loginfo(f"Completed zigzag trajectory with {zig_count} zigs.")

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
        print("8. You can enter (p) if you want to stop the turtule motion at any time")

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
                dr_increment = float(input("Enter the radius increment per step: "))/100.0
                move_spiral(dr_increment)

            elif choice == 5:
                rospy.loginfo("Point-to-Point trajectory selected")
                target_x = float(input("Enter the target x-coordinate: "))
                target_y = float(input("Enter the target y-coordinate: "))
                move_point_to_point(target_x, target_y)

            elif choice == 6:
                rospy.loginfo("Zigzag trajectory selected")
                zig_length = float(input("Enter the length of each zig or zag: "))
                zig_angle = float(input("Enter the angle of each zig or zag (degrees): "))
                zig_count = int(input("Enter the number of zigs: "))
                move_zigzag(zig_length, zig_angle, zig_count)

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
