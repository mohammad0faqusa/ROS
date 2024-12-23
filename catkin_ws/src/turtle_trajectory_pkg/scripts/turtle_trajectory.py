#!/usr/bin/env python3

import rospy 

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
                # Placeholder for move_square()
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
    rospy.init_node('turtle_trajectory', anonymous=True)
    rospy.loginfo('Turtle trajectory initialized')
    main_menu()


if __name__ == "__main__":
    main() 


