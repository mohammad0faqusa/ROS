### **Step 5: Add Menu System & Basic Input**

#### **Actions**:

1. Open the `turtle_trajectory.py` script for editing:

   ```bash
   nano ~/catkin_ws/src/turtle_trajectory_pkg/scripts/turtle_trajectory.py
   ```
2. Update the script to include a menu system:

   ```python
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
       rospy.loginfo("Turtle Trajectory Node Initialized")
       main_menu()

   if __name__ == '__main__':
       main()
   ```
3. Stage and commit the changes:

   ```bash
   cd ~/catkin_ws
   git add src/turtle_trajectory_pkg/scripts/turtle_trajectory.py
   git commit -m "Implement main menu system"
   ```

---

#### **Commit Message**:

`Implement main menu system`

---

Let me know when you're ready to move on to **Step 6: Implement Basic Move & Rotate Functions**!
