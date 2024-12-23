### **Implement Triangle Trajectory**

#### **Actions** :

1. Open the `turtle_trajectory.py` script for editing:
   ```bash
   nano ~/catkin_ws/src/turtle_trajectory_pkg/scripts/turtle_trajectory.py
   ```
2. Add the `move_triangle` function:
   ```python
   def move_triangle(side_length):
       """Move the turtle in a triangle trajectory."""
       for _ in range(3):  # A triangle has three sides
           move_forward(side_length)
           rotate_angle(120)  # Rotate 120 degrees for each corner
       rospy.loginfo("Completed triangle trajectory")
   ```
3. Update the menu in the `main_menu` function to call `move_triangle`:
   ```python
   def main_menu():
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
                   edge_length = float(input("Enter the side length of the square: "))
                   move_square(edge_length)
               elif choice == 2:
                   rospy.loginfo("Triangle trajectory selected")
                   side_length = float(input("Enter the side length of the triangle: "))
                   move_triangle(side_length)
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
   ```
4. Test the triangle trajectory:
   * Run the script and select the **Triangle** option from the menu.
   * Enter a valid side length (e.g., `3.0`) and observe the turtle moving in a triangle.
5. Stage and commit the changes:
   ```bash
   cd ~/catkin_ws
   git add src/turtle_trajectory_pkg/scripts/turtle_trajectory.py
   git commit -m "Implement move_triangle function"
   ```

---
