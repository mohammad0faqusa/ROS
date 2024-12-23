
---

## **Commit Title: Implement move_square function**

### **What Was Done:**

1. **Added a Function** :

* Implemented the `move_square` function, which makes the turtle move in a square trajectory by combining straight-line movements and 90-degree rotations.

1. **Modified the Menu** :

* Updated the `main_menu` function to allow users to select the "Square" trajectory and input the side length.

1. **Tested the Functionality** :

* Ran the script, selected the "Square" option, provided the side length, and verified the turtle moved in a square.

---

### **Steps to Build**

#### **1. Adding the `move_square` Function**

The function `move_square` was added to the script:

```python
def move_square(edge_length):
    """Move the turtle in a square trajectory."""
    for _ in range(4):  # A square has four sides
        move_forward(edge_length)  # Move forward by the given edge length
        rotate_angle(90)  # Rotate 90 degrees at each corner
    rospy.loginfo("Completed square trajectory")
```

* **Explanation** :
* This function calls `move_forward` to move a straight distance equal to the side length.
* It then calls `rotate_angle(90)` to rotate the turtle 90 degrees clockwise. This repeats four times to complete a square.

---

#### **2. Updating the Menu**

The `main_menu` function was updated to include an option for the square trajectory:

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
                move_square(edge_length)  # Call the move_square function
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
```

* **Explanation** :
* When the user selects option  **1** , they are prompted to input the side length for the square.
* The program then calls the `move_square` function with the inputted length.

---

### **How to Run and Verify**

1. **Start the ROS Core** :

```bash
   roscore
```

1. **Launch the Turtlesim Node** :

```bash
   rosrun turtlesim turtlesim_node
```

1. **Run the Trajectory Script** :

```bash
   rosrun turtle_trajectory_pkg turtle_trajectory.py
```

1. **Verify the Square Trajectory** :

* Select option **1** from the menu.
* Input a side length (e.g., `2.0`).
* Observe the turtle moving in a square with the specified side length.

---

### **Expected Output**

1. The program should print logs like:
   ```
   Square trajectory selected
   Enter the side length of the square: 2.0
   Completed square trajectory
   ```
2. The turtle should visibly move in a square path in the Turtlesim window.

---
