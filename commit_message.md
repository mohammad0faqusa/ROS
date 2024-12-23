### **Implement Circular Trajectory**

---

#### **Features Added** :

1. **Circular Motion** :

* A function `move_circular(radius, linear_speed=1.0)` was added to enable the turtle to move in a perfect circular trajectory within the turtlesim environment.
* Users can specify the **radius** of the circle, which dynamically determines the angular speed for the motion.
* This feature calculates and completes one full circle based on the specified radius and speed.

1. **Menu Integration** :

* The **Circular** option was added to the menu. When selected, the user is prompted to input the desired circle radius. The `move_circular` function is then called to execute the motion.

---

#### **Purpose** :

The circular trajectory is one of the most fundamental motion patterns in robotics. This feature simulates the turtle moving in a predefined circular path, which is useful for understanding and demonstrating:

* Coordinated movement using linear and angular velocities.
* Basics of controlling a robot's trajectory in ROS.

---

#### **How It Works** :

1. **Input Parameters** :

* The function accepts a **radius** (distance from the center to the circular path) and an optional **linear speed** (default is 1.0).
* The **angular speed** is calculated using the formula:
  ```python
  angular_speed = linear_speed / radius
  ```

1. **Publishing Velocities** :

* The `cmd_vel` topic is used to send velocity commands to the turtle.
* A `Twist` message is created where:
  * `linear.x` is set to the linear speed.
  * `angular.z` is set to the calculated angular speed.

1. **Elapsed Time for One Circle** :

* The total time to complete one circle is calculated as:
  ```python
  time_to_complete = 2 * math.pi * radius / linear_speed
  ```
* The function uses a loop to publish velocity commands until the elapsed time exceeds the computed `time_to_complete`.

1. **Stopping the Turtle** :

* After completing the circle, the turtle is stopped by publishing a `Twist` message with all velocities set to 0.

---

#### **Code** :

Here’s the core function for circular motion:

```python
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
```

---

#### **Menu Integration** :

The `main_menu` function was updated to include the **Circular** option:

```python
elif choice == 3:
    rospy.loginfo("Circular trajectory selected")
    radius = float(input("Enter the radius of the circle: "))
    move_circular(radius)
```

---

#### **Testing** :

* After implementing, the functionality was tested by running the script, selecting the **Circular** option, and providing a radius (e.g., `2.0`).
* Observed that the turtle completed a full circle as expected.

---

#### **Commit Message** :

`Implement move_circular function`

* Added `move_circular` to enable the turtle to move in a circular trajectory based on user-defined radius and speed.
* Updated `main_menu` to include an option for circular motion.
* Calculated angular speed dynamically and completed the circle using elapsed time to ensure accuracy.
