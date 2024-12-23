### Explanation of Step 10: Implement Spiral Trajectory

#### **Overview**

In this step, we implemented the **spiral trajectory** feature for the Turtle Trajectory project. This feature allows the turtle to move in an outward spiral motion, starting from a small radius and gradually expanding its path as it moves forward. This is achieved by incrementally increasing the radius while maintaining a linear speed.

---

#### **Features Added**

1. **Spiral Motion** : The turtle moves in a spiral trajectory with customizable radius increments.
2. **Dynamic Radius Adjustment** : The radius increases in small increments (`dr_increment`) during each iteration, creating the spiral effect.
3. **Duration Limit** : The spiral motion is limited by time to prevent indefinite execution.
4. **Integration into the Menu** : Users can select the spiral trajectory from the main menu and specify the radius increment dynamically.

---

#### **Purpose**

The spiral trajectory demonstrates the ability to:

1. Combine linear and angular velocities to produce complex motions.
2. Allow dynamic trajectory adjustment (e.g., radius increment).
3. Show how simple adjustments in velocity can create visually interesting patterns in robotic motion.

---

#### **How It Works**

1. The function `move_spiral(dr_increment, linear_speed)` takes two parameters:

   * `dr_increment`: The amount by which the radius increases in each iteration.
   * `linear_speed`: The speed at which the turtle moves forward.
2. Angular speed (`angular_speed`) is calculated dynamically as:

   ```python
   angular_speed = linear_speed / radius
   ```

   This ensures the turtle moves in a consistent spiral pattern.
3. The function publishes a `Twist` message to the `/turtle1/cmd_vel` topic at regular intervals using a loop, which:

   * Sets the linear velocity (`velocity_msg.linear.x`).
   * Sets the angular velocity (`velocity_msg.angular.z`).
4. The radius is incremented in each iteration:

   ```python
   radius += dr_increment
   ```
5. The loop runs until a set duration (e.g., 20 seconds) is reached.
6. After completing the trajectory, the turtle's motion stops by publishing a zero velocity.

---

#### **Key Code**

Below is the core implementation of the spiral trajectory:

```python
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
```

---

#### **Menu Integration**

The spiral trajectory is integrated into the main menu with the ability to input the radius increment (`dr_increment`):

```python
elif choice == 4:
    rospy.loginfo("Spiral trajectory selected")
    dr_increment = float(input("Enter the radius increment per step: "))
    move_spiral(dr_increment)
```

---

#### **Testing**

* Select **Spiral** from the menu.
* Enter a valid increment value (e.g., `0.2`).
* Observe the turtle move outward in a smooth spiral pattern.

---
