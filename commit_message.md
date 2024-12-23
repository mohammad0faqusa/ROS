
---

### **Sequence of Actions Implemented in Step 6**

#### **1. Declared Global Variables**

 **Code** :

```python
cmd_vel_pub = None
current_pose = None
```

 **Explanation** :

* **`cmd_vel_pub`** : A global variable to store the publisher for sending velocity commands to the `/turtle1/cmd_vel` topic.
* **`current_pose`** : A global variable to store the current position and orientation of the turtle, updated by the `/turtle1/pose` subscriber.

---

#### **2. Created `pose_callback` Function**

 **Code** :

```python
def pose_callback(pose):
    """Callback function to update the current pose of the turtle."""
    global current_pose
    current_pose = pose
```

 **Explanation** :

* **Purpose** : This function updates the `current_pose` global variable with the latest `x`, `y`, and `theta` values from the `/turtle1/pose` topic.
* **How It Works** :
* Subscribes to the `/turtle1/pose` topic.
* Automatically executed whenever a new `Pose` message is published.

---

#### **3. Implemented `move_forward` Function**

 **Code** :

```python
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
```

 **Explanation** :

* **Purpose** : Moves the turtle forward by a specified distance.
* **How It Works** :
* Captures the turtle’s initial position (`start_x` and `start_y`).
* Publishes a linear velocity command (`velocity_msg.linear.x`) to the `/turtle1/cmd_vel` topic.
* Continuously calculates the Euclidean distance moved and stops when the turtle reaches or exceeds the target distance.
* Sends a stop command to halt the turtle.

---

#### **4. Implemented `rotate_angle` Function**

 **Code** :

```python
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
```

 **Explanation** :

* **Purpose** : Rotates the turtle by a specified angle (in degrees).
* **How It Works** :
* Converts the angle from degrees to radians.
* Publishes angular velocity commands (`velocity_msg.angular.z`) to the `/turtle1/cmd_vel` topic.
* Tracks the angle rotated using the difference between the current and initial `theta` values.
* Stops when the desired rotation is completed.

---

#### **5. Updated the `main()` Function**

 **Code** :

```python
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
```

 **Explanation** :

* **Purpose** : Sets up the ROS node, initializes the publisher and subscriber, and starts the main menu.
* **How It Works** :
* Initializes the ROS node (`rospy.init_node`).
* Publishes velocity commands to `/turtle1/cmd_vel` via the `cmd_vel_pub`.
* Subscribes to the `/turtle1/pose` topic to update the turtle’s pose.
* Calls `main_menu()` after ensuring all connections are established.

---

#### **6. Tested Motion Functions**

**Temporary Testing Code** (Added to the menu system for testing):

```python
if choice == 1:
    rospy.loginfo("Testing move_forward function")
    move_forward(2.0, speed=1.0)  # Example test: move forward 2 meters
elif choice == 2:
    rospy.loginfo("Testing rotate_angle function")
    rotate_angle(90, angular_speed=30.0)  # Example test: rotate 90 degrees
```

 **Explanation** :

* Temporarily added calls to `move_forward` and `rotate_angle` in the menu system to verify functionality.
* These tests confirmed that the turtle moves as expected based on the utility functions.

---

### **Keywords Summary**

#### **Libraries**

1. **`rospy`** :

* Manages the ROS node and interactions.
* Functions used: `rospy.init_node`, `rospy.Publisher`, `rospy.Subscriber`, `rospy.is_shutdown`, `rospy.Rate`, `rospy.sleep`.

1. **`geometry_msgs.msg.Twist`** :

* Used for controlling the turtle’s linear and angular velocities.

1. **`turtlesim.msg.Pose`** :

* Provides real-time position and orientation of the turtle.

1. **`math`** :

* Used for mathematical calculations such as converting degrees to radians and calculating distances.

---

#### **Variables**

1. **`cmd_vel_pub`** :

* A global publisher for sending velocity commands to the turtle.

1. **`current_pose`** :

* A global variable updated with the turtle’s latest position and orientation.

1. **`velocity_msg`** :

* An instance of `Twist` used to set linear and angular velocities.

---

#### **Functions**

1. **`pose_callback(pose)`** :

* Updates the turtle’s current position and orientation.

1. **`move_forward(distance, speed=1.0)`** :

* Moves the turtle forward by a specified distance.

1. **`rotate_angle(angle_deg, angular_speed=30.0)`** :

* Rotates the turtle by a specified angle (in degrees).

1. **`main()`** :

* Sets up the ROS node, initializes the publisher and subscriber, and starts the main menu.

---

This sequence, along with the detailed explanations and code snippets, can now be copied into `commit_message.md`. Let me know if you need help integrating or formatting this!
