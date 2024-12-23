### Explanation of Step 14: Implement Pause/Return-to-Menu Logic

#### **Features Added**

1. **Pause Functionality** :

* Introduced the ability to pause the turtle’s trajectory and return to the main menu by pressing the `p` key during motion.
* This functionality allows users to interrupt ongoing trajectories without waiting for them to complete.

1. **Non-Blocking Input Handling** :

* Utilized Python's `select` module to implement non-blocking input detection.
* This ensures the program continues to execute the trajectory logic while simultaneously checking for user input.

---

#### **Purpose**

1. **User Control** :

* The pause functionality provides users with greater control over the turtle's motion, enabling them to stop and change trajectories at any time.

1. **Enhanced Interactivity** :

* By allowing interruption, users can experiment with multiple trajectories without restarting the program.

---

#### **How It Works**

1. **Pause Key Detection** :

* The function `check_for_pause_key()` listens for the `p` key during motion. If detected, it breaks out of the motion loop.
* This is achieved using the `select` module to check for user input with a short timeout, ensuring non-blocking behavior.

```python
   import select
   import sys

   def check_for_pause_key():
       """
       Check if the user has pressed 'p' to pause the trajectory and return to the menu.
       :return: True if 'p' is pressed, False otherwise.
       """
       print("Press 'p' to pause and return to the menu.")
       i, _, _ = select.select([sys.stdin], [], [], 0.1)  # Wait for input for 0.1 seconds
       if i:
           key = sys.stdin.read(1)
           if key == 'p':
               rospy.loginfo("Pause key detected. Returning to menu.")
               return True
       return False
```

1. **Integration with Motion Functions** :

* Each motion function (e.g., `move_forward`, `move_circular`) was modified to call `check_for_pause_key()` inside their loops.
* If the `p` key is detected, the function breaks out of the loop and stops the turtle.

   Example from `move_forward`:

```python
   def move_forward(distance, speed=1.0):
       """Move the turtle forward a specified distance."""
       global cmd_vel_pub, current_pose
       velocity_msg = Twist()
       rate = rospy.Rate(10)  # 10 Hz

       start_x, start_y = current_pose.x, current_pose.y
       velocity_msg.linear.x = speed

       while not rospy.is_shutdown():
           if check_for_pause_key():
               break  # Exit the loop if 'p' is pressed

           distance_moved = math.sqrt((current_pose.x - start_x) ** 2 +
                                      (current_pose.y - start_y) ** 2)
           if distance_moved >= distance:
               break
           cmd_vel_pub.publish(velocity_msg)
           rate.sleep()

       velocity_msg.linear.x = 0
       cmd_vel_pub.publish(velocity_msg)
```

1. **User Prompt** :

* While the turtle is in motion, a message is displayed: `"Press 'p' to pause and return to the menu."`
* This prompt informs users of the pause functionality during trajectory execution.

1. **README Update** :

* Instructions about the pause functionality were added to the `README.md`:
  ```markdown
  ## Pause Functionality
  During any trajectory, you can press `p` to pause the turtle's motion and return to the main menu.
  ```

---

#### **Code Added**

1. `check_for_pause_key()` function:
   * Implements the core logic for detecting the pause key.
2. Integration into all motion functions:
   * Functions like `move_forward`, `move_circular`, and others now include `check_for_pause_key()` to handle pausing.
3. Updated README documentation.

---

#### **Commit Message**

`Add pause functionality using non-blocking input`

---

This explanation and details can be copied into a `commit_message.md` file. Would you like me to create it for you?
