### **Commit Message Explanation for Step 12: Implement Zigzag Trajectory**

#### **What has been done?**

In this step, the `move_zigzag` function was implemented in the `turtle_trajectory.py` script. This function enables the turtle to move in a zigzag trajectory based on user-defined parameters such as the length of each zig or zag, the angle of the zigzag, and the total number of zigs (and zags). The function was integrated into the main menu system so that users can select the zigzag trajectory as one of the options.

---

#### **Features Added**

1. **Zigzag Motion Functionality** :

* A new function, `move_zigzag`, was added to control the turtle's movement in a zigzag pattern.
* Parameters allow for flexible customization of the zigzag motion:
  * `zig_length`: The forward distance for each zig or zag.
  * `zig_angle`: The angular turn for the zigzag motion.
  * `zig_count`: The total number of zig motions (alternates between zig and zag).

1. **User Interaction** :

* The menu was updated to include the zigzag option, allowing the user to provide inputs for `zig_length`, `zig_angle`, and `zig_count`.
* The program validates the inputs and executes the zigzag motion.

---

#### **Purpose**

The purpose of the zigzag trajectory is to provide a dynamic motion pattern that can simulate scenarios requiring alternating directional movement. This is useful in robotics for obstacle avoidance, search patterns, or demonstrations of complex motion paths.

---

#### **How It Works**

1. **User Input** :

* The user selects the "Zigzag" option from the main menu.
* The program prompts the user to input the length of each zig or zag, the angle of the zigzag (in degrees), and the number of zigs.

1. **Execution** :

* The `move_zigzag` function moves the turtle forward by the specified `zig_length` and alternates the turn angle between `+zig_angle` and `-zig_angle` for each zigzag step.
* The turtle repeats this motion for the specified number of zigs.

1. **Code Highlights** :

```python
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
```

1. **Menu Integration** :

* The zigzag option is added to the main menu:
  ```python
  elif choice == 6:
      rospy.loginfo("Zigzag trajectory selected")
      zig_length = float(input("Enter the length of each zig or zag: "))
      zig_angle = float(input("Enter the angle of each zig or zag (degrees): "))
      zig_count = int(input("Enter the number of zigs: "))
      move_zigzag(zig_length, zig_angle, zig_count)
  ```

1. **Stopping Condition** :

* The turtle completes the zigzag motion after executing the specified number of zigs.

---
