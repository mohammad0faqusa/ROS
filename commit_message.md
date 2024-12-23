### **Step 15 Explanation: Add Launch File**

#### **What Has Been Done**

In this step, we created a **ROS launch file** named `turtle_trajectory.launch` to streamline the process of running the turtlesim simulator and the trajectory controller node together. This file simplifies launching multiple nodes simultaneously with a single command, which is essential for maintaining workflow efficiency and reducing manual startup steps.

#### **Features Added**

1. **Launch File (`turtle_trajectory.launch`):**
   * Starts the `turtlesim_node`, which simulates the turtle's environment.
   * Starts the `turtle_trajectory.py` node from the `turtle_trajectory_pkg` package, which handles trajectory control.
   * Includes configuration to run the Python script with the appropriate interpreter (`python3`).
2. **README Update:**
   * Added detailed instructions on how to use the new launch file to start the application.

---

#### **Purpose of the Launch File**

* **Simplifies Node Management:** Instead of manually launching the `turtlesim_node` and the `turtle_trajectory` node in separate terminals, the launch file starts both nodes automatically.
* **Consistency:** Ensures both nodes are launched in the correct order with predefined settings.
* **Ease of Use:** Reduces the risk of errors during node startup and makes it user-friendly for new users or testers.

---

#### **How It Works**

The launch file contains two `<node>` elements:

1. **Turtlesim Node:**

   ```xml
   <node pkg="turtlesim" type="turtlesim_node" name="turtlesim" output="screen"/>
   ```

   * **Package:** `turtlesim`
   * **Node Type:** `turtlesim_node` (the binary file that simulates the turtle environment)
   * **Name:** `turtlesim` (the node will be referred to with this name)
   * **Output:** Sends log messages to the screen for visibility.
2. **Turtle Trajectory Controller Node:**

   ```xml
   <node pkg="turtle_trajectory_pkg" type="turtle_trajectory.py" name="turtle_trajectory" output="screen" launch-prefix="python3"/>
   ```

   * **Package:** `turtle_trajectory_pkg`
   * **Node Type:** `turtle_trajectory.py` (the Python script handling the trajectory logic)
   * **Name:** `turtle_trajectory`
   * **Output:** Sends log messages to the screen for visibility.
   * **Launch Prefix:** Ensures the script runs with `python3`.

---

#### **Code Example: Launch File Content**

```xml
<launch>
    <!-- Launch the turtlesim simulator -->
    <node pkg="turtlesim" type="turtlesim_node" name="turtlesim" output="screen"/>

    <!-- Launch the trajectory controller node -->
    <node pkg="turtle_trajectory_pkg" type="turtle_trajectory.py" name="turtle_trajectory" output="screen" launch-prefix="python3"/>
</launch>
```

---

#### **How to Use It**

1. Run the following command to launch the application:
   ```bash
   roslaunch turtle_trajectory_pkg turtle_trajectory.launch
   ```
2. **Expected Behavior:**
   * The `turtlesim_node` will start, displaying the turtle simulator window.
   * The `turtle_trajectory` node will initialize, displaying the trajectory control menu in the terminal.

---
