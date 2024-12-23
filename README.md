

# Turtle Trajectory Project

Welcome to the  **Turtle Trajectory Project** , a ROS-based system that demonstrates various trajectory paths in the [turtlesim](http://wiki.ros.org/turtlesim) simulator. This documentation will guide you through the setup, usage, and the step-by-step commits that built the project from scratch.

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Prerequisites](#prerequisites)
3. [Installation & Setup](#installation--setup)
4. [Usage](#usage)
   1. [Launching the Project](#launching-the-project)
   2. [Menu Options](#menu-options)
   3. [Pause Functionality](#pause-functionality)
5. [Boundary Checks](#boundary-checks)
6. [table of commits](#table-of-commits)
   1. [Implement move_circular function](https://github.com/mohammad0faqusa/ROS/blob/51a26fbfd6fbf47f436ce857ecbd04e967b0bc9a/commit_message.md)
   2. [Implement move_spiral function](https://github.com/mohammad0faqusa/ROS/blob/1135fb9c07167cb119b0ad85c4401180484c2f66/commit_message.md)
   3. [Implement move_point_to_point function](https://github.com/mohammad0faqusa/ROS/blob/ee322920b69927a0f8fb039fbb1c208d9ecf0b38/commit_message.md)
   4. [Implement move_zigzag function](https://github.com/mohammad0faqusa/ROS/blob/c06733e04e56f05f10ec6a6a5b41cdf698b1d6b3/commit_message.md)
   5. [Add pause functionality using non-blocking input](https://github.com/mohammad0faqusa/ROS/blob/f45d20f20f8eb2eb22579f1359256d2f4a285306/commit_message.md)
   6. [Add turtle_trajectory.launch file](https://github.com/mohammad0faqusa/ROS/blob/bcdb36505a1a3acb3020343b882ef1ac4227784a/commit_message.md)
   7. [Update README.md - Project documentation](https://github.com/mohammad0faqusa/ROS/blob/37c668a083b938ee8027244d9abaacec6e0aabdd/commit_message.md)
   8. [Update README.md](https://github.com/mohammad0faqusa/ROS/blob/1cdb9e15c65f9cb4f4da55895ea406b573f6a30b/commit_message.md)
   9. [Update README.md](https://github.com/mohammad0faqusa/ROS/blob/525883b8aa65e8b256896c24a7967fb8ec79dfa0/commit_message.md)
8. [Repository Structure](#repository-structure)
9. [License](#license)

---

## Project Overview

This package allows users to control the turtle’s movement in various  **pre-defined trajectories** —such as Square, Triangle, Circular, Spiral, Point-to-Point, Zigzag—and also provides helpful **utility functions** (e.g., move and rotate) and **boundary checks** to keep the turtle within the simulator limits.

---

## Prerequisites

* **ROS** (Robot Operating System) installed (e.g., ROS Noetic on Ubuntu 20.04).
* **Turtlesim** package (usually included with standard ROS desktop installations).
* **catkin_tools** or **catkin_make** to build the workspace.
* **Python 3** for running the scripts (depending on your ROS version; ROS Noetic supports Python 3).

---

## Installation & Setup

1. **Clone this repository** (assuming your GitHub URL is `https://github.com/mohammad0faqusa/ROS.git`):

   ```bash
   cd ~
   git clone https://github.com/mohammad0faqusa/ROS.git
   ```
2. **Create and initialize your Catkin workspace** if you haven’t already:

   ```bash
   mkdir -p ~/catkin_ws/src
   cp -r ROS ~/catkin_ws/src/
   cd ~/catkin_ws
   catkin_make
   ```

   > If you already have a catkin workspace, just copy or clone the repo into `~/catkin_ws/src/`.
   >
3. **Source your workspace** :

```bash
   source ~/catkin_ws/devel/setup.bash
```

---

## Usage

### Launching the Project

To launch the turtlesim simulator **and** the trajectory controller node simultaneously, use:

```bash
roslaunch turtle_trajectory_pkg turtle_trajectory.launch
```

This launch file will:

1. Start the `turtlesim_node`.
2. Start the `turtle_trajectory` Python script (controller node).

### Menu Options

Once the `turtle_trajectory` node starts, you will see a **menu** in your terminal:

```
Select a motion trajectory:
1. Square
2. Triangle
3. Circular
4. Spiral
5. Point to Point
6. Zigzag
7. Exit
```

* **Square** : Moves the turtle in a perfect square path (you’ll be asked for the side length).
* **Triangle** : Moves the turtle in an equilateral triangle path (you’ll be asked for the side length).
* **Circular** : Moves the turtle in a circle based on a specified radius.
* **Spiral** : Moves the turtle in an expanding spiral path.
* **Point to Point** : Moves the turtle to a user-specified (x, y) coordinate.
* **Zigzag** : Moves the turtle in a zigzag pattern, based on length, angle, and number of zigs.
* **Exit** : Shuts down the trajectory node.

### Pause Functionality

During any trajectory execution, **press `p`** in the terminal to pause the turtle’s movement and  **return to the main menu** .

---

## Boundary Checks

The turtle’s environment is defined with the following bounds:

```
X_MIN = 0.0  X_MAX = 11.0
Y_MIN = 0.0  Y_MAX = 11.0
```

* If a commanded trajectory tries to move the turtle beyond these boundaries, the node issues a **warning** and attempts to stop or prevent further movement outside the valid region.

---

## table of commits 


---

## Repository Structure

A typical structure for this project might look like:

```
catkin_ws/
└── src/
    └── turtle_trajectory_pkg/
        ├── launch/
        │   └── turtle_trajectory.launch
        ├── scripts/
        │   └── turtle_trajectory.py
        ├── CMakeLists.txt
        ├── package.xml
        ├── README.md          # (This documentation or a brief summary)
        └── commit_message.md  # Detailed step-by-step commit messages
```

* **launch/** : Contains the `.launch` file to run the simulator and controller node.
* **scripts/** : Contains Python scripts (in this case, `turtle_trajectory.py`) to control the turtle.
* **CMakeLists.txt** /  **package.xml** : Standard ROS package configuration.
* **commit_message.md** : Descriptions and details for each commit (or each step).
* **README.md** : Project overview and user instructions (this file).

---

## License

Include your chosen license here (e.g., MIT, Apache, BSD) or remove this section if it’s not relevant.

---

### Thank You!

We hope this documentation helps you get started with the  **Turtle Trajectory Project** . For any further questions, please check:

* The [ROS Wiki](http://wiki.ros.org/) for general ROS information.
* The `commit_message.md` file for detailed commit logs.
* The **Issues** section of this repository if you encounter any problems.

Feel free to contribute by forking the repo and submitting a pull request!

---

#### How to Link Between Markdown Files From Different Commits

If you have multiple branches or commits where `commit_message.md` (or other docs) differ, you can:

* **Use the “blob” URL pattern** with a specific commit SHA:

  ```
  https://github.com/mohammad0faqusa/ROS/blob/<COMMIT_SHA>/commit_message.md#section-anchor
  ```

  This ensures you reference the exact version of the file at that commit.
* **Use the “tree” URL pattern** with a specific branch or tag:

  ```
  https://github.com/mohammad0faqusa/ROS/tree/<BRANCH_OR_TAG>/path/to/commit_message.md
  ```

  Then add a `#section-anchor` if needed.

This approach guarantees that readers see the exact documentation as it was at that point in time.

---

**Happy Turtling!**
