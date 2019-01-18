<h1 style="border:none"> RISE ABB IRB-120 Manipulation Package </h1>
&copy; 2018, Susung Park

<hr>

## 1. How to Install

Extract two separate packages `rise_assembler` and `rise_assembler_moveit` either in ROS or MoveIt! workspace. `catkin_make` both workspaces (or the workspace if ROS and MoveIt! are not separated). That's it, you're ready to go!


## 2. Structure of Package

The `rise_assembler` package is a metapackage, including three subsidiary packages, which are written in the following.

* `rise_assembler_control`
* `rise_assembler_launcher`
* `rise_assembler_model`

`rise_assembler_control` contains codes to spawn ROS-Industrial controllers, and Python scripts that will take charge of assigning high-level tasks to the robot.

`rise_assembler_launcher` contains launchers, and this is the package that the highest-level launch files are located in. Of course, the other two packages also contain launch files, but they are fragmentary. Therefore, whenever one would use this package, launcher files in this package should be used and using those in other packages is **DEPRECATED**.

`rise_assembler_model` contains modeling files, which are required for Gazebo simulations and non-collisive path planning in MoveIt!. Currently, for the use in laboratory, a gripper is attached as the end-effector. Those who are fluent in manipulting `.urdf` or `.xacro` files might simply erase the `<joint>` and `<link>` tags that defines the bondage between the 6<sup>th</sup> link and the gripper, and the gripper itself.


## 3. How to Use

**Disclaimer |** As the writer of this package started this project from being a novice in ROS (lmao..), currently the package is not organized cleanly.

You can use Python codes to take control of this package.