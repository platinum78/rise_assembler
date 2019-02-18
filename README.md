<h1 style="border:none"> RISE ABB IRB-120 Manipulation Package </h1>
&copy; 2018, Susung Park

<hr>

## 1. How to Install

Extract the metapackage `rise_assembler` into `${ros_workspace}/src`. `catkin_make` your workspace. That's it, you're ready to go!


## 2. Structure of Package

To be updated...


## 3. How to Use

### 3.1. CLI Controller

You can use the CLI controller. Type `rosrun rise_assembler assembler_manual_controller` in the terminal, and you will get a CLI controller displayed up on your console. To see the list of commands, type `h` and `Enter`.

### 3.2. GUI Controller

You can use the GUI controller. Type `rosrun rise_assembler assembler_gui_controller` in the terminal, and then the GUI console will pop out. This program is not complete yet, so it might be prone to errors. Please be careful when using the program.

### 3.3. Path-Planning APIs

The script `assembler_controller.py` provides a convenient method to plan paths. One should give the pose of end-effector in the following form; [x, y, z, roll, pitch, yaw].

* `move_to_pose()`: 
* fdf