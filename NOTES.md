## Some Acknowledgements!
* Be careful of namespaces. They are everywhere, even in .xacro files.
    * In .xacro files, there is a part to setup which type of simulator to use. In this scope, one should setup the namespace of simulation. (`<gazebo>` tag)
* Be careful of the details of .xacro files.
    * In .xacro file `<transmission>` tags, there is a sub-tag named `<hardwareInterface>`. This tag does matter, when changing the controller between position controllers and effort (force) controllers.
* Workflow of ROSLAUNCH
    * First, launch an empty world in Gazebo simulator.
    * Next, load the .urdf file into the parameter server. The file is converted from .xacro to .urdf, but not parsed. Just the raw .urdf file will go into the parameter server. This file will be parsed by `gazebo_ros` package later.
    * Read the controller .yaml files, and load these onto the parameter server.
    * Parse these .yaml files using `controller_manager` package.
    * Now, we're ready both for the topology and joint controls.
    * Run `spawn_model` node of `gazebo_ros` package to spawn the robot model onto Gazebo simulator. The .urdf files will be parsed into usable ROS parameters and then will show up as a complete robot model on the simulator.
* Remapped topics seem to be disappeared. The destination topic remains, but original topic is not seen on `rostopic list`.
    * If a topic (or namespace) had subsidiary elements below it, they will also be automatically appended to the namespaces.
* Be careful not to confuse between controllers and action servers.
    * The followings are types of **CONTROLLERS**.
        * effort_controllers - Command a desired force/torque to joints.
            * joint_effort_controller
            * joint_position_controller
            * joint_velocity_controller
        * joint_state_controller - Read all joint positions.
            * joint_state_controller
        * position_controllers - Set one or multiple joint positions at once.
            * joint_position_controller
            * joint_group_position_controller
        * velocity_controllers - Set one or multiple joint velocities at once.
            * joint_velocity_controller
            * joint_group_velocity_controller
        * joint_trajectory_controllers - Extra functionality for splining an entire trajectory.
            * position_controller
            * velocity_controller
            * effort_controller
            * position_velocity_controller
            * position_velocity_acceleration_controller
        * gripper_command - **Don't be confused by its weird name!**
    * `JointTrajectoryAction` is a type of action, which is needed since trajectory execution requires finite amount of time.
    * `move_group` node subscribes to `/joint_states` topic, which gives information of states of each joint.
        * This topic is published by `Gazebo` or `joint_state_publisher` package, *not by any controller packege*.