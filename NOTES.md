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