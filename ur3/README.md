## ROS2 Sim-to-Real Robot Control: Universal Robots - UR3

### INSTRUCTIONS: Launch ROS2 Packages

__Launch Gazebo Simulation Environment__

The following command launches a Gazebo Simulation Environment with the UR3 in it. It does not have any special functionality, but it could be useful to check that CAD/mesh files and components have been added to the environments successfully:
```sh
ros2 launch ros2srrc_ur3_gazebo ur3_simulation.launch.py
```

__Launch Gazebo + MoveIt!2 Environment__

The following command launches the Gazebo Simulation Environment and the MoveIt!2 Framework for the control of the UR3 robot:
```sh
ros2 launch ros2srrc_ur3_moveit2 ur3_interface.launch.py
```

__Launch RobotBringup + MoveIt!2 Environment__

The following command launches the ROS2 Node that establishes the connection between the Robot's ROS2 Driver and the Robot Controller, and the MoveIt!2 Framework for the control of the robot:
```sh
ros2 launch ros2srrc_ur3_bringup ur3_bringup.launch.py
```

### EXAMPLES

__Execute a program for the UR3 in a Gazebo Simulation__

TBD.

__Connect ROS2 with an ABB UR3 in ABB Robot Studio and execute a program__

TBD.

__Connect ROS2 with a Real ABB UR3 and execute a program__

TBD.

