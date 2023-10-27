## ROS2 Sim-to-Real Robot Control: Universal Robots - Dobot Magician

### INSTRUCTIONS: Launch ROS2 Packages

__Launch Gazebo Simulation Environment__

The following command launches a Gazebo Simulation Environment with the Dobot Magician in it. It does not have any special functionality, but it could be useful to check that CAD/mesh files and components have been added to the environments successfully:
```sh
ros2 launch ros2srrc_dobot_gazebo dobot_simulation.launch.py
```

__Launch Gazebo + MoveIt!2 Environment__

The following command launches the Gazebo Simulation Environment and the MoveIt!2 Framework for the control of the Dobot Magician robot:
```sh
ros2 launch ros2srrc_dobot_moveit2 dobot_interface.launch.py
```

__Launch RobotBringup + MoveIt!2 Environment__

The following command launches the ROS2 Node that establishes the connection between the Robot's ROS2 Driver and the Robot Controller, and the MoveIt!2 Framework for the control of the robot:
```sh
NOT IMPLEMENTED YET
```

### EXAMPLES

__Execute a program for the Dobot Magician in a Gazebo Simulation__

TBD.


__Connect ROS2 with a Real Dobot Magician and execute a program__

TBD.

