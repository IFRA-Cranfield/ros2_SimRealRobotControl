## ROS2 Sim-to-Real Robot Control: ABB IRB-120

### INSTRUCTIONS: Launch ROS2 Packages

__Launch Gazebo Simulation Environment__

The following command launches a Gazebo Simulation Environment with the ABB IRB-120 in it. It does not have any special functionality, but it could be useful to check that CAD/mesgh files and components have been added to the environments successfully:
```sh
ros2 launch ros2srrc_irb120_gazebo irb120_simulation.launch.py
```

__Launch Gazebo + MoveIt!2 Environment__

The following command launches the Gazebo Simulation Environment and the MoveIt!2 Framework for the control of the ABB IRB-120 robot:
```sh
ros2 launch ros2srrc_irb120_moveit2 irb120_interface.launch.py
```

__Launch RobotBringup + MoveIt!2 Environment__

The following command launches the ROS2 Node that establishes the connection between the Robot's ROS2 Driver and the Robot Controller, and the MoveIt!2 Framework for the control of the robot:
```sh
ros2 launch ros2srrc_irb120_moveit2 irb120_bringup.launch.py
```

### EXAMPLES

__Execute a program for the ABB IRB-120 in a Gazebo Simulation__

TBD.

__Connect ROS2 with an ABB IRB-120 in ABB Robot Studio and execute a program__

TBD.

__Connect ROS2 with a Real ABB IRB-120 and execute a program__

TBD.

