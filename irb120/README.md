## ROS2 Sim-to-Real Robot Control: ABB IRB-120

### INSTRUCTIONS: Launch ROS2 Packages

__Launch Gazebo Simulation Environment__

The following command launches a Gazebo Simulation Environment with the ABB IRB-120 in it. It does not have any special functionality, but it could be useful to check that CAD/mesh files and components have been added to the environments successfully:
```sh
ros2 launch ros2srrc_irb120_gazebo simulation.launch.py layout:=ABC endeffector:=ABC 
```

__Launch Gazebo + MoveIt!2 Environment__

The following command launches the Gazebo Simulation Environment and the MoveIt!2 Framework for the control of the ABB IRB-120 robot:
```sh
ros2 launch ros2srrc_irb120_moveit2 irb120_interface.launch.py layout:=ABC endeffector:=ABC
```

__Launch RobotBringup + MoveIt!2 Environment__

The following command launches the ROS2 Node that establishes the connection between the Robot's ROS2 Driver and the Robot Controller, and the MoveIt!2 Framework for the control of the robot:
```sh
ros2 launch ros2srrc_irb120_bringup irb120_bringup.launch.py layout:=ABC endeffector:=ABC robot_ip:=ABC
```

__Variants for the ROS2srrc ABB IRB120__

Layout:
- ros2srrc_irb120_1: ABB IRB-120 alone.
- ros2srrc_irb120_2: Cranfield University IA Lab enclosure.
- ros2srrc_irb120_3: Pick and Place Use-Case.

End-Effector:
- RobAlone: Robot alone: No End-Effector.
- egp64: Schunk EGP-64 Parallel Gripper.

### EXAMPLES

__Execute a program for the ABB IRB-120 in a Gazebo Simulation__

TBD.

__Connect ROS2 with an ABB IRB-120 in ABB Robot Studio and execute a program__

TBD.

__Connect ROS2 with a Real ABB IRB-120 and execute a program__

TBD.

