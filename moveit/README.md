# IFRA-Cranfield: ROS2 Sim-to-Real Robot Control

## ros2srrc-moveit package

Within ros2_SimRealRobotControl, the MoveIt!2 framework is responsible for kinematic control, motion planning, and collision detection of the robots.

The ros2srrc_moveit package provides all the necessary configuration files required for integrating robots with MoveIt!2. It contains a single /config folder, which includes the following key elements:

- _RVIZ Configuration (.rviz)_: Defines the visualization of the robot in RViz, the ROS 2 visualization tool. This file specifies the robot model, display settings, and any preconfigured markers or interactive controls. RViz is used to visualize the robot’s state and to plan and preview motions. _Example_: irb120_egp64.rviz displays the ABB IRB-120 robot together with the Schunk EGP-64 gripper, offering an intuitive interface for motion planning.

- _SRDF Configuration (.srdf)_: The Semantic Robot Description Format (SRDF) complements the URDF by defining the robot’s semantic information, such as self-collision matrices, kinematic groups, and predefined poses. This information is essential for MoveIt!2 to perform inverse kinematics, motion planning, and collision checking. _Example_: irb120.srdf specifies which links are allowed to collide and provides the semantic data required for planning with the ABB IRB-120.

__NOTE__: In ros2srrc, every robot or robot + end-effector combination requires a paired .rviz and .srdf file. To ensure reusability and avoid regenerating these files for each new robot cell, all predefined RViz and SRDF configurations are grouped in the ros2srrc_moveit package.