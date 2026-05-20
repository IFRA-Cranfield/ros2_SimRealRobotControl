# IFRA-Cranfield: ROS2 Sim-to-Real Robot Control

## ros2srrc_moveit package

Within ros2_SimRealRobotControl, the MoveIt!2 framework is responsible for kinematic control, motion planning, and collision detection of the robots.

The ros2srrc_moveit package provides all the necessary configuration files required for integrating robots with MoveIt!2. It contains a single /config folder, which includes the following key elements:

- _RVIZ Configuration (.rviz)_: Defines the visualization of the robot in RViz, the ROS 2 visualization tool. RViz is used to visualize the robot's state and to plan and preview motions. The shared `ros2srrc.rviz` configuration is robot-agnostic and is used by all single-arm robot configurations.

- _SRDF Configuration (.srdf)_: The Semantic Robot Description Format (SRDF) complements the URDF by defining the robot’s semantic information, such as self-collision matrices, kinematic groups, and predefined poses. This information is essential for MoveIt!2 to perform inverse kinematics, motion planning, and collision checking. _Example_: irb120.srdf specifies which links are allowed to collide and provides the semantic data required for planning with the ABB IRB-120.

__NOTE__: In ros2srrc, every robot or robot + end-effector combination requires an SRDF file, but the RViz configuration is shared. Multi-arm launch files generate temporary RViz files from the shared configuration so that each RViz instance can use the correct robot namespace and planning group.
