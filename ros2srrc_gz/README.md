# IFRA-Cranfield: ROS 2 Sim-to-Real Robot Control

## ros2srrc_gz package

The ros2srrc_gz package contains the standard Gazebo Harmonic / GZ Sim world files (.sdf) used in the ros2_SimRealRobotControl framework. By centralizing these environments in a single package, robot cell packages can reference them directly without needing to duplicate world files, ensuring consistency, modularity, and easier maintenance across the repository.

A Gazebo / GZ Sim world file defines the simulation environment in which robots operate. It specifies elements such as the ground plane, lights, physics settings, and any static objects that make up the scene where the robot is placed.
