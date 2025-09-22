# IFRA-Cranfield: ROS2 Sim-to-Real Robot Control

## ros2srrc-gazebo package

The ros2srrc_gazebo package contains all the standard Gazebo world files (.world) used in the ros2_SimRealRobotControl framework. By centralizing these environments in a single package, robot cell packages can reference them directly without needing to duplicate world files, ensuring consistency, modularity, and easier maintenance across the repository.

A Gazebo world file (.world) defines the simulation environment in which robots operate. It specifies elements such as the ground plane, lights, physics settings, and any static objects (like tables or stands) that make up the scene where the robot is placed.