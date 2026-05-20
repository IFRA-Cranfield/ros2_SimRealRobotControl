# IFRA-Cranfield: ROS 2 Sim-to-Real Robot Control

## Gazebo Classic to Gazebo Fortress / GZ Sim Migration

This branch uses Gazebo Fortress / GZ Sim with ROS 2 Humble. The notes below summarize the migration rules followed in this repository when moving the simulation stack away from Gazebo Classic.

### Package and Dependency Changes

- Gazebo Classic launch and bridge dependencies have been replaced with the ROS-GZ stack.
- Simulation launch files use `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image`, and `ros_gz_interfaces`.
- Robot control uses `gz_ros2_control` instead of `gazebo_ros2_control`.
- Gazebo Fortress worlds are stored in the `ros2srrc_gz` package.

### Launch File Changes

- Simulation launch files start Gazebo Fortress through `ros_gz_sim/launch/gz_sim.launch.py`.
- Robot spawning is performed through the `/world/ros2srrc_GzWorld/create` service with `ros_gz_interfaces/srv/SpawnEntity`.
- Camera and image topics are bridged with ROS-GZ bridge/image nodes.
- MoveIt!2 launch files keep the same robot configuration flow as the Gazebo Classic branch, but point to the Gazebo Fortress / GZ Sim launch, spawn, and bridge interfaces.

### Robot and End-Effector Description Changes

- URDF/Xacro files use the Gazebo Fortress `gz_ros2_control` plugin.
- Control plugin filenames, parameter blocks, and namespaces have been updated for the GZ Sim runtime.
- Meshes, inertias, joint limits, transmissions, and MoveIt!2 configuration files should remain equivalent to the Gazebo Classic branch unless the Fortress runtime requires a compatibility tweak.

### Object Spawning

- Dynamic object spawning uses SDF files instead of the Gazebo Classic URDF object flow.
- `SpawnObject.py` expects objects under an `/sdf` folder inside the selected package and sends them to the Gazebo Fortress spawn service.

### Naming Conventions

- User-facing documentation should refer to the simulator as `Gazebo Fortress / GZ Sim`.
- ROS concepts should use `ROS 2`, `ROS 2 Control`, and `ROS 2 controllers`.
- MoveIt documentation and launch comments should use `MoveIt!2`.
- RViz should be written as `RViz`.
