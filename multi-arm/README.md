# IFRA-Cranfield: ROS 2 Sim-to-Real Robot Control

## ros2srrc: Multi-Arm

This folder contains the required launch files and support utilities to launch, manage, and operate a multi-arm robot environment in ROS 2, specifically for ROS 2 Humble in this branch, using Gazebo Classic and MoveIt! 2.

It also includes the __ros2srrc_ma_examples__ ROS 2 package, which contains example multi-arm configurations, their combined URDFs, and the configuration file that defines each example setup.

## Included Packages

- [`ros2srrc_multiarm`](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/tree/humble/multi-arm/ros2srrc_multiarm): multi-arm launch files and helper scripts.
- [`ros2srrc_ma_examples`](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/tree/humble/multi-arm/ros2srrc_ma_examples): example combined URDFs and configuration definitions.

The currently available examples are defined in [`configurations.yaml`](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/blob/humble/multi-arm/ros2srrc_ma_examples/config/configurations.yaml), and the combined URDFs are stored in [`urdf/`](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/tree/humble/multi-arm/ros2srrc_ma_examples/urdf).

## How It Works

This multi-arm support is based on the single-arm `ros2srrc` architecture, with a set of modifications to enable multiple robots in the same environment.

The current dual-arm implementation works as follows:

- A single combined URDF is used to describe the full cell, including all robots, stands, and end-effectors.
- A single `ros2_control` setup is used for the whole environment, with prefixes to differentiate the joints, links, controllers, and interfaces of each robot.
- One namespaced `move_group` instance is launched for each robot.
- One namespaced `/Move`, `/Robmove`, and `/Robpose` interface is launched for each robot.
- One RViz instance is launched per robot.

This structure allows each arm to be controlled independently, while still sharing the same global robot description and collision environment. It also allows simultaneous control of multiple robots.

## Current Status

Dual-arm support has been implemented.

Tri-arm and more general multi-arm support will be implemented in the future.

Program execution by sequence has not been implemented yet for the multi-arm environment. It will be added in the future and will support multi-arm operation.

## Launch

Launch a dual-arm Gazebo-only simulation:

```bash
ros2 launch ros2srrc_multiarm dualarm_simulation.launch.py package:=ros2srrc_ma_examples config:=multiarm_1
```

Launch a multi-arm MoveIt! 2 + Gazebo Classic environment:

```bash
ros2 launch ros2srrc_multiarm dualarm_moveit2.launch.py package:=ros2srrc_ma_examples config:=multiarm_1
```

Launch a dual-arm real-robot bringup environment:

```bash
ros2 launch ros2srrc_multiarm dualarm_bringup.launch.py package:=ros2srrc_ma_examples config:=multiarm_1
```

Other example configurations currently available are:

```bash
ros2 launch ros2srrc_multiarm dualarm_moveit2.launch.py package:=ros2srrc_ma_examples config:=multiarm_2
ros2 launch ros2srrc_multiarm dualarm_moveit2.launch.py package:=ros2srrc_ma_examples config:=multiarm_3
```

## Interfaces

Each robot gets its own namespaced interfaces. For example, in a dual-arm setup with `rob1` and `rob2`:

- `/rob1/Move` and `/rob2/Move`
- `/rob1/Robmove` and `/rob2/Robmove`
- `/rob1/Robpose` and `/rob2/Robpose`

### /Move

`/Move` executes robot or end-effector actions such as `MoveJ`, `MoveR`, `MoveL`, `MoveROT`, `MoveRP`, and `MoveG`.

Example `MoveJ` command for robot 1:

```bash
ros2 action send_goal /rob1/Move ros2srrc_data/action/Move "{action: MoveJ, speed: 1.0, movej: {joint1: 0.0, joint2: -90.0, joint3: 90.0, joint4: 0.0, joint5: 90.0, joint6: 0.0}}"
```

Example `MoveR` command for robot 2:

```bash
ros2 action send_goal /rob2/Move ros2srrc_data/action/Move "{action: MoveR, speed: 1.0, mover: {joint: joint1, value: 45.0}}"
```

### /Robmove

`/Robmove` moves the robot TCP to a desired pose using either `PTP` or `LIN`.

Example `PTP` command for robot 1:

```bash
ros2 action send_goal /rob1/Robmove ros2srrc_data/action/Robmove "{type: PTP, speed: 1.0, x: 0.0, y: 0.25, z: 0.95, qx: 0.0, qy: 1.0, qz: 0.0, qw: 0.0}"
```

Example `LIN` command for robot 2:

```bash
ros2 action send_goal /rob2/Robmove ros2srrc_data/action/Robmove "{type: LIN, speed: 0.1, x: 0.0, y: 0.25, z: 0.86, qx: 0.0, qy: 1.0, qz: 0.0, qw: 0.0}"
```

### /Robpose

`/Robpose` publishes the current robot TCP pose as a ROS 2 topic.

Echo robot 1 pose:

```bash
ros2 topic echo /rob1/Robpose
```

Echo robot 2 pose:

```bash
ros2 topic echo /rob2/Robpose
```
