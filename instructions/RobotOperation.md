# IFRA-Cranfield: ROS 2 Sim-to-Real Robot Control

## Robot Operation: Instructions

This document provides detailed instructions for operating a robot in the ROS 2 Sim-to-Real Robot Control framework. It also provides information about the usage of additional features and tools for both Gazebo simulation and real robot control.

### ROBOT MOVEMENT

Robot movements in the ROS 2 Sim-to-Real Robot Control framework are controlled via specific ROS 2 actions. The two main ROS 2 actions for movement are __/Move__ and __/Robmove__.

__/Move ROS 2 Action__

The /Move action allows you to execute various robot motion commands based on specific movement types and parameters such as speed, joint positions, Cartesian paths, and rotations.

Robot movements are executed from a single ROS 2 node in ros2_SimRealRobotControl. A robot motion request consists of a simple ROS 2 action (/Move) call, where the following parameters must be specified:
- The ACTION that is going to be executed.
- The speed at which the robot will execute the action.
- The value of the action to be executed.

Actions can be executed by running the following commands in the Ubuntu Terminal:

* MoveJ: The robot moves to the specific waypoint, which is specified by joint pose values.
  ```sh
  ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveJ', movej: {joint1: 0.00, joint2: 0.00, joint3: 0.00, joint4: 0.00, joint5: 0.00, joint6: 0.00}, speed: 1.0}"  # (6-DOF)
  ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveJ', movej: {joint1: 0.00, joint2: 0.00, joint3: 0.00, joint4: 0.00, joint5: 0.00, joint6: 0.00, joint7: 0.0}, speed: 1.0}" # (7-DOF)
  ```

* MoveL: The robot executes a Cartesian/linear path. The end-effector orientation is kept constant, and the position changes by +/-(x,y,z).
  ```sh
  ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveL', movel: {x: 0.00, y: 0.00, z: 0.00}, speed: 1.0}"
  ```
* MoveR: The robot rotates the selected joint by a specific number of degrees.
  ```sh
  ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveR', mover: {joint: '--', value: 0.00}, speed: 1.0}"
  ```
* MoveROT: The robot rotates/orientates the end-effector frame according to the input Euler angles (yaw,pitch,roll). The ROT(yaw,pitch,roll) input determines the added rotation of the end-effector, which is applied to the end-effector coordinate frame.
  ```sh
  ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveROT', moverot: {yaw: 0.00, pitch: 0.00, roll: 0.00}, speed: 1.0}"
  ```
* MoveRP: End-effector rotation around a point -> The robot rotates/orientates and moves the end-effector frame according to the input Euler angles (yaw,pitch,roll) and point (x,y,z). The ROT(yaw,pitch,roll) input determines the added rotation of the end-effector, which is applied to the end-effector coordinate frame around the (x,y,z) point.
  ```sh
  ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveRP', moverp: {x: 0.00, y: 0.00, z: 0.00, yaw: 0.00, pitch: 0.00, roll: 0.00}, speed: 1.0}"
  ```
* MoveG: The gripper fingers move to the specific pose. The "moveg" value is between 0-100 and represents the gripper opening percentage.
  ```sh
  ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveG', moveg: 0.0, speed: 1.0}"
  ```
* NOTE: The robot joint speed is controlled by the "speed" parameter when executing the specific ROS 2 action. The value must be (0,1], with 1 being the maximum velocity and 0 being the null velocity, which is not valid. A small value must be defined, e.g. 0.01 represents a very slow movement.

__/Robmove ROS 2 Action__

The /Robmove action is used to move the robot’s end-effector to a specific __end-effector pose__. It allows for two types of movement:

- PTP (Point-to-Point): The robot moves directly to the target pose via an optimal path in joint space.
- LIN (Linear): The robot moves in a straight line between its current pose and the target pose.

To execute a /Robmove action, the following parameters need to be defined:
- The TYPE of movement: It can be Point-to-Point ("PTP"), or LINEAR ("LIN").
- The speed at which the robot will execute the action.
- The POSE, (POSITION - x,y,z + ROTATION - qx,qy,qz,qw).

/Robmove can be executed by running the following command in the Ubuntu Terminal:

```sh
ros2 action send_goal -f /Robmove ros2srrc_data/action/Robmove "{type: '---', speed: 1.0, x: 0.0, y: 0.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 0.0}"
```

It is recommended to combine /Robmove with /Robpose (ROS 2 topic, see below). This ROS 2 topic publishes the current real-time pose of the robot's end-effector, which helps the user define the robot's next pose.

### MONITOR ROBOT STATE

Monitoring the robot’s state and pose is crucial for real-time feedback and system control. There are two main tools for this:

- RobotState.py script (joint values).
- /Robpose ROS 2 topic (end-effector pose).

__RobotState.py script__

The __RobotState.py__ script allows the user to get the state of the robot in __joint values__, by simply executing the following command:

```sh
ros2 run ros2srrc_execution RobotState.py
```

__Robpose ROS 2 Topic__

The /Robpose topic publishes the real-time pose of the robot’s end-effector in terms of position and orientation. You can monitor the current pose by subscribing to this topic:

```sh
ros2 topic echo /Robpose
```

### EXTRA: Spawn Object to a Gazebo Environment

The SpawnObject.py script allows users to spawn objects into the Gazebo simulation environment. The objects must be defined in a .urdf file and placed in the appropriate package folder (/urdf/objects folder inside the specified package).

```sh
ros2 run ros2srrc_execution SpawnObject.py --package "{}" --urdf "{}.urdf" --name "{}" --x {} --y {} --z {}
```
