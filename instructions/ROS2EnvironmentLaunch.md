# IFRA-Cranfield: ROS2 Sim-to-Real Robot Control

## ROS 2 Environment Launch: Instructions

The ros2srrc_launch package contains various ROS 2 launch files that execute different robot environments for simulation, testing, and real robot control. Below are the commands to launch these environments depending on the purpose:

__Gazebo Simulation Environment (simulation.launch.py)__

Launches a basic Gazebo simulation of the robot with a pre-configured setup (such as the robot's cell and end-effector). This environment is ideal for verifying the CAD models and the correctness of the built simulation world.

```sh
ros2 launch ros2srrc_launch simulation.launch.py package:=<PACKAGE_NAME> config:=<CONFIG_NAME>
```

__Gazebo Simulation Environment + MoveIt!2 Framework (moveit2.launch.py)__

This command launches the Gazebo simulation along with the MoveIt!2 framework, enabling the robot to be controlled, monitored, and operated through MoveIt!2. It also loads RVIZ for visualization and gives access to the custom ROS 2 tools (/Move, /RobMove, /RobPose) for robot manipulation and monitoring.

```sh
ros2 launch ros2srrc_launch moveit2.launch.py package:=<PACKAGE_NAME> config:=<CONFIG_NAME>
```

__Real Robot Bringup + MoveIt!2 Framework (bringup.launch.py)__

This launches the ROS 2 driver for controlling a physical robot using the MoveIt!2 framework, along with the ROS 2tools (/Move, /RobMove, /RobPose). The IP address of the robot is required to connect to it. The specific launch file varies depending on the type of robot (e.g., ABB or UR).

```sh
ros2 launch ros2srrc_launch bringup.launch.py package:=<PACKAGE_NAME> config:=<CONFIG_NAME> robot_ip:=<ROBOT_IP>
```

## EXAMPLES: Launch default ROS 2 Environments in ros2_SimRealRobotControl

```sh
# ABB IRB-120 Robot (simulation):
ros2 launch ros2srrc_launch simulation.launch.py package:=ros2srrc_irb120 config:=irb120_1

# ABB IRB-1200 Robot w/Schunk EGP-64 Parallel Gripper (moveit2):
ros2 launch ros2srrc_launch moveit2.launch.py package:=ros2srrc_irb1200 config:=irb120_2

# UR3 Robot w/Robotiq Hand-E Gripper (bringup):
ros2 launch ros2srrc_launch bringup.launch.py package:=ros2srrc_ur3 config:=ur3_3 robot_ip:=0.0.0.0
```

## Connecting to Real Robots in ros2_SimRealRobotControl

(THIS SECTION WILL BE INCLUDED SOON)

We will explain how to connect to:
- UR Robots, either to the Physical Robot or through URSim.
- ABB Robots, either to the Physical Robot or through ABB Robot Studio.