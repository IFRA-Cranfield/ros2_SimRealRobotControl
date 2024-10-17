# IFRA-Cranfield: ROS2 Sim-to-Real Robot Control

## Installation Steps

The steps below must be followed in order to properly set-up a ROS 2 Humble machine which is needed for the usage of the ROS 2 Packages in the _ROS 2 Sim-to-Real Robot Control_ repository. It is recommended to install Ubuntu 22.04 Desktop on your PC for an optimal performance, but a VM could be used for simple simulations and executions.

__PC Set-Up for Robot Simulation and Control in ROS2__

1. Install Ubuntu 22.04: https://ubuntu.com/desktop

2. Install Git:

    ```sh
    # In the terminal shell:
    sudo apt install git

    # Git account configuration:
    git config --global user.name YourUsername
    git config --global user.email YourEmail
    git config --global color.ui true
    git config --global core.editor code --wait # Visual Studio Code is recommended.
    git config --global credential.helper store
    ```

3. Install ROS2 Humble:
    - Follow instructions in: [ROS2 Humble Tutorials - Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
    - Source the ROS2.0 Humble installation in the .bashrc file (hidden file in /home):
        ```sh
        source /opt/ros/humble/setup.bash
        ```

4. Install MoveIt!2 for ROS2 Humble ([REF: MoveIt!2 Website](https://moveit.picknik.ai/humble/index.html)):

    ```sh
    # Command for BINARY INSTALL (recommended):
    sudo apt install ros-humble-moveit
    ```

5. Modify the move_group_interface.h script: A small improvement of the move_group_interface.h file has been developed in order to execute the Robot/Gripper triggers in this repository. Both the upgraded file and the instructions of how to implement it can be found here: [move_group_interface_improved.h](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/tree/humble/include)

6. Create and configure the ROS2.0 Humble ~/dev_ws environment/workspace:
    - Follow instructions in: [ROS2 Humble Tutorials - Create a ROS2 Workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).
    - Source the ~/dev_ws workspace in .bashrc file:
        ```sh
        source ~/dev_ws/install/local_setup.bash
        ```

7. Install ROS2 packages, which are required for ROS2-based Robot Simulation and Control:

    ```sh
    # Rosdep, vcstool and colcon:
    sudo apt install python3-rosdep
    sudo apt install python3-vcstool
    sudo apt install python3-colcon-common-extensions

    # ROS2 Control + ROS2 Controllers:
    sudo apt install ros-humble-ros2-control
    sudo apt install ros-humble-ros2-controllers
    sudo apt install ros-humble-gripper-controllers

    # Gazebo for ROS2 Humble:
    sudo apt install gazebo
    sudo apt install ros-humble-gazebo-ros2-control
    sudo apt install ros-humble-gazebo-ros-pkgs

    # xacro:
    sudo apt install ros-humble-xacro

    # Install CycloneDDS RMW for ROS 2 Humble to fix cycle time issues in humble-moveit (temporary fix):
    sudo apt install ros-humble-rmw-cyclonedds-cpp 
    # Add the following statement into .bashrc file: 
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ```   

    (EXTRA STEP) -> Due to problems with URDF file processing for the newest version of ROS 2 Control-Gazebo plugin, Gazebo-ROS2-Control must be downgraded to the 0.4.6 version:

    ```sh
    # Uninstall Gazebo ROS2 Control:
    sudo apt remove ros-humble-gazebo-ros2-control

    # Download and install the 0.4.6 version:
    cd ~/dev_ws/src
    git clone https://github.com/ros-controls/gazebo_ros2_control.git
    cd gazebo_ros2_control
    git reset --hard 9a3736c # Commit for the 0.4.6 version!
    cd ~/dev_ws
    colcon build
    ``` 

__Download and install the required ROS 2 Packages for the Simulation and Control of Robot Arms__

1. __ABB DRIVER for ROS2__: The installation of the [abb_ros2](https://github.com/PickNikRobotics/abb_ros2) driver is required for the control of any real ABB robot using ROS 2.

    ```sh
    mkdir -p ~/dev_ws/src/ABBDriver
    cd ~/dev_ws/src/ABBDriver
    git clone https://github.com/PickNikRobotics/abb_ros2.git -b rolling
    sudo rosdep init
    rosdep update
    vcs import < abb_ros2/abb.repos
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
    cd ~/dev_ws
    colcon build
    ```

2. __Universal Robots ROS2 Driver__: The installation of the [ur-robot-driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) is required for the control of any real UR robot using ROS 2. Binary install, for ROS2 Humble:

    ```sh
    sudo apt-get install ros-humble-ur
    ```

3. Import and install the following ROS2 Packages developed by IFRA-Cranfield:

    ```sh
    # IFRA-Cranfield/IFRA_LinkAttacher:
    cd ~/dev_ws/src
    git clone https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.git
    cd ~/dev_ws
    colcon build
    
    # IFRA-Cranfield/IFRA_ObjectPose:
    cd ~/dev_ws/src
    git clone https://github.com/IFRA-Cranfield/IFRA_ObjectPose.git
    cd ~/dev_ws
    colcon build

    # IFRA-Cranfield/IFRA_LinkPose:
    cd ~/dev_ws/src
    git clone https://github.com/IFRA-Cranfield/IFRA_LinkPose.git
    cd ~/dev_ws
    colcon build

    # IFRA-Cranfield/ros2_RobotiqGripper:
    cd ~/dev_ws/src
    git clone https://github.com/IFRA-Cranfield/ros2_RobotiqGripper.git
    cd ~/dev_ws
    colcon build
    ```

__Download and install ros2_SimRealRobotControl__

```sh
cd ~/dev_ws/src
git clone https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl
cd ~/dev_ws
colcon build
```   