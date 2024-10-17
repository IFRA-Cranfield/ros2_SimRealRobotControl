# IFRA-Cranfield: ROS2 Sim-to-Real Robot Control

## Robot Simulation & Control: ROS 2 Packages

In this document we will explain how the Robot Simulation & Control ROS 2 Packages are structured in our ros2srrc repository, and we will list all the available robot & end-effector configurations we have.

### ROS 2 PACKAGE STRUCTURE

To integrate a new robot into the ros2srrc repository, two specific ROS 2 packages are required:

- Gazebo Simulation package.
- MoveIt!2 package.

These packages ensure seamless simulation and control of the robot, following the standards established for modularity, reusability, and ease of integration. Using the ABB IRB-120 robot as an example, this section will explain the key components and structure of each package.

__Gazebo Simulation Package__

This package is responsible for housing the robot's CAD files and URDF models. The package includes several folders and files necessary for setting up robot simulation.

Folder structure:

- _/config_: Contains the configurations.yaml file, which defines various robot/cell configurations. This file allows users to switch between different setups by defining a set of parameters for each configuration (e.g., robot type, end effector, and URDF file). The launch files refer to this YAML file to load the required configurations. Example:

    ```sh
    Configurations:

        - ID: "irb120_1"
        Name: "ABB IRB-120 on top of Robot Stand."
        urdf: "irb120.urdf.xacro"
        rob: "irb120"
        ee: "none"

        - ID: "irb120_2"
        Name: "ABB IRB-120 + Schunk EGP-64 Gripper on top of Robot Stand."
        urdf: "irb120_egp64.urdf.xacro"
        rob: "irb120"
        ee: "egp64"
    ```

    In this example, each configuration defines:

    - ID: Unique identifier for the configuration.
    - Name: Descriptive name of the robot setup.
    - urdf: Path to the robot's URDF file.
    - rob: Name of the robot.
    - ee: End effector (if applicable).

- _/urdf_: Contains the URDF (Unified Robot Description Format) files. These files describe the robot's physical properties (joints, links, dimensions) and include references to the robot and end effector models. The URDF defines the geometry, sensors, and controllers of the robot in the simulation environment. Examples:
    
    - irb120.urdf.xacro: URDF for the base robot.
    - irb120_egp64.urdf.xacro: URDF for the robot with the Schunk EGP-64 gripper.

    In these URDF files, the standard urdf files of both the robot and the end-effector (located within the __/robots__ and __/endeffectors__ folders of this repository) are loaded and linked. This approach allows the usage of single URDF files for raw robots and end-effectors in multiple configurations, enhancing scalability, modularity and reusability.

- _/worlds_: Contains the Gazebo world file that specifies the environment in which the robot will be simulated. This file defines objects, lighting, and the physics properties of the simulation environment.

__MoveIt!2 Package__

The MoveIt!2 package manages the kinematic control, motion planning, and collision detection for the robot. This package includes the necessary configurations to operate the robot in MoveIt!2. The package has a single _/config_ folder, which contains several important configuration files that allow MoveIt!2 to communicate with the robot and control its movements in simulation and real-world applications: 

- _RVIZ Configuration (.rviz)_: This file defines the visual representation of the robot in RVIZ, the ROS 2 visualization tool. It specifies the robot model, display parameters, and any pre-configured markers or interactive controls. RVIZ is used to visualize the robot's state and motion planning. Example: irb120egp64_moveit2.rviz contains the settings to display the IRB-120 robot and the associated end effector in RVIZ, providing an intuitive interface for motion planning.

- _SRDF Configuration (.srdf)_: The Semantic Robot Description Format (SRDF) file contains information about the robot's kinematic properties, self-collision matrices, and predefined poses (if any). This file complements the URDF and is required by MoveIt!2 to perform kinematic calculations and collision avoidance. Example: irb120_moveit2.srdf defines which parts of the robot are allowed to collide and describes joints and actuators for inverse kinematics and motion planning.

__NOTE: Controller Parameters and Key Specifications__

One important feature of the ros2srrc repository is its modular architecture, which minimizes duplication and redundancy in robot setup. All the controller parameters and key specifications specific to the robot or end-effector (such as joint limits, PID gains, velocity/position controllers, etc.) are already predefined in the main robot and end-effector folders within the repository. These predefined configurations ensure that:

- Robot-specific parameters: Properties such as joint limits, inertia, and dynamics for robots like the ABB IRB-120 are already set up.
- End-effector-specific parameters: Specifications such as gripper dimensions, actuation constraints, and grasping capabilities for end effectors like the Schunk EGP-64 Gripper are also included.

Features of this modular setup:

- No redefinition is needed: These parameters do not need to be redefined or replicated in the ROS 2 Gazebo or MoveIt!2 packages. The packages simply reference the existing parameters.
- Centralized control: Any updates or changes to the robot or end-effector specifications can be handled in their respective folders without affecting the overall system configuration, maintaining consistency and ease of use across various ROS 2 packages.

This modular approach reduces complexity and makes the system highly maintainable, as different components (robot and end-effector) are managed independently but integrated seamlessly.

### ROS 2 Packages available in IFRA-Cranfield/ros2_SimRealRobotControl

List of all available (standard) ROS 2 Packages and configurations:

__ABB IRB-120__

Package name: ros2srrc_irb120

Configurations:

- irb120_1: ABB IRB-120 on top of Robot Stand.
- irb120_2: ABB IRB-120 + Schunk EGP-64 Gripper on top of Robot Stand.
- irb120_3: ABB IRB-120 + Schunk EGP-64 Gripper (rounded fingers) on top of Robot Stand.
- irb120_4: ABB IRB-120 + Lamination Sheet Vacuum-Gripper on top of Robot Stand.

__ABB IRB-1200__

Package name: ros2srrc_irb1200

Configurations:

- irb1200_1: ABB IRB-1200 on top of Robot Stand.
- irb1200_2: ABB IRB-1200 + Schunk EGP-64 Gripper on top of Robot Stand.

__ABB IRB-6640__

Package name: ros2srrc_irb6640

Configurations:

- irb6640_1: ABB IRB-6640 on top of Robot Stand.
- irb6640_2: ABB IRB-6640 + Zimmer GP5010NC-00-A Parallel Gripper on top of Robot Stand.

__Universal Robots UR3__

Package name: ros2srrc_ur3

Configurations:

- ur3_1: UR3 on top of Robot Stand.
- ur3_2: UR3 + Robotiq 2f-85 gripper on top of Robot Stand.
- ur3_3: UR3 + Robotiq HandE gripper on top of Robot Stand.

__Universal Robots UR5__

Package name: ros2srrc_ur5

Configurations:

- ur5_1: UR5 on top of Robot Stand.
- ur5_2: UR5 + Robotiq 2f-85 gripper on top of Robot Stand.
- ur5_3: UR5 + Robotiq HandE gripper on top of Robot Stand.

__Universal Robots UR10e__

Package name: ros2srrc_ur10e

Configurations:

- ur10e_1: UR10e on top of Robot Stand.
- ur10e_2: UR10e + Robotiq 2f-85 gripper on top of Robot Stand.
- ur10e_3: UR10e + Robotiq HandE gripper on top of Robot Stand.

__Universal Robots UR16e__

Package name: ros2srrc_ur16e

Configurations:

- ur16e_1: UR16e on top of Robot Stand.
- ur16e_2: UR16e + Robotiq 2f-85 gripper on top of Robot Stand.
- ur16e_3: UR16e + Robotiq HandE gripper on top of Robot Stand.

__KUKA LBR-iiwa__

Package name: ros2srrc_iiwa

Configurations:

- iiwa_1: KUKA LBR-iiwa on top of Robot Stand.
- iiwa_2: KUKA LBR-iiwa + Robotiq 2f-85 gripper on top of Robot Stand.