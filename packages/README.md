# IFRA-Cranfield: ROS2 Sim-to-Real Robot Control

## Robot Simulation & Control: Standard ROS 2 Packages for different Robot Manipulators

The /packages folder contains standard ROS 2 packages for each robot arm included in ros2_SimRealRobotControl. These packages provide simple, raw simulation environments where a robot (and end-effector, if applicable) is placed on top of a basic robot stand.

The main purpose of these packages is to serve as reference examples for building your own robot cells. By following this structure, you can quickly create a new ROS 2 package for any robot + end-effector combination by adding:

- A _configuration_ file that defines the different layouts and configurations of your robot cell.
- The robot cell’s URDF files, together with CAD files of the cell or any relevant objects.

This modular approach is possible because all robot- and end-effector-specific data is already centralized in the top-level /robots and /endeffectors folders of this repository. Whenever you design a new robot cell, you simply reference these standardized definitions—without duplicating data or parameters.

If you are looking for more advanced examples that go beyond the simple robot-on-stand setup (e.g., full cell CAD models or specific applications), you can explore:

- https://github.com/IFRA-Cranfield/irb120_CranfieldRobotics
- https://github.com/IFRA-Cranfield/ur3_CranfieldRobotics

</br>

---

ROS 2 Package folder structure:

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

__NOTE: Controller Parameters, Key Specifications and MoveIt!2 Config Files__

One important feature of the ros2srrc repository is its modular architecture, which minimizes duplication and redundancy in robot setup. All the controller parameters and key specifications specific to the robot or end-effector (such as joint limits, PID gains, velocity/position controllers, etc.) are already predefined in the main robot and end-effector folders within the repository. These predefined configurations ensure that:

- Robot-specific parameters: Properties such as joint limits, inertia, and dynamics for robots like the ABB IRB-120 are already set up.
- End-effector-specific parameters: Specifications such as gripper dimensions, actuation constraints, and grasping capabilities for end effectors like the Schunk EGP-64 Gripper are also included.

In addition, the _.rviz_ and _.srdf_ files required for MoveIt!2 configuration have been standardised for every robot+end-effector combination, and included inside the ros2srrc_moveit package.

Features of this modular setup:

- No redefinition is needed: These parameters do not need to be redefined or replicated in the ROS 2 Gazebo or MoveIt!2 packages. The packages simply reference the existing parameters.
- Centralized control: Any updates or changes to the robot or end-effector specifications can be handled in their respective folders without affecting the overall system configuration, maintaining consistency and ease of use across various ROS 2 packages.

This modular approach reduces complexity and makes the system highly maintainable, as different components (robot and end-effector) are managed independently but integrated seamlessly.

## ROS 2 Packages available in IFRA-Cranfield/ros2_SimRealRobotControl

List of all available (standard) ROS 2 Packages and configurations:

__ABB IRB-120__

Package name: ros2srrc_irb120

Configurations:

- irb120_1: ABB IRB-120 on top of Robot Stand.
- irb120_2: ABB IRB-120 + Schunk EGP-64 Gripper on top of Robot Stand.
- irb120_21: ABB IRB-120 + Schunk EGP-64 Gripper (rounded fingers) on top of Robot Stand.
- irb120_3: ABB IRB-120 + Lamination Sheet Vacuum-Gripper on top of Robot Stand.

__ABB IRB-1200__

Package name: ros2srrc_irb1200

Configurations:

- irb1200_1: ABB IRB-1200 on top of Robot Stand.
- irb1200_2: ABB IRB-1200 + Schunk EGP-64 Gripper on top of Robot Stand.

__ABB IRB-1600__

Package name: ros2srrc_irb1600

Configurations:

- irb1600_1: ABB IRB-1600 on top of Robot Stand.

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

__Universal Robots UR3e__

Package name: ros2srrc_ur3e

Configurations:

- ur3e_1: UR3e on top of Robot Stand.
- ur3e_2: UR3e + Robotiq 2f-85 gripper on top of Robot Stand.
- ur3e_3: UR3e + Robotiq HandE gripper on top of Robot Stand.

__Universal Robots UR5__

Package name: ros2srrc_ur5

Configurations:

- ur5_1: UR5 on top of Robot Stand.
- ur5_2: UR5 + Robotiq 2f-85 gripper on top of Robot Stand.
- ur5_3: UR5 + Robotiq HandE gripper on top of Robot Stand.

__Universal Robots UR5e__

Package name: ros2srrc_ur5e

Configurations:

- ur5e_1: UR5e on top of Robot Stand.
- ur5e_2: UR5e + Robotiq 2f-85 gripper on top of Robot Stand.
- ur5e_3: UR5e + Robotiq HandE gripper on top of Robot Stand.

__Universal Robots UR10__

Package name: ros2srrc_ur10

Configurations:

- ur10_1: UR10 on top of Robot Stand.
- ur10_2: UR10 + Robotiq 2f-85 gripper on top of Robot Stand.
- ur10_3: UR10 + Robotiq HandE gripper on top of Robot Stand.

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

__Universal Robots UR20__

Package name: ros2srrc_ur20

Configurations:

- ur20_1: UR20 on top of Robot Stand.
- ur20_2: UR20 + Robotiq 2f-85 gripper on top of Robot Stand.
- ur20_3: UR20 + Robotiq HandE gripper on top of Robot Stand.

__KUKA LBR-iiwa__

Package name: ros2srrc_iiwa

Configurations:

- iiwa_1: KUKA LBR-iiwa on top of Robot Stand.
- iiwa_2: KUKA LBR-iiwa + Robotiq 2f-85 gripper on top of Robot Stand.

__Staubli RX-160__

Package name: ros2srrc_rx160

Configurations:

- rx160_1: Staubli RX-160 on top of Robot Stand.