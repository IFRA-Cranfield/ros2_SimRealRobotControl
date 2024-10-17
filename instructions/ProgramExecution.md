# IFRA-Cranfield: ROS2 Sim-to-Real Robot Control

## Program Execution: Instructions

This guide provides detailed instructions on how to execute static sequences (programs) using the ExecuteProgram.py script.

### ExecuteProgram.py

__SCRIPT INFORMATION__

ExecuteProgram.py is a Python script designed to automate the execution of static robotic programs within our ROS 2 environment. It is responsible for interpreting and executing predefined sequences of robot movements or actions stored in a __.yaml__ file located within a specific ROS 2 package. These programs, written in YAML format, define a series of sequential steps to control robotic joints, grippers, or any other components involved in a given task.

Upon invocation, the script reads the specified program file (e.g., PROGRAM_NAME.yaml), __which must be located in the /programs folder of any ROS 2 package__. Each step of the program outlines a particular action, such as moving joints to a specific position (MoveJ), rotating a joint by a defined angle (MoveR), or translating the robot's end-effector in space (MoveL). The script processes these commands in the order they are listed, executing the movement at the given speed, applying any necessary delays, and controlling other components like grippers or external devices, based on the provided inputs.

ExecuteProgram.py also uses the "Specifications" section of the YAML file to ensure it is configured correctly to control the designated robot, end-effector, and any attached objects. It selects the appropriate Python clients to control these components based on the robot and other hardware specifications. In essence, the script acts as a central execution engine, translating high-level descriptions of robotic tasks into low-level commands that can be executed in real time.

__COMMAND -> PROGRAM EXECUTION__

To execute a program, use the following command:

```sh
ros2 run ros2srrc_execution ExecuteProgram.py package:="PACKAGE_NAME" program:="PROGRAM_NAME"
```

- PACKAGE_NAME: The name of the ROS 2 Package where your program is located (inside the /programs folder).
- PROGRAM_NAME: The name of the program you want to execute (without the .yaml extension).

__PROGRAM STRUCTURE__

The structure of the .yaml file (program) is divided into two main sections:

- _SPECIFICATIONS_ define the essential components required for program execution, including the robot's name, the end-effector (if any), the end-effector link (where objects are attached), and any objects involved during execution. This section ensures that the script knows which hardware and software components are being used.

- _SEQUENCE_ outlines the step-by-step actions to be performed by the robot. Each step contains details such as the movement type (e.g., MoveJ, MoveL), speed, delay, and the specific inputs needed for the action (e.g., joint angles or positional values). The steps are executed in the order they are listed, forming the program's sequence of operations.

### Template File

__templates.yaml__ is a reference file that provides predefined action templates for constructing the sequence of steps in a static program executed by ExecuteProgram.py. It serves as a guide to help users easily define different types of robot movements or end-effector actions in their program’s YAML files. Each action template is customizable, allowing users to modify parameters such as speed, position, or delays to suit their specific tasks.

#### Specifications Definition

```sh
Specifications:
  Robot: ""
  EndEffector: "" 
  EELink: "" 
  Objects: "" 
```

- Robot: Specifies the robot name, e.g. "irb120".
- EndEffector: The name of the end-effector, currently -> "ParallelGripper", "VacuumGripper", "EGP64/ABB", "GPP5010NC/ABB", "vgr/ABB", "RobotiqHandE/UR".
- EELink: The link of the end-effector to which objects are attached, e.g. "EE_egp64".
- Objects: List of objects that can be attached to the end-effector during the program execution, e.g. ["BlueCube", "WhiteCube", "RedCube"].
- None (without "") is the word to be used if there is no end-effector or object involved in the execution.

#### Action Definition

Here's a breakdown of each action template in _templates.yaml_:

_Robot Movements:_

- MoveJ: This template defines a movement where the robot’s joints move to specific target angles (joint1 to joint6,7). It is ideal for precise joint-by-joint movements and is commonly used when moving to specific configurations in joint space.

    ```sh
    - Step: 0
      Name: "Please type the name of your program step here."
      Type: "MoveJ"
      Speed: 1.0
      Delay: 0.0
      Input:
        joint1: 0.0
        joint2: 0.0
        joint3: 0.0
        joint4: 0.0
        joint5: 0.0
        joint6: 0.0
    ```

- MoveR: This template controls the rotation of a single joint by a specified relative value. It allows users to incrementally adjust the position of a specific joint by a certain number of degrees, useful for fine-tuning joint positions.

    ```sh
    - Step: 0
      Name: "Please type the name of your program step here."
      Type: "MoveR"
      Speed: 1.0
      Delay: 0.0
      Input:
        joint: "jointN"
        value: 0.0
    ```

- MoveL: This template allows the robot’s end-effector to move linearly in Cartesian space (along x, y, and z axes). It is used for straight-line movements of the robot’s tool or gripper, which are often critical in tasks such as pick-and-place operations.

    ```sh
    - Step: 0
      Name: "Please type the name of your program step here."
      Type: "MoveL"
      Speed: 1.0
      Delay: 0.0
      Input:
        x: 0.0
        y: 0.0
        z: 0.0
    ```

- MoveROT: This template allows the robot’s end-effector to rotate around its yaw, pitch, and roll axes. It is used for orienting the tool or gripper in space, crucial when tasks require precise orientation changes during operation.

    ```sh
    - Step: 0
      Name: "Please type the name of your program step here."
      Type: "MoveROT"
      Speed: 1.0
      Delay: 0.0
      Input:
        yaw: 0.0
        pitch: 0.0
        roll: 0.0
    ```

- MoveRP: This template allows the robot’s end-effector to rotate around a specific point in Cartesian space, with both translational (x, y, z) and rotational (yaw, pitch, roll) movements. The x, y, and z values represent the point relative to the end-effector where the rotation occurs, and the angle values (yaw, pitch, roll) define the rotation. It is particularly useful for tasks where the robot needs to rotate around a fixed point, such as welding or machining, while adjusting its orientation and position.

    ```sh
    - Step: 0
      Name: "Please type the name of your program step here."
      Type: "MoveRP"
      Speed: 1.0
      Delay: 0.0
      Input:
        x: 0.0
        y: 0.0
        z: 0.0
        yaw: 0.0
        pitch: 0.0
        roll: 0.0
    ```

- RobMove - PTP: This template defines a point-to-point movement where the robot moves its end-effector to a specified target pose. The target pose is defined by the x, y, z Cartesian coordinates, and the quaternion values (qx, qy, qz, qw) for orientation. In a PTP movement, the robot travels directly to the desired end-effector position, typically without concern for the exact path it takes. This type of movement is fast and efficient when only the final pose matters.

    ```sh
    - Step: 0
      Name: "Please type the name of your program step here."
      Type: "RobMove"
      Movement: "PTP"
      Speed: 1.0
      Delay: 0.0
      Input:
        x: 0.0
        y: 0.0
        z: 0.0
        qx: 0.0
        qy: 0.0
        qz: 0.0
        qw: 0.0
    ```

- RobMove - LIN: The linear movement template allows the robot to move its end-effector in a straight line to the desired target pose. Similar to PTP, the target pose is specified by the x, y, z Cartesian coordinates and the orientation using quaternion values (qx, qy, qz, qw). However, in LIN, the robot ensures that the end-effector follows a precise, linear trajectory between the start and end poses, which is crucial for tasks requiring accurate path control, such as welding or painting.

    ```sh
    - Step: 0
      Name: "Please type the name of your program step here."
      Type: "RobMove"
      Movement: "LIN"
      Speed: 1.0
      Delay: 0.0
      Input:
        x: 0.0
        y: 0.0
        z: 0.0
        qx: 0.0
        qy: 0.0
        qz: 0.0
        qw: 0.0
    ```

_End-Effector in Gazebo Simulator:_

- MoveG: This template is designed for parallel gripper control in Gazebo, allowing the user to define a gripper’s closing value as a percentage (0-100). It controls the gripper’s grip strength, typically used in pick-and-place tasks involving object manipulation. This command simply controls the open/close action of the gripper.

    ```sh
    - Step: 0
      Name: "Please type the name of your program step here."
      Type: "MoveG"
      Speed: 1.0
      Delay: 0.0
      Input:
        value: 0.0  # This value represents gripper CLOSING AVERAGE, [0, 100].
    ```

- Parallel Gripper: These templates control the parallel gripper in a Gazebo simulation environment. The OPEN action fully opens the gripper, while the CLOSE action moves the gripper fingers toward each other to grasp an object. If the gripper is activated in a position where an object (defined in the _Specifications_ section) can be grasped, the LinkAttacher plugin in Gazebo is triggered, and the object is securely attached to the gripper, simulating a successful grasp in the virtual environment.

    ```sh
    # Open Gripper:
    - Step: 0 
      Name: "Please type the name of your program step here."
      Type: "ParallelGripper"
      Action: "OPEN"
      Delay: 0.0

    # Close Gripper:
    - Step: 0
      Name: "Please type the name of your program step here."
      Type: "ParallelGripper"
      Action: "CLOSE"
      Value: 0.0
      Delay: 0.0
    ```
- Vacuum Gripper: These templates manage the vacuum gripper in a Gazebo simulation. The ACTIVATE action turns on the vacuum, allowing the gripper to attach to an object, while the DEACTIVATE action releases it. Similar to the parallel gripper, if the vacuum gripper is activated in a position where an object (specified in the _Specifications_ section) is present, the LinkAttacher plugin in Gazebo is engaged, ensuring the object is correctly grasped in the simulation.

    ```sh
    - Step: 0
      Name: "Please type the name of your program step here."
      Type: "VacuumGripper"
      Action: "ACTIVATE" // "DEACTIVATE"
      Delay: 0.0
    ```

_End-Effector in Real Robot (ABB):_

- Schunk EGP-64 Gripper: These templates are specific to the Schunk EGP-64 gripper mounted on an ABB robot. The OPEN and CLOSE actions control the gripper for gripping and releasing objects during operation.

    ```sh
    - Step: 0
      Name: "Please type the name of your program step here."
      Type: "EGP64/ABB"
      Action: "OPEN" // "CLOSE"
      Delay: 0.0
    ```

- Zimmer GPP5010NC Gripper: These templates are for controlling the Zimmer GPP5010NC gripper on an ABB robot. The OPEN and CLOSE actions allow the gripper to either grasp or release an object, facilitating automated object handling tasks.

    ```sh
    - Step: 0
      Name: "Please type the name of your program step here."
      Type: "GPP5010NC/ABB"
      Action: "OPEN" // "CLOSE"
      Delay: 0.0
    ```

- Vacuum Gripper: Similar to the Gazebo vacuum gripper templates, these are for real-world vacuum gripper control on an ABB robot. The ACTIVATE action starts the vacuum for object manipulation, and DEACTIVATE releases the object when necessary.

    ```sh
    - Step: 0
      Name: "Please type the name of your program step here."
      Type: "vgr/ABB"
      Action: "ACTIVATE" // "DEACTIVATE"
      Delay: 0.0
    ```

_End-Effector in Real Robot (UR):_

- Robotiq HandE Gripper: These templates manage the Robotiq Hand-E Gripper mounted on a UR robot. The OPEN and CLOSE actions allow precise control over gripping operations, useful for object manipulation tasks in industrial settings.

    ```sh
    - Step: 0
      Name: "Please type the name of your program step here."
      Type: "RobotiqHandE/UR"
      Action: "OPEN"
      Delay: 0.0
    ``` 