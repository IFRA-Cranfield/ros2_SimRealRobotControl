## ROS2 Sim-to-Real Robot Control: ros2srrc_execution

The ros2srrc_execution package contains all the source code to execute Robot Movements and programs/sequences. Code is mainly in c++ (MoveGroup-related), with a particular .py script that converts .txt data into a ROS2 Action Call for further program/sequence execution.

### Single ROBOT MOVEMENT execution

As explained, Robot Movements are executed from a single ROS 2 Node in ros2_SimRealRobotControl. A Robot Motion request consists of a simple ROS2 Action (/Move) call, where the following parameters must be specified:
- The ACTION that is going to be executed.
- The speed at which the robot will execute the action.
- The value of the action to be executed.

Actions can be executed by running the following commands in the Ubuntu Terminal:

* MoveJ: The Robot moves to the specific waypoint, which is specified by Joint Pose values.
  ```sh
  ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveJ', movej: {joint1: 0.00, joint2: 0.00, joint3: 0.00, joint4: 0.00, joint5: 0.00, joint6: 0.00}, speed: 1.0}"  # (6-DOF)
  ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveJ', movej: {joint1: 0.00, joint2: 0.00, joint3: 0.00, joint4: 0.00, joint5: 0.00, joint6: 0.00, joint7: 0.0}, speed: 1.0}" # (7-DOF)
  ```
* MoveG: The Gripper fingers move to the specific pose.
  ```sh
  ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveG', moveg: 0.0, speed: 1.0}"
  ```
* MoveL: The Robot executes a CARTESIAN/LINEAR path. The End-Effector orientation is kept constant, and the position changes by +-(x,y,z).
  ```sh
  ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveL', movel: {x: 0.00, y: 0.00, z: 0.00}, speed: 1.0}"
  ```
* MoveR: The Robot rotates the selected joint a specific amount of degrees.
  ```sh
  ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveR', mover: {joint: '--', value: 0.00}, speed: 1.0}"
  ```
* MoveXYZW: The Robot moves to the specific waypoint, which is represented by the Position(x,y,z) + EulerAngles(yaw,pitch,roll) End-Effector coordinates.
  ```sh
  ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveXYZW', movexyzw: {x: 0.00, y: 0.00, z: 0.00, yaw: 0.00, pitch: 0.00, roll: 0.00}, speed: 1.0}"
  ```
* MoveXYZ: The Robot moves to the specific waypoint -> Position(x,y,z) maintaining the End-Effector orientation.
  ```sh
  ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveXYZ', movexyz: {x: 0.00, y: 0.00, z: 0.00}, speed: 1.0}"
  ```
* MoveYPR: The Robot rotates/orientates the End-Effector frame according to the input: EulerAngles(yaw,pitch,roll). The YPR(yaw,pitch,roll)determines the FINAL ROTATION of the End-Effector, which is related to the GLOBAL COORDINATE FRAME.
  ```sh
  ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveYPR', moveypr: {yaw: 0.00, pitch: 0.00, roll: 0.00}, speed: 1.0}"
  ```
* MoveROT: The Robot rotates/orientates the End-Effector frame according to the input: EulerAngles(yaw,pitch,roll). THE ROT(yaw,pitch,roll) determines the ADDED ROTATION of the End-Effector, which is applied to the END-EFFECTOR COORDINATE FRAME.
  ```sh
  ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveROT', moverot: {yaw: 0.00, pitch: 0.00, roll: 0.00}, speed: 1.0}"
  ```
* MoveRP: End-Effector rotation AROUND A POINT -> The Robot rotates/orientates + moves the End-Effector frame according to the input: EulerAngles(yaw,pitch,roll) + Point(x,y,z). THE ROT(yaw,pitch,roll) determines the ADDED ROTATION of the End-Effector, which is applied to the END-EFFECTOR COORDINATE FRAME, AROUND THE (x,y,z) POINT.
  ```sh
  ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveRP', moverp: {x: 0.00, y: 0.00, z: 0.00, yaw: 0.00, pitch: 0.00, roll: 0.00}, speed: 1.0}"
  ```
* NOTE: The Robot JOINT SPEED is controlled by the "speed" parameter when executing the specific ROS2.0 action. The value must be (0,1]. being 1 the maximum velocity and 0 the null velocity (which is not valid -> A small value must be defined, e.g.: 0.01 represents a very slow movement).

### PROGRAM/SEQUENCE execution
Programs can be executed by running the following command in the Ubuntu Terminal:
```sh
ros2 run ros2srrc_execution sequence.py --ros-args -p PROGRAM_FILENAME:="---" -p ROBOT_MODEL:="---" -p EE_MODEL:="---" -p GzBr_ENV:="---"
```
* The PROGRAM_FILENAME parameter is the name of the file which contains the program. The program is saved in a .txt file, and the name must be inputted excluding the ".txt" extension.
* The ROBOT_MODEL parameter represents the model of the robot. Options: irb120.
* The EE_MODEL parameter represents the model of the end-effector. Options: egp64 - none.
* The GzBr_ENV parameter defines whether the execution is being done in Gazebo or real robot. Options: gazebo - bringup.

__Pre-defined sequence: Format__

The pre-defined programs are saved inside the /programs folder as .txt files. Every single line of the .txt file represents an execution step (being the 1st line: 1st step, 2nd line: 2nd step, ...), and it is represented as a python dictionary. The following list showcases how every single Robot Movement has to be inputted in the program.txt:
* For MoveJ ---> {'action': 'MoveJ', 'value': {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0, 'joint4': 0.0, 'joint5': 0.0, 'joint6': 0.0}, 'speed': 1.0}
* For MoveL ---> {'action': 'MoveL', 'value': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'speed': 1.0}
* For MoveR ---> {'action': 'MoveR', 'value': {'joint': '---', 'value': 0.0}, 'speed': 1.0}
* For MoveXYZW ---> {'action': 'MoveXYZW', 'value': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}, 'speed': 1.0}
* For MoveXYZ ---> {'action': 'MoveXYZ', 'value': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'speed': 1.0}
* For MoveYPR ---> {'action': 'MoveYPR', 'value': {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}, 'speed': 1.0}
* For MoveROT ---> {'action': 'MoveROT', 'value': {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}, 'speed': 1.0}
* For MoveRP ---> {'action': 'MoveRP', 'value': {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0, 'x': 0.0, 'y': 0.0, 'z': 0.0}, 'speed': 1.0}
* For MoveG (Gazebo) ---> {'action': 'MoveG', 'value': {'value': 0.0}, 'speed': 1.0}
* For MoveG (ABB Robot): 
    * To Open Gripper ---> {'action': 'GripperOpen'}
    * To Close Gripper ---> {'action': 'GripperClose'}
* For the object manipulation (IFRA_LinkAttacher Gazebo Plugin):
    * To attach object to end-effector ---> {'action': 'Attach', 'value': {'model1': '---', 'link1': '---', 'model2': '---', 'link2': '---'}}
    * To detach object from end-effector ---> {'action': 'Attach', 'value': {'model1': '---', 'link1': '---', 'model2': '---', 'link2': '---'}}
    * Elements are defined as follows:
      * model1 -> Name of the model/robot (defined in the robot .urdf). 
      * link1 -> Name of the end-effector link that the object will be attached to.
      * model2 -> Name of the object to be attached (defined in the object .urdf). 
      * link2 -> Name of the object link.

__Example of a simple ROBOT PROGRAM: irb120.txt__
```txt
{'action': 'MoveJ', 'value': {'joint1': 90.0, 'joint2': 0.0, 'joint3': 0.0, 'joint4': 0.0, 'joint5': 0.0, 'joint6': 45.0}, 'speed': 1.0}
{'action': 'MoveJ', 'value': {'joint1': 90.0, 'joint2': -20.0, 'joint3': 40.0, 'joint4': 0.0, 'joint5': -20.0, 'joint6': 45.0}, 'speed': 1.0}
{'action': 'MoveL', 'value': {'x': 0.0, 'y': 0.1, 'z': 0.0}, 'speed': 0.05}
{'action': 'MoveL', 'value': {'x': 0.0, 'y': -0.1, 'z': 0.0}, 'speed': 0.05}
{'action': 'MoveL', 'value': {'x': 0.0, 'y': 0.0, 'z': -0.2}, 'speed': 1.0}
{'action': 'MoveL', 'value': {'x': 0.0, 'y': 0.1, 'z': 0.0}, 'speed': 0.05}
{'action': 'MoveL', 'value': {'x': 0.0, 'y': -0.1, 'z': 0.0}, 'speed': 0.05}
{'action': 'MoveJ', 'value': {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0, 'joint4': 0.0, 'joint5': 0.0, 'joint6': 0.0}, 'speed': 1.0}
{'action': 'MoveROT', 'value': {'pitch': 90.0, 'yaw': 0.0, 'roll': 0.0}, 'speed': 1.0}
{'action': 'MoveL', 'value': {'x': 0.0, 'y': 0.0, 'z': -0.2}, 'speed': 1.0}
{'action': 'MoveL', 'value': {'x': 0.0, 'y': 0.0, 'z': -0.1}, 'speed': 0.05}
{'action': 'MoveL', 'value': {'x': 0.0, 'y': 0.0, 'z': 0.1}, 'speed': 0.05}
{'action': 'MoveR', 'value': {'joint': 'joint1', 'value': -45.0}, 'speed': 1.0}
{'action': 'MoveL', 'value': {'x': 0.0, 'y': 0.0, 'z': -0.1}, 'speed': 0.05}
{'action': 'MoveL', 'value': {'x': 0.0, 'y': 0.0, 'z': 0.1}, 'speed': 0.05}
{'action': 'MoveJ', 'value': {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0, 'joint4': 0.0, 'joint5': 0.0, 'joint6': 0.0}, 'speed': 1.0}
```

__Example of a ROBOT PROGRAM w/ object Pick&Place: ur3cubePP.txt__
```txt
{'action': 'MoveJ', 'value': {'joint1': -9.13615, 'joint2': -117.5559, 'joint3': -66.32366, 'joint4': -86.14282, 'joint5': 90.0091, 'joint6': 35.86377}, 'speed': 1.0}
{'action': 'MoveL', 'value': {'x': 0.0, 'y': 0.0, 'z': -0.1}, 'speed': 0.2}
{'action': 'MoveG', 'value': {'value': 0.4}, 'speed': 1.0}
{'action': 'Attach', 'value': {'model1': 'ur3', 'link1': 'EE_robotiq_2f85', 'model2': 'box', 'link2': 'box'}}
{'action': 'MoveL', 'value': {'x': 0.0, 'y': 0.0, 'z': 0.1}, 'speed': 0.2}
{'action': 'MoveJ', 'value': {'joint1': -42.2092, 'joint2': -77.7123, 'joint3': -112.1476, 'joint4': -80.1719, 'joint5': 89.9934, 'joint6': 2.7906}, 'speed': 1.0}
{'action': 'MoveL', 'value': {'x': 0.0, 'y': 0.0, 'z': -0.09}, 'speed': 0.2}
{'action': 'MoveG', 'value': {'value': 0.0}, 'speed': 1.0}
{'action': 'Detach', 'value': {'model1': 'ur3', 'link1': 'EE_robotiq_2f85', 'model2': 'box', 'link2': 'box'}}
{'action': 'MoveL', 'value': {'x': 0.0, 'y': 0.0, 'z': 0.09}, 'speed': 0.2}
{'action': 'MoveR', 'value': {'joint': 'joint6', 'value': 180.0}, 'speed': 1.0}
{'action': 'MoveL', 'value': {'x': 0.0, 'y': 0.0, 'z': -0.1}, 'speed': 0.2}
{'action': 'MoveG', 'value': {'value': 0.4}, 'speed': 1.0}
{'action': 'Attach', 'value': {'model1': 'ur3', 'link1': 'EE_robotiq_2f85', 'model2': 'box', 'link2': 'box'}}
{'action': 'MoveL', 'value': {'x': 0.0, 'y': 0.0, 'z': 0.1}, 'speed': 0.2}
{'action': 'MoveJ', 'value': {'joint1': -9.13615, 'joint2': -117.5559, 'joint3': -66.32366, 'joint4': -86.14282, 'joint5': 90.0091, 'joint6': 215.86377}, 'speed': 1.0}
{'action': 'MoveL', 'value': {'x': 0.0, 'y': 0.0, 'z': -0.09}, 'speed': 0.2}
{'action': 'MoveG', 'value': {'value': 0.0}, 'speed': 1.0}
{'action': 'Detach', 'value': {'model1': 'ur3', 'link1': 'EE_robotiq_2f85', 'model2': 'box', 'link2': 'box'}}
{'action': 'MoveL', 'value': {'x': 0.0, 'y': 0.0, 'z': 0.09}, 'speed': 0.2}
{'action': 'MoveJ', 'value': {'joint1': 45.0, 'joint2': -90.0, 'joint3': 0.0, 'joint4': 0.0, 'joint5': 0.0, 'joint6': -90.0}, 'speed': 1.0}
```