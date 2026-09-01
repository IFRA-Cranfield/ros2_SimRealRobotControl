## ROS 2 Sim-to-Real Robot Control: ros2srrc_data

The ros2srrc_data package contains all different ROS 2 data structures that are required to execute the Source Code defined in the ros2srrc_execution package.

### Robot Movement execution

__ROS 2 .action__

In ros2_SimRealRobotControl, individual Robot Movements are contained in a single ROS 2 Node (ROS 2 Action Server, move.cpp), and are executed by calling a single action, called, "Move".

Move.action:
* Input: Action(string), Speed (float64), MoveJ (joints), MoveR (joint), MoveL (xyz), MoveXYZW (xyzypr), MoveXYZ (xyz), MoveYPR (ypr), MoveROT (ypr), MoveRP (xyzypr), MoveG (float64).
* Output: result(string), feedback(string).

__ROS 2 .msg__

Every single Robot Movement type (MoveJ, MoveR, MoveL...) is defined on a specific ROS 2 MSG format:

Joints.msg:
* Data: joint1(float64), joint2(float64), joint3(float64), joint4(float64), joint5(float64), joint6(float64), joint7(float64).

Joint.msg:
* Data: joint(string), value(float64).

Xyz.msg:
* Data: x(float64), y(float64), z(float64).

Xyzypr.msg:
* Data: x(float64), y(float64), z(float64), yaw(float64), pitch(float64), roll(float64).

Ypr.msg:
* Data: yaw(float64), pitch(float64), roll(float64).

### Sequence execution

__ROS 2 .action__

The sequences/programs are executed by calling the single ROS 2 Action "Sequence", which contains an array with Robot Movements (defined in "Action.msg") that are executed one after the other. Instead of having to call the ROS 2 Action "Move" for every single step**, the whole sequence is passed to sequence.cpp, and movements are executed one by one using MoveGroupInterface.

Sequence.action:
* Input: Sequence(action[]), robot(string), endeffector(string), environment(string).
* Output: result(string), feedback(string).

__ROS 2 .msg__

All possible Robot Movements have been put into the "Action" ROS 2 message, in order to be able to generate an array containing the whole sequence.

Action.msg:
* Data: Action(string), Speed (float64), MoveJ (joints), MoveR (joint), MoveL (xyz), MoveXYZW (xyzypr), MoveXYZ (xyz), MoveYPR (ypr), MoveROT (ypr), MoveRP (xyzypr), MoveG (float64), Attach(string), Detach(string).

</br>
</br>

** This method avoids the small delay introduced when a separate ROS 2 Action call is made before every step.


