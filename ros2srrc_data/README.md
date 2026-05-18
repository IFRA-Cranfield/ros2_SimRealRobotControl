## ROS2 Sim-to-Real Robot Control: ros2srrc_data

The ros2srrc_data package contains all different ROS2 data structures that are required to execute the Source Code defined in the ros2srrc_execution package.

### Robot Movement execution

__ROS2 .action__

In ros2_SimRealRobotControl, individual Robot Movements contained in a single ROS2 Node (ROS2 Action Server, move.cpp), and are executed by calling a single action, called, "Move".

Move.action:
* Input: action(string), speed (float64), movej (joints), mover (joint), movel (xyz), moverot (ypr), moverp (xyzypr), moveg (float64).
* Output: result(string), feedback(string).

Robmove.action:
* Input: type(string), speed(float64), x(float64), y(float64), z(float64), qx(float64), qy(float64), qz(float64), qw(float64).
* Output: success(bool), message(string), feedback(string).

__ROS2 .msg__

Every single Robot Movement type (MoveJ, MoveR, MoveL...) is defined on a specific ROS2 MSG format:

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

Robpose.msg:
* Data: x(float64), y(float64), z(float64), qx(float64), qy(float64), qz(float64), qw(float64).

Specs.msg:
* Data: ee_max(float64), ee_min(float64), ee_vector(float64[]), robot_max(float64[]), robot_min(float64[]).

Linkattacher.msg:
* Data: model1_name(string), link1_name(string), model2_name(string), link2_name(string).

### Sequence execution

__ROS2 .action__

The sequences/programs are executed by calling the single ROS2 Action "Sequence", which contains an array with Robot Movements (defined in "Action.msg") that are executed one after the other. Instead of having to call the ROS2 Action "Move" for every single step**, the whole sequence is passed to ExecuteProgram.py, and movements are executed one by one using MoveGroupInterface.

Sequence.action:
* Input: Sequence(action[]), robot(string), endeffector(string), environment(string).
* Output: result(string), feedback(string).

__ROS2 .msg__

All possible Robot Movements have been put into the "Action" ROS2 message, in order to be able to generate an array containing the whole sequence.

Action.msg:
* Data: action(string), speed (float64), movej (joints), mover (joint), movel (xyz), movexyzw (xyzypr), movexyz (xyz), moveypr (ypr), moverot (ypr), moverp (xyzypr), moveg (float64), attach (linkattacher), detach (linkattacher).

</br>
</br>

** This method is used in ros2_RobotSimulation, which includes a small delay between steps due to the ROS2 Action call which needs to be done before every step.