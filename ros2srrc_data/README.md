## ROS 2 Sim-to-Real Robot Control: ros2srrc_data

The ros2srrc_data package contains the ROS 2 data structures required by the source code defined in the ros2srrc_execution package.

### Robot Movement execution

__ROS 2 .action__

In ros2_SimRealRobotControl, individual robot movements are contained in a single ROS 2 node (ROS 2 action server, move.cpp), and are executed by calling a single action named "Move".

Move.action:
* Input: action(string), speed(float64), movej(joints), mover(joint), movel(xyz), moverot(ypr), moverp(xyzypr), moveg(float64).
* Output: result(string), feedback(string).

Robmove.action:
* Input: type(string), speed(float64), x(float64), y(float64), z(float64), qx(float64), qy(float64), qz(float64), qw(float64).
* Output: success(bool), message(string), feedback(string).

__ROS 2 .msg__

Every single robot movement type (MoveJ, MoveR, MoveL...) is defined on a specific ROS 2 message format:

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

__ROS 2 .action__

The sequences/programs are executed by calling the single ROS 2 action "Sequence", which contains an array with robot movements (defined in "Action.msg") that are executed one after the other. Instead of having to call the ROS 2 action "Move" for every single step**, the whole sequence is passed to ExecuteProgram.py, and movements are executed one by one using MoveGroupInterface.

Sequence.action:
* Input: Sequence(action[]), robot(string), endeffector(string), environment(string).
* Output: result(string), feedback(string).

__ROS 2 .msg__

All possible robot movements have been put into the "Action" ROS 2 message, in order to be able to generate an array containing the whole sequence.

Action.msg:
* Data: action(string), speed(float64), movej(joints), mover(joint), movel(xyz), movexyzw(xyzypr), movexyz(xyz), moveypr(ypr), moverot(ypr), moverp(xyzypr), moveg(float64).

</br>
</br>

** This method is used in ros2_RobotSimulation, which includes a small delay between steps due to the ROS 2 action call which needs to be done before every step.


