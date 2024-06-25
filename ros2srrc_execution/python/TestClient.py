#!/usr/bin/python3

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Dr. Seemal Asif  - s.asif@cranfield.ac.uk                                   #
#           Prof. Phil Webb  - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: June, 2024.                                                                    #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2024) ROS 2 Sim-to-Real Robot Control. URL: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl.

# TestClient.py
# This python script executes different ROBOT/END-EFFECTOR commands by using the Python Clients in ros2srrc_execution.

# ===== IMPORT REQUIRED COMPONENTS ===== #
# System functions and classes:
import sys, os
# Required to include ROS2 and its components:
import rclpy
from ament_index_python.packages import get_package_share_directory

# IMPORT ROS2 Custom Messages:
from ros2srrc_data.msg import Action
from ros2srrc_data.msg import Joint
from ros2srrc_data.msg import Joints
from ros2srrc_data.msg import Xyz
from ros2srrc_data.msg import Xyzypr
from ros2srrc_data.msg import Ypr
from ros2srrc_data.msg import Robpose

# IMPORT Python classes:
PATH = os.path.join(get_package_share_directory("ros2srrc_execution"), 'python')
PATH_robot = PATH + "/robot"
PATH_endeffector = PATH + "/endeffector"
PATH_endeffector_gz = PATH + "/endeffector_gz"
# ROBOT CLASS:
sys.path.append(PATH_robot)
from robot import RBT
# END EFFECTOR CLASSES (Gazebo):
sys.path.append(PATH_endeffector)
from robotiq_ur import RobotiqGRIPPER
from schunk_abb import SchunkGRIPPER
# END EFFECTOR CLASSES (Gazebo):
sys.path.append(PATH_endeffector_gz)
from parallelGripper import parallelGR

# ===================================================================== #
# Test -> ROBOT CLASS (RobMove):
def Test_ROBOT_Move():

    client = RBT()
    ACTION = Action()

    # Select ACTION TYPE: (MoveJ, MoveL, MoveR, MoveROT, MoveRP, MoveG)
    ActionType = "MoveJ"

    # Define -> Action type + speed:
    ACTION.action = ActionType
    ACTION.speed = 1.0

    if ActionType == "MoveJ":

        INPUT = Joints()
        INPUT.joint1 = 0.0
        INPUT.joint2 = 0.0
        INPUT.joint3 = 0.0
        INPUT.joint4 = 0.0
        INPUT.joint5 = 90.0
        INPUT.joint6 = 0.0
        ACTION.movej = INPUT

    elif ActionType == "MoveL":

        INPUT = Xyz()
        INPUT.x = 0.0
        INPUT.y = 0.0
        INPUT.z = 0.15
        ACTION.movel = INPUT

    elif ActionType == "MoveR":

        INPUT = Joint()
        INPUT.joint = "joint1"
        INPUT.value = 90.0
        ACTION.mover = INPUT

    elif ActionType == "MoveROT":

        INPUT = Ypr()
        INPUT.pitch = 0.0
        INPUT.yaw = 0.0
        INPUT.roll = 90.0
        ACTION.moverot = INPUT

    elif ActionType == "MoveRP":

        INPUT = Xyzypr()
        INPUT.x = 0.0
        INPUT.y = 0.0
        INPUT.z = 0.10
        INPUT.pitch = 0.0
        INPUT.yaw = 0.0
        INPUT.roll = 90.0

    else:

        print("[TEST-ROBOT-Move]: ERROR -> /Move TYPE not properly defined.")
        return()

    # Execute:
    RES = client.Move_EXECUTE(ACTION)
    print("[TEST-ROBOT-Move]: Action executed!")
    print("[TEST-ROBOT-Move]: Result -> " + RES["Message"])
    print("")
    
    return()

# ===================================================================== #
# Test -> ROBOT CLASS (RobMove):
def Test_ROBOT_RobMove():

    client = RBT()

    # Define -> MOVEMENT TYPE: (PTP, LIN)
    MovType = "PTP"

    # Define -> SPEED:
    Speed = 1.0

    # Define -> POSE:     
    InputPose = Robpose()
    InputPose.x = 0.44
    InputPose.y = 0.52
    InputPose.z = 1.36
    InputPose.qx = 0.0
    InputPose.qy = 1.0
    InputPose.qz = 0.0
    InputPose.qw = 0.0

    # Execute:
    RES = client.RobMove_EXECUTE(MovType, Speed, InputPose)
    print("[TEST-ROBOT-RobMove]: Action executed!")
    print("[TEST-ROBOT-RobMove]: Result -> " + RES["Message"])
    print("") 

    return()

# ===================================================================== #
# Test -> Parallel Gripper class:
def Test_PG():
    
    OBJECTS = ["BlackCube", "RedCube"]
    ROBOT = "irb120"
    EE_link = "EE_egp64"

    client = parallelGR(OBJECTS, ROBOT, EE_link)

    # Select ACTION: (OPEN, CLOSE)
    ACTION = "CLOSE"

    # If CLOSE, define closing value between 0(completely open)-100(completely closed):
    VAL = 25.0 

    if ACTION == "CLOSE":
        client.CLOSE(VAL)
    
    if ACTION == "OPEN":
        client.OPEN()
        
    return()

# ===================================================================== #
# Test -> RobotiqUR class:
def Test_RobotiqUR():
    None # TBD
    return()

# ===================================================================== #
# Test -> SchunkABB class:
def Test_SchunkABB():
    None # TBD
    return()

# ===================================================================== #
# Get OPTION as input argument:
def AssignArgument(ARGUMENT):
    ARGUMENTS = sys.argv
    for y in ARGUMENTS:
        if (ARGUMENT + ":=") in y:
            ARG = y.replace((ARGUMENT + ":="),"")
            return(ARG)

# ===================================================================== #
# ===================================================================== #

def main(args=None):
    
    rclpy.init(args=args)

    # CHOOSE OPTION -> "ROBOT-Move", "ROBOT-RobMove", "ParallelGripper", "RobotiqUR", "SchunkABB"
    OPTION = AssignArgument("OPTION")

    if OPTION == "ROBOT-Move":

        print("[TestClient]: Executing ROBOT -> /Move action...")
        print("")
        Test_ROBOT_Move()

    elif OPTION == "ROBOT-RobMove":

        print("[TestClient]: Executing ROBOT -> /RobMove action...")
        print("")
        Test_ROBOT_RobMove()

    elif OPTION == "ParallelGripper":

        print("[TestClient]: Testing ParallelGripper...")
        print("")
        Test_PG()

    elif OPTION == "RobotiqUR":

        print("[TestClient]: Testing RobotiqUR...")
        print("")
        Test_RobotiqUR()

    elif OPTION == "SchunkABB":

        print("[TestClient]: Testing SchunkABB...")
        print("")
        Test_SchunkABB()

    else:

        print("[TestClient]: OPTION not correct. Try again!")

    print("[TestClient]: Closing program... BYE!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()