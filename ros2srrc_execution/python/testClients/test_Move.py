#!/usr/bin/env python3

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
# IFRA-Cranfield (2023) ROS 2 Sim-to-Real Robot Control. URL: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl.

# test_Move.py -> This python script creates an instance of the RBT() class in ros2srrc_execution and executes the Move ROS 2 Action.

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

# IMPORT Python classes:
PATH = os.path.join(get_package_share_directory("ros2srrc_execution"), 'python')
PATH_robot = PATH + "/robot"
# ROBOT CLASS:
sys.path.append(PATH_robot)
from robot import RBT

# ===================================================================== #
# ===================================================================== #

def main(args=None):
    
    rclpy.init(args=args)

    # Initialise class:
    client = RBT()
    ACTION = Action()

    # ACTION TYPE -> (MoveJ, MoveL, MoveR, MoveROT, MoveRP, MoveG)
    ActionType = "MoveJ"

    # Define -> Action type + speed:
    ACTION.action = ActionType
    ACTION.speed = 1.0

    if ActionType == "MoveJ":

        # Intermediate Position for UR3 Robot:
        INPUT = Joints()
        INPUT.joint1 = 90.0
        INPUT.joint2 = -90.0
        INPUT.joint3 = 90.0
        INPUT.joint4 = -90.0
        INPUT.joint5 = -90.0
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
        print("")

        rclpy.shutdown()
        print("CLOSING PROGRAM... BYE!")
        exit()

    # Execute:
    RES = client.Move_EXECUTE(ACTION)
    print("[TEST-ROBOT-Move]: Action executed!")
    print("[TEST-ROBOT-Move]: Result -> " + RES["Message"])
    print("")

    rclpy.shutdown()
    print("CLOSING PROGRAM... BYE!")
    exit()


if __name__ == '__main__':
    main()