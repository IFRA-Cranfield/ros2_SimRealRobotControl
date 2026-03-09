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

# test_RobMove.py -> This python script creates an instance of the RBT() class in ros2srrc_execution and executes the RobMove ROS 2 Action.

# ===== IMPORT REQUIRED COMPONENTS ===== #
# System functions and classes:
import sys, os
# Required to include ROS2 and its components:
import rclpy
from ament_index_python.packages import get_package_share_directory

# IMPORT ROS2 Custom Messages:
from ros2srrc_data.msg import Robpose

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

    client = RBT()

    # ======================================== #
    # Movement1 -> PointToPoint:

    # Define -> MOVEMENT TYPE: (PTP, LIN)
    MovType = "PTP"
    # Define -> SPEED:
    Speed = 1.0

    # Define -> POSE:     
    InputPose = Robpose()
    InputPose.x = 0.0
    InputPose.y = 0.25
    InputPose.z = 0.95
    InputPose.qx = 0.0
    InputPose.qy = 1.0
    InputPose.qz = 0.0
    InputPose.qw = 0.0

    # Execute:
    RES = client.RobMove_EXECUTE(MovType, Speed, InputPose)
    print("[TEST-ROBOT-RobMove]: Action executed!")
    print("[TEST-ROBOT-RobMove]: Result -> " + RES["Message"])
    print("")

    # ======================================== #
    # Movement2 -> LINEAR:

    # Define -> MOVEMENT TYPE: (PTP, LIN)
    MovType = "LIN"
    # Define -> SPEED:
    Speed = 0.1

    # Define -> POSE:     
    InputPose = Robpose()
    InputPose.x = 0.0
    InputPose.y = 0.25
    InputPose.z = 0.86
    InputPose.qx = 0.0
    InputPose.qy = 1.0
    InputPose.qz = 0.0
    InputPose.qw = 0.0

    # Execute:
    RES = client.RobMove_EXECUTE(MovType, Speed, InputPose)
    print("[TEST-ROBOT-RobMove]: Action executed!")
    print("[TEST-ROBOT-RobMove]: Result -> " + RES["Message"])
    print("")

    rclpy.shutdown()
    print("CLOSING PROGRAM... BYE!")
    exit()

    # ============= # 
    # NOTE: The Robot Poses defined in this script which are the input to RobMove have been tested on a UR3, 
    # therefore these might need to be adjusted if a different robot is used.
    # ============= #

if __name__ == '__main__':
    main()