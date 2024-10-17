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

# test_PG.py -> This python script creates an instance of the parallelGR() class in ros2srrc_execution and operates the PARALLEL GRIPPER in Gazebo.

# ===== IMPORT REQUIRED COMPONENTS ===== #
# System functions and classes:
import sys, os
# Required to include ROS2 and its components:
import rclpy
from ament_index_python.packages import get_package_share_directory

# IMPORT Python classes:
PATH = os.path.join(get_package_share_directory("ros2srrc_execution"), 'python')
PATH_endeffector_gz = PATH + "/endeffector_gz"
# END EFFECTOR CLASSES (Gazebo):
sys.path.append(PATH_endeffector_gz)
from parallelGripper import parallelGR

# ===================================================================== #
# ===================================================================== #

def main(args=None):
    
    rclpy.init(args=args)

    # ============================ #
    # Get VARIABLES for CLIENT INIT:

    # Environment variables:
    OBJECTS = None
    ROBOT = "ur3"
    EE_link = "EE_robotiq_hande"

    # ACTION:
    ACTION = "CLOSE"
    VAL = 15.0

    # ================ #
    # Initialise CLIENT:
    client = parallelGR(OBJECTS, ROBOT, EE_link)

    # Execute:
    if ACTION == "CLOSE":
        RES = client.CLOSE(VAL)
    elif ACTION == "OPEN":
        RES = client.OPEN()
    else:
        RES = {}
        RES["Message"] = "ERROR: ACTION was not properly defined, it should be OPEN/CLOSE."

    print("[TEST-pGRIPPER-Gz]: Action executed! -> Requested: " + ACTION)
    print("[TEST-pGRIPPER-Gz]: Result -> " + RES["Message"])
    print("") 

    rclpy.shutdown()
    print("CLOSING PROGRAM... BYE!")
    exit()

    # ============= # 
    # NOTE: The ROBOT, END-EFFECTOR and OBJECT(S) defined in this script which are the input to parallelGR() 
    # have been tested on a UR robot with a Robotiq HandE gripper and different colored cubes, therefore 
    # these might need to be adjusted if a different robot, end-effector or use-case is used.
    # ============= #

if __name__ == '__main__':
    main()