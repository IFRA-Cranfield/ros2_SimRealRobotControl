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

# parallelGripper.py
# This CLIENT operates any parallelGripper (which could be operated by MoveG), and checks for any potential
# attachments to any of the objects within the robot's workspace in Gazebo:

# ===== IMPORT REQUIRED COMPONENTS ===== #
# System functions and classes:
import sys, os, time
# Required to include ROS2 and its components:
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
# Import ROS2 messages:
from ros2srrc_data.msg import Action

# Import -> RobotClient for MoveG action execution:
PATH = os.path.join(get_package_share_directory("ros2srrc_execution"), 'python', 'robot')
sys.path.append(PATH)
from robot import RBT
                    
# =============================================================================== #
# parallelGR class, to OPEN/CLOSE the Parallel Gripper in Gazebo:

class parallelGR():

    def __init__(self):
        
        # Initialise RBT client -> For MoveG execution:
        self.RBTClient = RBT()

    def CLOSE(self, VAL):
        
        T_start = time.time()
        
        # Initialise -> RES:
        RES = {
            "Message": "",
            "Success": False,
            "ExecTime": -1.0
        }

        print("[CLIENT - parallelGripper.py]: EXECUTION REQUEST -> CLOSE GRIPPER.")
        print("")

        # Close GRIPPER -> /Move:
        
        G = Action()
        G.action = "MoveG"
        G.speed = 1.0
        G.moveg = VAL

        gRES = self.RBTClient.Move_EXECUTE(G)

        
        T_end = time.time()
        T = round((T_end - T_start), 4)

        RES["ExecTime"] = T
        RES["Success"] = gRES["Success"]
        RES["Message"] = "MoveG-CLOSE, Result -> " + gRES["Message"]

        print("[CLIENT - parallelGripper.py]: " + RES["Message"])
        print("")
        return(RES)

    def OPEN(self):
        
        T_start = time.time()

        # Initialise -> RES:
        RES = {
            "Message": "",
            "Success": False,
            "ExecTime": -1.0
        }
         
        print("[CLIENT - parallelGripper.py]: EXECUTION REQUEST -> OPEN GRIPPER.")
        print("")

        # Open GRIPPER -> /Move:
        
        G = Action()
        G.action = "MoveG"
        G.speed = 1.0
        G.moveg = 0.0

        gRES = self.RBTClient.Move_EXECUTE(G)

        T_end = time.time()
        T = round((T_end - T_start), 4)

        RES["ExecTime"] = T
        RES["Success"] = gRES["Success"]
        RES["Message"] = "MoveG-OPEN, Result -> " + gRES["Message"]

        print("[CLIENT - parallelGripper.py]: " + RES["Message"])
        print("")
        return(RES)

        return(RES)