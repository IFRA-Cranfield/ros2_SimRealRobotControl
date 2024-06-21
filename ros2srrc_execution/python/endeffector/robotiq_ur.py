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

# robotiq_ur.py
# This CLIENT operates the Robotiq Gripper connected to the UR Robot through ROS 2.

# ===== IMPORT REQUIRED COMPONENTS ===== #
# Required to include ROS2 and its components:
import rclpy
from rclpy.node import Node
# Import ROS2 Services:
from ros2_robotiqgripper.srv import RobotiqGripper

# =============================================================================== #
# Robotiq Gripper - ROS2 Service Client:

class ServiceClient(Node):

    def __init__(self):

        super().__init__('RobotiqGripper_client')

        print("[CLIENT - robotiq_ur.py]: Initialising Robotiq-UR ROS 2 Service Client!")
        self.cli = self.create_client(RobotiqGripper, '/Robotiq_Gripper')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            print('[CLIENT - robotiq_ur.py]: Waiting for RobotiqGripper Service Server to be available...')

        print('[CLIENT - robotiq_ur.py]: RobotiqGripper Service Server detected, ready!')
        print("")

        self.req = RobotiqGripper.Request()

    def send_request(self, ACTION):
        self.req.action = ACTION
        self.future = self.cli.call_async(self.req)

    def OPEN_SRV(self):
        action = "OPEN"
        self.send_request(action)

    def CLOSE_SRV(self):
        action = "CLOSE"
        self.send_request(action)

# =============================================================================== #
# Robotiq Gripper CLASS:

class RobotiqGRIPPER():

    def __init__(self):
        self.CLIENT = ServiceClient()

    def OPEN(self):

        print('[CLIENT - robotiq_ur.py]: Sending request -> OPEN GRIPPER.')

        # Initialise RESULT:
        RES = {}
        RES["Success"] = False
        RES["Value"] = 0
        RES["Average"] = 0.0
        RES["Message"] = "null"

        self.CLIENT.OPEN_SRV()

        while rclpy.ok():
            rclpy.spin_once(self.CLIENT)

            if self.CLIENT.future.done():
                
                try:
                    OpenRES = self.CLIENT.future.result()

                except Exception as exc:
                    print("[CLIENT - robotiq_ur.py]: /Robotiq_Gripper Service call failed -> " + str(exc))
                    print("")
                    return(RES)

                else:
                    RES["Success"] = OpenRES.success
                    RES["Value"] = OpenRES.value
                    RES["Average"] = OpenRES.average
                    RES["Message"] = OpenRES.message

                    print('[CLIENT - robotiq_ur.py]: OPEN GRIPPER execution finished, result -> ' + RES["Message"])
                    print("")

                    return(RES)

    def CLOSE(self):

        print('[CLIENT - robotiq_ur.py]: Sending request -> CLOSE GRIPPER.')
        
        # Initialise RESULT:
        RES = {}
        RES["Success"] = False
        RES["Value"] = 0
        RES["Average"] = 0.0
        RES["Message"] = "null"

        self.CLIENT.CLOSE_SRV()

        while rclpy.ok():
            rclpy.spin_once(self.CLIENT)

            if self.CLIENT.future.done():
                
                try:
                    CloseRES = self.CLIENT.future.result()

                except Exception as exc:
                    print("[CLIENT - robotiq_ur.py]: /Robotiq_Gripper Service call failed -> " + str(exc))
                    return(RES)

                else:
                    RES["Success"] = CloseRES.success
                    RES["Value"] = CloseRES.value
                    RES["Average"] = CloseRES.average
                    RES["Message"] = CloseRES.message

                    print('[CLIENT - robotiq_ur.py]: CLOSE GRIPPER execution finished, result -> ' + RES["Message"])
                    print("")

                    return(RES)