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

# vgr_abb.py
# This CLIENT operates the Vacuum Gripper connected to the ABB Robot through ROS 2.

# ===== IMPORT REQUIRED COMPONENTS ===== #
# Required to include ROS2 and its components:
import rclpy
from rclpy.node import Node
# Import ROS2 Services:
from abb_robot_msgs.srv import SetIOSignal

# =============================================================================== #
# ABB Robot I/O - ROS2 Service Client:

class vgrABB(Node):

    def __init__(self):

        super().__init__('VacuumGripper_client')

        print("[CLIENT - vgr_abb.py]: Initialising ABB-RWS I/O ROS 2 Service Client.")

        self.cli = self.create_client(SetIOSignal, '/rws_client/set_io_signal')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            print('[CLIENT - vgr_abb.py]: Waiting for ABB-RWS I/O Service Server to be available...')

        print('[CLIENT - vgr_abb.py]: ABB-RWS I/O Service Server detected, ready!')
        print("")

        self.req = SetIOSignal.Request()

    def send_request(self, signal, value):
        self.req.signal = signal
        self.req.value = value
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        print('[CLIENT - vgr_abb.py]: ROS 2 Service successfully executed. Signal -> ' + signal + ", Value -> " + value)
        print("")

    def DEACTIVATE(self):

        RES = {}
        RES["Success"] = False
        RES["Message"] = ""

        print('[CLIENT - vgr_abb.py]: Sending request -> DEACTIVATE VACUUM.')
        signal = "vacuum"
        value = "0"
        self.send_request(signal,value)

        RES["Message"] = "Vacuum Gripper (0-DEACTIVATE) signal successfully sent to ABB Robot Controller."
        RES["Success"]= True
        return(RES)

    def ACTIVATE(self):

        RES = {}
        RES["Success"] = False
        RES["Message"] = ""

        print('[CLIENT - vgr_abb.py]: Sending request -> ACTIVATE VACUUM.')
        signal = "vacuum"
        value = "1"
        self.send_request(signal,value)

        RES["Message"] = "Vacuum Gripper (1-ACTIVATE) signal successfully sent to ABB Robot Controller."
        RES["Success"]= True
        return(RES)