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

# robot.py
# This CLIENT executes Robot Movements, by calling the following ROS2 Actions:
#   - /Robmove allows the user to move the robot to a specific End-Effector pose. 
#   - /Move allows the user to execute a specific robot movement: Cartesian-Space, Joint-Space, Single Joint, Rotation... 

# ===== IMPORT REQUIRED COMPONENTS ===== #
# System:
import time
# Required to include ROS2 and its components:
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
# Import /Move and /RobMove ROS2 Actions:
from ros2srrc_data.action import Move
from ros2srrc_data.action import Robmove
  
# Global Variable -> RES:
RES = {}
RES["Success"] = False
RES["Message"] = "null"
RES["ExecTime"] = -1.0

# =============================================================================== #
# /RobMove ACTION CLIENT:

class RobMoveCLIENT(Node):

    def __init__(self):

        super().__init__('ros2srrc_RobMove_Client')
        self._action_client = ActionClient(self, Robmove, 'Robmove')

        print("[CLIENT - robot.py]: Initialising ROS2 /RobMove Action Client!")
        print("[CLIENT - robot.py]: Waiting for /Robmove ROS2 ActionServer to be available...")
        self._action_client.wait_for_server()
        print("[CLIENT - robot.py]: /Robmove ACTION SERVER detected, ready!")
        print("")

    def send_goal(self, TYPE, SPEED, TARGET_POSE):
        
        goal_msg = Robmove.Goal()
        goal_msg.type = TYPE
        goal_msg.speed = SPEED
        goal_msg.x = TARGET_POSE.x
        goal_msg.y = TARGET_POSE.y
        goal_msg.z = TARGET_POSE.z
        goal_msg.qx = TARGET_POSE.qx
        goal_msg.qy = TARGET_POSE.qy
        goal_msg.qz = TARGET_POSE.qz
        goal_msg.qw = TARGET_POSE.qw
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        
        goal_handle = future.result()

        if not goal_handle.accepted:
            print('[CLIENT - robot.py]: RobMove ACTION CALL -> GOAL has been REJECTED.')
            return
        
        print('[CLIENT - robot.py]: RobMove ACTION CALL -> GOAL has been ACCEPTED.')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        
        global RES
        
        RESULT = future.result().result
        RES["Success"] = RESULT.success  
        RES["Message"] = RESULT.message 
        
# =============================================================================== #
# /Move ACTION CLIENT:

class MoveCLIENT(Node):

    def __init__(self):

        super().__init__('ros2srrc_Move_Client')
        self._action_client = ActionClient(self, Move, 'Move')

        print("[CLIENT - robot.py]: Initialising ROS2 /Move Action Client!")
        print("[CLIENT - robot.py]: Waiting for /Move ROS2 ActionServer to be available...")
        self._action_client.wait_for_server()
        print("[CLIENT - robot.py]: /Move ACTION SERVER detected, ready!")
        print("")

    def send_goal(self, ACTION):

        goal_msg = Move.Goal()
        goal_msg.action = ACTION.action
        goal_msg.speed = ACTION.speed
        goal_msg.movej = ACTION.movej
        goal_msg.mover = ACTION.mover
        goal_msg.movel = ACTION.movel
        goal_msg.moverot = ACTION.moverot
        goal_msg.moverp = ACTION.moverp
        goal_msg.moveg = ACTION.moveg
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        
        goal_handle = future.result()

        if not goal_handle.accepted:
            print('[CLIENT - robot.py]: Move ACTION CALL -> GOAL has been REJECTED.')
            return
        
        print('[CLIENT - robot.py]: Move ACTION CALL -> GOAL has been ACCEPTED.')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        
        global RES
        
        RESULT = future.result().result  
        RES["Message"] = RESULT.result 

        if "FAILED" in RES["Message"]:
            RES["Success"] = False
        else:
            RES["Success"] = True
            
# =============================================================================== #
# ROBOT class, to execute any robot movement:

class RBT():

    def __init__(self):

        # Initialise /Move and /RobMove Action Clients:
        self.MoveClient = MoveCLIENT()
        self.RobMoveClient = RobMoveCLIENT()

    def Move_EXECUTE(self, ACTION):

        global RES
        
        T_start = time.time()

        # Initialise RES:
        RES["Success"] = False
        RES["Message"] = "null"
        RES["ExecTime"] = -1.0
        
        self.MoveClient.send_goal(ACTION)
        while rclpy.ok():
            rclpy.spin_once(self.MoveClient)

            if (RES["Message"] != "null"):
                break

        print('[CLIENT - robot.py]: Move ACTION EXECUTED -> Result: ' + RES["Message"])
        print("")
        
        T_end = time.time()
        T = round((T_end - T_start), 4)
        RES["ExecTime"] = T

        return(RES)

    def RobMove_EXECUTE(self, TYPE, SPEED, POSE):

        global RES
        
        T_start = time.time()

        # Initialise RES:
        RES["Success"] = False
        RES["Message"] = "null"
        RES["ExecTime"] = -1.0

        self.RobMoveClient.send_goal(TYPE, SPEED, POSE)
        while rclpy.ok():
            rclpy.spin_once(self.RobMoveClient)

            if (RES["Message"] != "null"):
                break

        print('[CLIENT - robot.py]: RobMove ACTION EXECUTED -> Result: ' + RES["Message"])
        print("")
        
        T_end = time.time()
        T = round((T_end - T_start), 4)
        RES["ExecTime"] = T

        return(RES)