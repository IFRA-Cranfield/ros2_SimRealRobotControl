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
#  Date: April, 2023.                                                                   #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023) ROS 2 Sim-to-Real Robot Control. URL: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl.

# Import required libraries:
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import os
import ast
import time

# Import ACTION:
from ros2srrc_data.action import Sequence

# Import MSG:
from ros2srrc_data.msg import Action
from ros2srrc_data.msg import Joint
from ros2srrc_data.msg import Joints
from ros2srrc_data.msg import Xyz
from ros2srrc_data.msg import Xyzypr
from ros2srrc_data.msg import Ypr
from ros2srrc_data.msg import Linkattacher

# Define GLOBAL VARIABLE -> RES:
RES = "null"

# ===== ACTION CLIENT ===== #
class ACsequence(Node):
    
    def __init__(self):
        
        # 1. Initialise node:
        super().__init__('SEQUENCE_client')
        self._action_client = ActionClient(self, Sequence, 'Sequence')
        
        # 2. Wait for AC server to be available:
        print ("Waiting for ros2srrc_data/Sequence ACTION SERVER to be available...")
        self._action_client.wait_for_server()
        print ("Sequence ACTION SERVER detected.")
    
    def send_goal(self, SEQ, ROB, EE, ENV):
        
        # 1. Assign variables:
        goal_msg = Sequence.Goal()
        goal_msg.sequence = SEQ
        goal_msg.robot = ROB
        goal_msg.endeffector = EE
        goal_msg.environment = ENV
        
        # 2. ACTION CALL:
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return
        self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        
        global RES
        
        # 1. Assign RESULT variable:
        result = future.result().result
        RES = result.result
        
        # 2. Print RESULT:
        print ("")
        print ("[SUCESS]: Program -> " + PARAM_PROGRAM + " <- successfully executed.") 

    def feedback_callback(self, feedback_msg):
        
        # 1. Assign FEEDBACK variable:
        feedback = feedback_msg.feedback
        
        # 2. Print FEEDBACK:
        print (feedback_msg.feedback.feedback)


# ===== INPUT PARAMETERS ===== #
# CLASS: Input program (.txt) as ROS2 PARAMETER:
PARAM_PROGRAM = "default"
P_CHECK_PROGRAM = False
class ProgramPARAM(Node):
    def __init__(self):
        global PARAM_PROGRAM
        global P_CHECK_PROGRAM
        
        super().__init__('ros2srrc_program_param')
        self.declare_parameter('PROGRAM_FILENAME', "default")
        PARAM_PROGRAM = self.get_parameter('PROGRAM_FILENAME').get_parameter_value().string_value
        if (PARAM_PROGRAM == "default"):
            self.get_logger().info('PROGRAM_FILENAME ROS2 Parameter was not defined.')
            CloseProgram.CLOSE()
        else:    
            self.get_logger().info('PROGRAM_FILENAME ROS2 Parameter received: ' + PARAM_PROGRAM)
        P_CHECK_PROGRAM = True

# CLASS: Input ROBOT MODEL as ROS2 PARAMETER:
PARAM_ROBOT = "default"
P_CHECK_ROBOT = False
class RobotPARAM(Node):
    def __init__(self):
        global PARAM_ROBOT
        global P_CHECK_ROBOT
        
        super().__init__('ros2srrc_robot_param')
        self.declare_parameter('ROBOT_MODEL', "default")
        PARAM_ROBOT = self.get_parameter('ROBOT_MODEL').get_parameter_value().string_value
        if (PARAM_ROBOT == "default"):
            self.get_logger().info('ROBOT_MODEL ROS2 Parameter was not defined.')
            CloseProgram.CLOSE()
        else:
            self.get_logger().info('ROBOT_MODEL ROS2 Parameter received: ' + PARAM_ROBOT)

            # Check value:
            if (PARAM_ROBOT == "irb120"
                or PARAM_ROBOT == "ur3"
                or PARAM_ROBOT == "ur10e"):
                None # do nothing.
            else:
                self.get_logger().info('ERROR: The Robot model defined is not in the system.')
                CloseProgram.CLOSE()
        
        P_CHECK_ROBOT = True

# CLASS: Input ROBOT End-Effector MODEL as ROS2 PARAMETER:
PARAM_EE = "default"
P_CHECK_EE = False
class eePARAM(Node):
    def __init__(self):
        global PARAM_EE
        global P_CHECK_EE
        
        super().__init__('ros2srrc_ee_param')
        self.declare_parameter('EE_MODEL', "default")
        PARAM_EE = self.get_parameter('EE_MODEL').get_parameter_value().string_value
        if (PARAM_EE == "default"):
            self.get_logger().info('EE_MODEL ROS2 Parameter was not defined.')
            CloseProgram.CLOSE()
        else:
            self.get_logger().info('EE_MODEL ROS2 Parameter received: ' + PARAM_EE)
            
            if (PARAM_EE == "egp64" or
                PARAM_EE == "robotiq_2f85" or
                PARAM_EE == "none"):
                None # do nothing.
            else:
                self.get_logger().info('ERROR: The End-Effector model defined is not in the system.')
                CloseProgram.CLOSE()
        
        P_CHECK_EE = True

# CLASS: Input Gazebo/Bringup ENVIRONMENT FLAG as ROS2 PARAMETER:
PARAM_GzBr = "default"
P_CHECK_GzBr = False
class GzBrPARAM(Node):
    def __init__(self):
        global PARAM_GzBr
        global P_CHECK_GzBr
        
        super().__init__('ros2srrc_gzbr_param')
        self.declare_parameter('GzBr_ENV', "default")
        PARAM_GzBr = self.get_parameter('GzBr_ENV').get_parameter_value().string_value
        if (PARAM_GzBr == "default"):
            self.get_logger().info('GzBr_ENV ROS2 Parameter was not defined.')
            CloseProgram.CLOSE()
        else:
            self.get_logger().info('GzBr_ENV ROS2 Parameter received: ' + PARAM_GzBr)

            # Check value:
            if (PARAM_GzBr == "gazebo" or 
                PARAM_GzBr == "bringup"):
                None # do nothing.
            else:
                self.get_logger().info('ERROR: The GzBr Parameter must be either "gazebo" or "bringup".')
                CloseProgram.CLOSE()
        
        P_CHECK_GzBr = True

# CLASS: WARNING + CLOSE:
class CloseProgram():
    def CLOSE():
        print("")
        print("Please execute the program and input all ROS2 parameters in the Ubuntu Terminal as stated below:")
        print('COMMAND -> ros2 run ros2srrc_execution sequence.py --ros-args -p PROGRAM_FILENAME:="---" -p ROBOT_MODEL:="---" -p EE_MODEL:="---" -p GzBr_ENV:="---"')
        print("Closing... BYE!")
        time.sleep(5)
        exit()


# ==================================================================================================================================== #
# ==================================================================================================================================== #
# =============================================================== MAIN =============================================================== #
# ==================================================================================================================================== #
# ==================================================================================================================================== #

def main(args=None):
    
    # Import global variable RES:
    global RES
    
    # 1. INITIALISE ROS NODE:
    rclpy.init(args=args)

    print("")
    print(" --- Cranfield University --- ")
    print("        (c) IFRA Group        ")
    print("")

    print("ros2srrc_execution --> SEQUENCE EXECUTION")
    print("Python script -> sequence.py")
    print("")

    # 2. INITIALISE RECEIVED ROS2 PARAMETERS:
    global PARAM_PROGRAM
    global P_CHECK_PROGRAM
    global PARAM_ROBOT
    global P_CHECK_ROBOT
    global PARAM_EE
    global P_CHECK_EE
    global PARAM_GzBr
    global P_CHECK_GzBr

    paramNODE = ProgramPARAM()
    while (P_CHECK_PROGRAM == False):
        rclpy.spin_once(paramNODE)
    paramNODE.destroy_node()

    robotNODE = RobotPARAM()
    while (P_CHECK_ROBOT == False):
        rclpy.spin_once(robotNODE)
    robotNODE.destroy_node()

    eeNODE = eePARAM()
    while (P_CHECK_EE == False):
        rclpy.spin_once(eeNODE)
    eeNODE.destroy_node()

    GzBrNODE = GzBrPARAM()
    while (P_CHECK_GzBr == False):
        rclpy.spin_once(GzBrNODE)
    GzBrNODE.destroy_node()

    # 3. CHECK if ActionServer is ACTIVE:
    SEQ_CLIENT = ACsequence()

    # 4. CHECK PROGRAM FILENAME:
    print("")
    print("All checks complete!")
    print("")

    # Create NODE for LOGGING:
    nodeLOG = rclpy.create_node('node_LOG')

    EXISTS = False
    PR_NAME = PARAM_PROGRAM
    filepath = os.path.join(os.path.expanduser('~'), 'dev_ws', 'src', 'ros2_SimRealRobotControl', 'ros2srrc_execution', 'programs', PR_NAME + ".txt")
    EXISTS = os.path.exists(filepath)
    if (EXISTS == True):
        print(PR_NAME + " file found! Executing program...")
        nodeLOG.get_logger().info("SUCCESS: " + PR_NAME + " file (program) found.")
        time.sleep(1)
    elif (EXISTS == False):
        print(PR_NAME + " file not found. Please input the PROGRAM FILENAME correctly as a ROS2 parameter in the Ubuntu Terminal:")
        nodeLOG.get_logger().info("ERROR: " + PR_NAME + " file (program) not found. Please try again.")
        print('COMMAND -> ros2 run ros2srrc_execution sequence.py --ros-args -p PROGRAM_FILENAME:="---" -p ROBOT_MODEL:="---" -p EE_MODEL:="---" -p GzBr_ENV:="---"')
        print("Closing... BYE!")
        time.sleep(5)
        exit()

    # OPEN PR_NAME.txt FILE:
    with open(filepath) as file:
        f = file.readlines()
        i = 1
        readSEQ = dict()
        for line in f:
            readSEQ[str(i)] = ast.literal_eval(line)
            i = i + 1
        file.close()

    # 5. CONVERT TO Action[] and call ACsequence():
    SEQUENCE = []
    for i in range (1, len(readSEQ)+1):

        ACTION = Action()
        ACTION.action = readSEQ[str(i)]['action']

        if (ACTION.action == "MoveJ"):
            ACTION.speed = readSEQ[str(i)]['speed']
            MoveJ_VAR = Joints()
            MoveJ_VAR.joint1 = readSEQ[str(i)]['value']['joint1']
            MoveJ_VAR.joint2 = readSEQ[str(i)]['value']['joint2']
            MoveJ_VAR.joint3 = readSEQ[str(i)]['value']['joint3']
            MoveJ_VAR.joint4 = readSEQ[str(i)]['value']['joint4']
            MoveJ_VAR.joint5 = readSEQ[str(i)]['value']['joint5']
            MoveJ_VAR.joint6 = readSEQ[str(i)]['value']['joint6']
            ACTION.movej = MoveJ_VAR
        
        elif (ACTION.action == "MoveR"):
            ACTION.speed = readSEQ[str(i)]['speed']
            MoveR_VAR = Joint()
            MoveR_VAR.joint = readSEQ[str(i)]['value']['joint']
            MoveR_VAR.value = readSEQ[str(i)]['value']['value']
            ACTION.mover = MoveR_VAR

        elif (ACTION.action == "MoveL"):
            ACTION.speed = readSEQ[str(i)]['speed']
            MoveL_VAR = Xyz()
            MoveL_VAR.x = readSEQ[str(i)]['value']['x']
            MoveL_VAR.y = readSEQ[str(i)]['value']['y']
            MoveL_VAR.z = readSEQ[str(i)]['value']['z']
            ACTION.movel = MoveL_VAR

        elif (ACTION.action == "MoveXYZW"):
            ACTION.speed = readSEQ[str(i)]['speed']
            MoveXYZW_VAR = Xyzypr()
            MoveXYZW_VAR.x = readSEQ[str(i)]['value']['x']
            MoveXYZW_VAR.y = readSEQ[str(i)]['value']['y']
            MoveXYZW_VAR.z = readSEQ[str(i)]['value']['z']
            MoveXYZW_VAR.yaw = readSEQ[str(i)]['value']['yaw']
            MoveXYZW_VAR.pitch = readSEQ[str(i)]['value']['pitch']
            MoveXYZW_VAR.roll = readSEQ[str(i)]['value']['roll']
            ACTION.movexyzw = MoveXYZW_VAR

        elif (ACTION.action == "MoveXYZ"):
            ACTION.speed = readSEQ[str(i)]['speed']
            MoveXYZ_VAR = Xyz()
            MoveXYZ_VAR.x = readSEQ[str(i)]['value']['x']
            MoveXYZ_VAR.y = readSEQ[str(i)]['value']['y']
            MoveXYZ_VAR.z = readSEQ[str(i)]['value']['z']
            ACTION.movexyz = MoveXYZ_VAR

        elif (ACTION.action == "MoveYPR"):
            ACTION.speed = readSEQ[str(i)]['speed']
            MoveYPR_VAR = Ypr()
            MoveYPR_VAR.yaw = readSEQ[str(i)]['value']['yaw']
            MoveYPR_VAR.pitch = readSEQ[str(i)]['value']['pitch']
            MoveYPR_VAR.roll = readSEQ[str(i)]['value']['roll']
            ACTION.moveypr = MoveYPR_VAR

        elif (ACTION.action == "MoveROT"):
            ACTION.speed = readSEQ[str(i)]['speed']
            MoveROT_VAR = Ypr()
            MoveROT_VAR.yaw = readSEQ[str(i)]['value']['yaw']
            MoveROT_VAR.pitch = readSEQ[str(i)]['value']['pitch']
            MoveROT_VAR.roll = readSEQ[str(i)]['value']['roll']
            ACTION.moverot = MoveROT_VAR

        elif (ACTION.action == "MoveRP"):
            ACTION.speed = readSEQ[str(i)]['speed']
            MoveRP_VAR = Xyzypr()
            MoveRP_VAR.x = readSEQ[str(i)]['value']['x']
            MoveRP_VAR.y = readSEQ[str(i)]['value']['y']
            MoveRP_VAR.z = readSEQ[str(i)]['value']['z']
            MoveRP_VAR.yaw = readSEQ[str(i)]['value']['yaw']
            MoveRP_VAR.pitch = readSEQ[str(i)]['value']['pitch']
            MoveRP_VAR.roll = readSEQ[str(i)]['value']['roll']
            ACTION.moverp = MoveRP_VAR

        elif (ACTION.action == "MoveG"):
            ACTION.speed = readSEQ[str(i)]['speed']
            ACTION.moveg = readSEQ[str(i)]['value']['value']

        elif (ACTION.action == "Attach"):
            Attach_VAR = Linkattacher()
            Attach_VAR.model1_name = readSEQ[str(i)]['value']['model1']
            Attach_VAR.link1_name = readSEQ[str(i)]['value']['link1']
            Attach_VAR.model2_name = readSEQ[str(i)]['value']['model2']
            Attach_VAR.link2_name = readSEQ[str(i)]['value']['link2']
            ACTION.attach = Attach_VAR

        elif (ACTION.action == "Detach"):
            Detach_VAR = Linkattacher()
            Detach_VAR.model1_name = readSEQ[str(i)]['value']['model1']
            Detach_VAR.link1_name = readSEQ[str(i)]['value']['link1']
            Detach_VAR.model2_name = readSEQ[str(i)]['value']['model2']
            Detach_VAR.link2_name = readSEQ[str(i)]['value']['link2']
            ACTION.detach = Detach_VAR

        elif (ACTION.action == "ABB - GripperOpen"):
            pass # No action required, since ACTION name is passed and enough.
        elif (ACTION.action == "ABB - GripperClose"):
            pass # No action required, since ACTION name is passed and enough.
        
        SEQUENCE.append(ACTION)
    
    # 6. CALL ROS2 Action -> SEQUENCE:
    SEQ_CLIENT.send_goal(SEQUENCE, PARAM_ROBOT, PARAM_EE, PARAM_GzBr)
            
    while rclpy.ok():
        rclpy.spin_once(SEQ_CLIENT)
        if (RES != "null"):
            break
    
    nodeLOG.destroy_node()
    print("Closing... BYE!")
    time.sleep(5)
        

if __name__ == '__main__':
    main()