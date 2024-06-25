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
# Import LinkAttacher (ROS2 SRV):
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
# Import ROS2 messages:
from std_msgs.msg import String
from ros2srrc_data.msg import Action
from ros2srrc_data.msg import Robpose
from objectpose_msgs.msg import ObjectPose

# Import -> RobotClient for MoveG action execution:
PATH = os.path.join(get_package_share_directory("ros2srrc_execution"), 'python', 'robot')
sys.path.append(PATH)
from robot import RBT

# GLOBAL VARIABLE -> OBJECTS:
OBJECTS = []

# GLOBAL VARIABLE -> EEPose:
EEPose = Robpose()

# GLOBAL VARIABLE -> AttachCheck:
from dataclasses import dataclass
@dataclass
class AttDetCHECK:                     # Define -> Data class.
    ATTACHED: bool
    NAME: String
AttachCheck = AttDetCHECK(False,"")    # Initialise variable.

# =============================================================================== #
# ObjectPose SUBSCRIBER:

class ObjPOSE(Node):

    def __init__(self, ObjectLIST):

        super().__init__("ros2srrc_EEGz_ObjectPose_Subscriber")

        self.subLIST = []

        for x in ObjectLIST:

            TopicName = "/" + x + "/ObjectPose"
            self.subLIST.append(self.create_subscription(ObjectPose, TopicName, self.CALLBACK_FN, 10))

    def CALLBACK_FN(self, OBJ):

        global OBJECTS

        for x in OBJECTS:
            if OBJ.objectname == x.objectname:
                x.x = OBJ.x
                x.y = OBJ.y
                x.z = OBJ.z
                x.qx = OBJ.qx
                x.qy = OBJ.qy
                x.qz = OBJ.qz
                x.qw = OBJ.qw

    def getOBJECTS(self):

        global OBJECTS

        T = time.time() + 0.50
        while time.time() < T:
            rclpy.spin_once(self)
        
        return(OBJECTS)
    
# =============================================================================== #
# Robot(EE) Pose SUBSCRIBER:

class eePOSE(Node):

    def __init__(self):

        super().__init__("ros2srrc_EEGz_eePose_Subscriber")
        self.SUB = self.create_subscription(Robpose, "/Robpose", self.CALLBACK_FN, 10)

    def CALLBACK_FN(self, POSE):

        global EEPose

        EEPose.x = POSE.x
        EEPose.y = POSE.y
        EEPose.z = POSE.z
        EEPose.qx = POSE.qx
        EEPose.qy = POSE.qy
        EEPose.qz = POSE.qz
        EEPose.qw = POSE.qw

    def getEEPose(self):

        global EEPose

        T = time.time() + 0.50
        while time.time() < T:
            rclpy.spin_once(self)
        
        return(EEPose)
    
# =============================================================================== #
# LinkAttacher SERVICE CLIENT:

class LinkAttacher_Client(Node):

    def __init__(self, ROBOT, EE):

        super().__init__("ros2srrc_EEGz_LinkAttacher_Client")

        self.AttachClient = self.create_client(AttachLink, "/ATTACHLINK")
        self.DetachClient = self.create_client(DetachLink, "/DETACHLINK")

        print("[CLIENT - parallelGripper.py]: Initialising /ATTACHLINK and /DETACHLINK ROS 2 Service Clients.")

        while not self.AttachClient.wait_for_service(timeout_sec=1.0): 
            print("[CLIENT - parallelGripper.py]: /ATTACHLINK ROS2 Service not still available, waiting...")
        print("[CLIENT - parallelGripper.py]: /ATTACHLINK ROS2 Service ready.")
        while not self.DetachClient.wait_for_service(timeout_sec=1.0): 
            print("[CLIENT - parallelGripper.py]: /DETACHLINK ROS2 Service not still available, waiting...")
        print("[CLIENT - parallelGripper.py]: /DETACHLINK ROS2 Service ready.")

        print("")

        self.AttachRequest = AttachLink.Request()
        self.DetachRequest = DetachLink.Request()

        self.ROBOT = ROBOT
        self.EE = EE

    def ATTACHService(self, NAME):

        self.AttachRequest.model1_name = self.ROBOT
        self.AttachRequest.link1_name = self.EE
        self.AttachRequest.model2_name = NAME
        self.AttachRequest.link2_name = NAME

        self.AttachFuture = self.AttachClient.call_async(self.AttachRequest)

    def DETACHService(self, NAME):

        self.DetachRequest.model1_name = self.ROBOT
        self.DetachRequest.link1_name = self.EE
        self.DetachRequest.model2_name = NAME
        self.DetachRequest.link2_name = NAME

        self.DetachFuture = self.DetachClient.call_async(self.DetachRequest)

class LinkAttacher():

    def __init__(self, ROBOT, EE):
        self.CLIENT = LinkAttacher_Client(ROBOT, EE)

    def ATTACH(self, NAME):

        global AttachCheck

        self.CLIENT.ATTACHService(NAME)

        while rclpy.ok():
            rclpy.spin_once(self.CLIENT)
            if self.CLIENT.AttachFuture.done():
                try:
                    AttachRES = self.CLIENT.AttachFuture.result()
                except Exception as exc:
                    print("[CLIENT - parallelGripper.py]: /ATTACHLINK Service call failed -> " + str(exc))
                    print("")
                    return(False)
                else:
                    if (AttachRES.success):
                        print("[CLIENT - parallelGripper.py]: /ATTACHLINK successful -> " + str(AttachRES.message))
                        print("")
                        AttachCheck.ATTACHED = True
                        AttachCheck.NAME = NAME
                        return(True)
                    else:
                        print("[CLIENT - parallelGripper.py]: /ATTACHLINK unuccessful -> " + str(AttachRES.message))
                        print("")
                        return(False)
                    
    def DETACH(self, NAME):

        global AttachCheck

        self.CLIENT.DETACHService(NAME)

        while rclpy.ok():
            rclpy.spin_once(self.CLIENT)
            if self.CLIENT.DetachFuture.done():
                try:
                    DetachRES = self.CLIENT.DetachFuture.result()
                except Exception as exc:
                    print("[CLIENT - parallelGripper.py]: /DETACHLINK Service call failed -> " + str(exc))
                    print("")
                    return(False)
                else:
                    if (DetachRES.success):
                        print("[CLIENT - parallelGripper.py]: /DETACHLINK successful -> " + str(DetachRES.message))
                        print("")
                        AttachCheck.ATTACHED = False
                        AttachCheck.NAME = ""
                        return(True)
                    else:
                        print("[CLIENT - parallelGripper.py]: /DETACHLINK unuccessful -> " + str(DetachRES.message))
                        print("")
                        return(False)
                    
# =============================================================================== #
# parallelGR class, to OPEN/CLOSE the Parallel Gripper in Gazebo:

class parallelGR():

    def __init__(self, ObjectList, ROBOT, EE):
        
        # Initialise RBT client -> For MoveG execution:
        self.RBTClient = RBT()

        # Initialise OBJECTS variable:
        for x in ObjectList:
            OBJ = ObjectPose()
            OBJ.objectname = x
            OBJECTS.append(OBJ)

        # Initialise ObjPose and eePose classes:
        self.objPoseClient = ObjPOSE(ObjectList)
        self.eePoseClient = eePOSE()

        # Initialise LinkAttacher class:
        self.LinkAttacher = LinkAttacher(ROBOT, EE)

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

        if (gRES["Success"] == True):
            print("[CLIENT - parallelGripper.py]: MoveG-CLOSE, Result -> " + gRES["Message"])
            print("")
            
        else:
            RES["Message"] = "MoveG-CLOSE, Result -> " + gRES["Message"]
            print("[CLIENT - parallelGripper.py]: " + RES["Message"])
            print("")
            return(RES)

        # ===== CHECK GRASPING ===== #
        Objects = self.objPoseClient.getOBJECTS()
        EEPose = self.eePoseClient.getEEPose()

        objNAME = ""
        for x in Objects:

            Check = True
            print("[CLIENT - parallelGripper.py]: Checking if object is attached to ParallelGripper: " + x.objectname)
            print("[CLIENT - parallelGripper.py]: EEPose.x -> " + str(EEPose.x) + " / ObjectPose.x -> " + str(x.x))
            print("[CLIENT - parallelGripper.py]: EEPose.y -> " + str(EEPose.y) + " / ObjectPose.y -> " + str(x.y))
            print("[CLIENT - parallelGripper.py]: EEPose.z -> " + str(EEPose.z) + " / ObjectPose.z -> " + str(x.z))
            print("")

            if (EEPose.x - 0.01 > x.x) or (EEPose.x + 0.01 < x.x): 
                Check = False
            if (EEPose.y - 0.01 > x.y) or (EEPose.y + 0.01 < x.y): 
                Check = False
            if (EEPose.z - 0.01 > x.z) or (EEPose.z + 0.01 < x.z): 
                Check = False

            if Check == True:
                objNAME = x.objectname
                break

        # LinkAttacher:
        if Check:
            
            AttRES = self.LinkAttacher.ATTACH(objNAME)
            if AttRES:
                RES["Message"] = "Gripper closed, object->" + objNAME + " attached."
                RES["Success"] = True
                print("[CLIENT - parallelGripper.py]: " + RES["Message"])
                print("")
            else:
                RES["Message"] = "Gripper closed, object->" + objNAME + " not attached, LinkAttacher plugin failed."
                print("[CLIENT - parallelGripper.py]: " + RES["Message"])
                print("")

        else:
            RES["Message"] = "Gripper closed without grasping any object."
            RES["Success"] = True
            print("[CLIENT - parallelGripper.py]: " + RES["Message"])
            print("")
            
        T_end = time.time()
        T = round((T_end - T_start), 4)
        RES["ExecTime"] = T

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

        if (gRES["Success"] == True):
            print("[CLIENT - parallelGripper.py]: MoveG-OPEN, Result -> " + gRES["Message"])
            
        else:
            RES["Message"] = "MoveG-OPEN, Result -> " + gRES["Message"]
            print("[CLIENT - parallelGripper.py]: " + RES["Message"])
            return(RES)

        # CHECK if --> There is any object currently grasped:
        global AttachCheck 
        objNAME = AttachCheck.NAME

        if AttachCheck.ATTACHED:

            DetRES = self.LinkAttacher.DETACH(objNAME)

            if DetRES:
                RES["Message"] = "Gripper opened, object->" + objNAME + " detached."
                RES["Success"] = True
                print("[CLIENT - parallelGripper.py]: " + RES["Message"])
                print("")
            else: 
                RES["Message"] = "Gripper opened, object->" + objNAME + " not detached, LinkAttacher plugin failed."
                print("[CLIENT - parallelGripper.py]: " + RES["Message"])
                print("")

        else:
            RES["Message"] = "Gripper opened without dropping any object."
            RES["Success"] = True
            print("[CLIENT - parallelGripper.py]: Gripper opened without dropping any object.")
            print("")
            
        T_end = time.time()
        T = round((T_end - T_start), 4)
        RES["ExecTime"] = T

        return(RES)