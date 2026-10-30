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
#  Date: October, 2026.                                                                 #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2026) ROS 2 Sim-to-Real Robot Control. URL: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl.

# vacuumGripper.py
# This CLIENT operates any vacuumGripper, by checking for any potential
# attachments to any of the objects within the robot's workspace in Gazebo:

# ===== IMPORT REQUIRED COMPONENTS ===== #
# System functions and classes:
import sys, os, time
from functools import partial
# Required to include ROS 2 and its components:
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
# Import LinkAttacher (ROS 2 SRV):
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
# Import ROS 2 messages:
from std_msgs.msg import String
from ros2srrc_data.msg import Action
from ros2srrc_data.msg import Robpose
from objectpose_msgs.msg import ObjectPose

# Import -> RobotClient for MoveG action execution:
PATH = os.path.join(get_package_share_directory("ros2srrc_execution"), 'python', 'robot')
sys.path.append(PATH)
from robot import RBT

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

    def __init__(self):

        super().__init__("ros2srrc_EEGz_ObjectPose_Subscriber")
        self._object_subscriptions = {}
        self._object_poses = {}

    def _active_object_topics(self):

        topics = {}
        for topic, types in self.get_topic_names_and_types():
            parts = topic.split("/")
            if (len(parts) == 3 and parts[1] and parts[2] == "ObjectPose"
                    and "objectpose_msgs/msg/ObjectPose" in types
                    and self.count_publishers(topic) > 0):
                topics[parts[1]] = topic
        return topics

    def _refresh_subscriptions(self):

        topics = self._active_object_topics()
        for name in list(self._object_subscriptions):
            if name not in topics:
                self.destroy_subscription(self._object_subscriptions.pop(name))
                self._object_poses.pop(name, None)
        for name, topic in topics.items():
            if name not in self._object_subscriptions:
                self._object_subscriptions[name] = self.create_subscription(
                    ObjectPose, topic, partial(self.CALLBACK_FN, name), 10)
        return topics

    def CALLBACK_FN(self, NAME, OBJ):

        # Only use messages whose object name agrees with their topic name.
        if NAME in self._object_subscriptions and OBJ.objectname == NAME:
            self._object_poses[NAME] = OBJ

    def getOBJECTS(self):

        # Collect new samples on every activation; never reuse previous poses.
        self._object_poses.clear()
        deadline = time.monotonic() + 0.50
        try:
            while rclpy.ok():
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._refresh_subscriptions()
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                rclpy.spin_once(self, timeout_sec=min(remaining, 0.05))

            topics = self._active_object_topics()
            return [self._object_poses[name] for name in sorted(topics)
                    if name in self._object_poses]
        finally:
            # Remove subscribers between activations so queued messages from a
            # previous collection cannot be mistaken for current object poses.
            for subscription in self._object_subscriptions.values():
                self.destroy_subscription(subscription)
            self._object_subscriptions.clear()
            self._object_poses.clear()

# =============================================================================== #
# Robot(EE) Pose SUBSCRIBER:

class eePOSE(Node):

    def __init__(self):

        super().__init__("ros2srrc_EEGz_eePose_Subscriber")
        self._pose_received = False
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
        self._pose_received = True

    def getEEPose(self):

        global EEPose

        # Require a newly received pose for each activation attempt.
        self._pose_received = False
        deadline = time.monotonic() + 0.50
        while rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            rclpy.spin_once(self, timeout_sec=remaining)
            if self._pose_received:
                return(EEPose)

        return(None)

# =============================================================================== #
# LinkAttacher SERVICE CLIENT:

class LinkAttacher_Client(Node):

    def __init__(self, ROBOT, EE):

        super().__init__("ros2srrc_EEGz_LinkAttacher_Client")

        self.AttachClient = self.create_client(AttachLink, "/ATTACHLINK")
        self.DetachClient = self.create_client(DetachLink, "/DETACHLINK")

        print("[CLIENT - vacuumGripper.py]: Initialising /ATTACHLINK and /DETACHLINK ROS 2 Service Clients.")

        while not self.AttachClient.wait_for_service(timeout_sec=1.0):
            print("[CLIENT - vacuumGripper.py]: /ATTACHLINK ROS 2 Service not still available, waiting...")
        print("[CLIENT - vacuumGripper.py]: /ATTACHLINK ROS 2 Service ready.")
        while not self.DetachClient.wait_for_service(timeout_sec=1.0):
            print("[CLIENT - vacuumGripper.py]: /DETACHLINK ROS 2 Service not still available, waiting...")
        print("[CLIENT - vacuumGripper.py]: /DETACHLINK ROS 2 Service ready.")

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
                    print("[CLIENT - vacuumGripper.py]: /ATTACHLINK Service call failed -> " + str(exc))
                    print("")
                    return(False)
                else:
                    if (AttachRES.success):
                        print("[CLIENT - vacuumGripper.py]: /ATTACHLINK successful -> " + str(AttachRES.message))
                        print("")
                        AttachCheck.ATTACHED = True
                        AttachCheck.NAME = NAME
                        return(True)
                    else:
                        print("[CLIENT - vacuumGripper.py]: /ATTACHLINK unuccessful -> " + str(AttachRES.message))
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
                    print("[CLIENT - vacuumGripper.py]: /DETACHLINK Service call failed -> " + str(exc))
                    print("")
                    return(False)
                else:
                    if (DetachRES.success):
                        print("[CLIENT - vacuumGripper.py]: /DETACHLINK successful -> " + str(DetachRES.message))
                        print("")
                        AttachCheck.ATTACHED = False
                        AttachCheck.NAME = ""
                        return(True)
                    else:
                        print("[CLIENT - vacuumGripper.py]: /DETACHLINK unuccessful -> " + str(DetachRES.message))
                        print("")
                        return(False)

# =============================================================================== #
# vacuumGR class, to ACTIVATE/DEACTIVATE the Vacuum Gripper in Gazebo:

class vacuumGR():

    def __init__(self, ROBOT, EE):

        self.objPoseClient = ObjPOSE()
        self.eePoseClient = eePOSE()
        self.LinkAttacher = LinkAttacher(ROBOT, EE)

    def ACTIVATE(self):

        T_start = time.time()
        RES = {
            "Message": "",
            "Success": False,
            "ExecTime": -1.0
        }

        print("[CLIENT - vacuumGripper.py]: EXECUTION REQUEST -> ACTIVATE VACUUM.")
        print("")

        # Discover all currently publishing /<ObjectName>/ObjectPose topics.
        Objects = self.objPoseClient.getOBJECTS()
        if not Objects:
            RES["Message"] = "Vacuum activated without grasping any object."
            RES["Success"] = True
            RES["ExecTime"] = round(time.time() - T_start, 4)
            print("[CLIENT - vacuumGripper.py]: " + RES["Message"])
            print("")
            return(RES)

        EEPose = self.eePoseClient.getEEPose()
        if EEPose is None:
            RES["Message"] = "Vacuum activation failed: no robot pose received."
            RES["ExecTime"] = round(time.time() - T_start, 4)
            print("[CLIENT - vacuumGripper.py]: " + RES["Message"])
            print("")
            return(RES)

        objNAME = ""
        for x in Objects:

            Check = True
            print("[CLIENT - vacuumGripper.py]: Checking if object is attached to VacuumGripper: " + x.objectname)
            print("[CLIENT - vacuumGripper.py]: EEPose.x -> " + str(EEPose.x) + " / ObjectPose.x -> " + str(x.x))
            print("[CLIENT - vacuumGripper.py]: EEPose.y -> " + str(EEPose.y) + " / ObjectPose.y -> " + str(x.y))
            print("[CLIENT - vacuumGripper.py]: EEPose.z -> " + str(EEPose.z) + " / ObjectPose.z -> " + str(x.z))
            print("")

            if (EEPose.x - 0.01 > x.x) or (EEPose.x + 0.01 < x.x):
                Check = False
            if (EEPose.y - 0.01 > x.y) or (EEPose.y + 0.01 < x.y):
                Check = False
            if (EEPose.z - 0.01 > x.z) or (EEPose.z + 0.01 < x.z):
                Check = False

            if Check:
                objNAME = x.objectname
                break

        if objNAME:
            AttRES = self.LinkAttacher.ATTACH(objNAME)
            if AttRES:
                RES["Message"] = "Vacuum activated, object->" + objNAME + " attached."
                RES["Success"] = True
            else:
                RES["Message"] = "Vacuum activated, object->" + objNAME + " not attached, LinkAttacher plugin failed."
        else:
            RES["Message"] = "Vacuum activated without grasping any object."
            RES["Success"] = True

        print("[CLIENT - vacuumGripper.py]: " + RES["Message"])
        print("")
        RES["ExecTime"] = round(time.time() - T_start, 4)
        return(RES)

    def DEACTIVATE(self):

        T_start = time.time()
        RES = {
            "Message": "",
            "Success": False,
            "ExecTime": -1.0
        }

        print("[CLIENT - vacuumGripper.py]: EXECUTION REQUEST -> DEACTIVATE VACUUM.")
        print("")

        # Detach the held object even if discovery now finds no object topics.
        global AttachCheck
        objNAME = AttachCheck.NAME
        if AttachCheck.ATTACHED:
            DetRES = self.LinkAttacher.DETACH(objNAME)
            if DetRES:
                RES["Message"] = "Vacuum deactivated, object->" + objNAME + " detached."
                RES["Success"] = True
            else:
                RES["Message"] = "Vacuum deactivated, object->" + objNAME + " not detached, LinkAttacher plugin failed."
        else:
            RES["Message"] = "Vacuum deactivated without dropping any object."
            RES["Success"] = True

        print("[CLIENT - vacuumGripper.py]: " + RES["Message"])
        print("")
        RES["ExecTime"] = round(time.time() - T_start, 4)
        return(RES)
