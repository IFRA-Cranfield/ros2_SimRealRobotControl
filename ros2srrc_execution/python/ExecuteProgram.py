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

# ===== IMPORT REQUIRED COMPONENTS ===== #
# System functions and classes:
import sys, os, yaml, time
# Required to include ROS2 and its components:
import rclpy
from ament_index_python.packages import get_package_share_directory

# PATH -> Python Classes:
PATH = os.path.join(get_package_share_directory("ros2srrc_execution"), 'python')
PATH_EE = PATH + "/endeffector"
PATH_EEGz = PATH + "/endeffector_gz"
PATH_ROB = PATH + "/robot"

# IMPORT -> ROBOT:
sys.path.append(PATH_ROB)
from robot import RBT

# IMPORT -> EE-Gz:
sys.path.append(PATH_EEGz)
from vacuumGripper import vacuumGR
from parallelGripper import parallelGR

# IMPORT -> EE:
sys.path.append(PATH_EE)
from zimmer_abb import ZimmerGRIPPER
from schunk_abb import SchunkGRIPPER
from robotiq_ur import RobotiqGRIPPER
from vgr_abb import vgrABB

# IMPORT ROS2 Custom Messages:
from ros2srrc_data.msg import Action
from ros2srrc_data.msg import Joint
from ros2srrc_data.msg import Joints
from ros2srrc_data.msg import Xyz
from ros2srrc_data.msg import Xyzypr
from ros2srrc_data.msg import Ypr
from ros2srrc_data.msg import Robpose

# ========================================================================================= #
# =================================== CLASSES/FUNCTIONS =================================== #
# ========================================================================================= #

# ========================================================================================= #           
# Get SEQUENCE from {program}.yaml file:
def getSEQUENCE(packageNAME, yamlNAME):

    RESULT = {}

    PATH = os.path.join(get_package_share_directory(packageNAME), 'programs')
    yamlPATH = PATH + "/" + yamlNAME + ".yaml"

    if not os.path.exists(yamlPATH):
        RESULT["Success"] = False
        return(RESULT)
    
    # Get sequence from YAML:
    with open(yamlPATH, 'r') as YAML:
        seqYAML = yaml.safe_load(YAML)

    RESULT["Sequence"] = seqYAML["Sequence"]
    RESULT["Robot"] = seqYAML["Specifications"]["Robot"]
    RESULT["EEType"] = seqYAML["Specifications"]["EndEffector"]
    RESULT["EELink"] = seqYAML["Specifications"]["EELink"]
    RESULT["Objects"] = seqYAML["Specifications"]["Objects"]
    RESULT["Success"] = True
    
    return(RESULT)

# ========================================================================================= #           
# EVALUATE INPUT ARGUMENTS:
def AssignArgument(ARGUMENT):
    ARGUMENTS = sys.argv
    for y in ARGUMENTS:
        if (ARGUMENT + ":=") in y:
            ARG = y.replace((ARGUMENT + ":="),"")
            return(ARG)

# ========================================================================================= #
# ========================================= MAIN ========================================== #
# ========================================================================================= #
def main(args=None):
    
    rclpy.init(args=args)

    # PRINT - INIT:
    print("==================================================")
    print("ROS 2 Sim-to-Real Robot Control: Program Execution")
    print("==================================================")
    print("")

    # GET PACKAGE NAME:
    PACKAGE = AssignArgument("package")
    if PACKAGE != None:
        None
    else:
        print("ERROR: 'package' INPUT ARGUMENT has not been defined. Please try again.")
        print("Closing program... BYE!")
        exit()

    # GET PROGRAM (SEQUENCE) NAME:
    PROGRAM = AssignArgument("program")
    if PROGRAM != None:
        None
    else:
        print("ERROR: 'program' INPUT ARGUMENT has not been defined. Please try again.")
        print("Closing program... BYE!")
        exit()

    # Get SEQUENCE from {program}.yaml file:
    seqRES = getSEQUENCE(PACKAGE,PROGRAM)
    yamlPATH = "/" + PACKAGE + "/programs/" + PROGRAM + ".yaml"

    if seqRES["Success"] == False:
        print("ERROR: " + yamlPATH + " file not found. Please try again.")
        print("Closing program... BYE!")
        exit()

    # ASSIGN -> SEQUENCE:
    SEQUENCE = seqRES["Sequence"]

    # PRINT - INFORMATION:
    print("PROGRAM -> " + yamlPATH + " found! The sequence is formed by the following steps:")
    print("")

    for x in SEQUENCE:
        print("   - Step Number " + str(x["Step"]) + ":")
        print("     " + x["Name"])
    
    print("")
    print("============================================================")
    print("Loading Robot+EndEffector Python Clients...")
    print("")

    # LOAD ROBOT/EE MOVEMENT PYTHON CLIENTS:
    print("ROBOT: ")
    RobotClient = RBT()
    print("Loaded.")
    print("")
    
    print("END-EFFECTOR:")
    
    if seqRES["EEType"] == "None":
        EEClient = None
        print("Not required.")
    
    elif seqRES["EEType"] == "ParallelGripper":
        EEClient = parallelGR(seqRES["Objects"], seqRES["Robot"], seqRES["EELink"])
        print("Loaded -> ParallelGripper.")
    
    elif seqRES["EEType"] == "VacuumGripper":
        EEClient = vacuumGR(seqRES["Objects"], seqRES["Robot"], seqRES["EELink"])
        print("Loaded -> VacuumGripper.")
    
    elif seqRES["EEType"] == "EGP64/ABB":
        EEClient = SchunkGRIPPER()
        print("Loaded -> EGP64/ABB.")

    elif seqRES["EEType"] == "GPP5010NC/ABB":
        EEClient = ZimmerGRIPPER()
        print("Loaded -> GPP5010NC/ABB.")

    elif seqRES["EEType"] == "vgr/ABB":
        EEClient = vgrABB()
        print("Loaded -> vgr/ABB.")
    
    elif seqRES["EEType"] == "RobotiqHandE/UR":
        EEClient = RobotiqGRIPPER()
        print("Loaded -> RobotiqHandE/UR.")

    print("")

    print("============================================================")
    print("============================================================")
    print("Executing sequence...")
    print("")

    # Initialise -> RES VARIABLE:
    RES = None

    # ==== EXECUTE PROGRAM, STEP BY STEP ===== #
    for x in SEQUENCE:
        
        try:

            print("============================================================")
            print("Step N" + str(x["Step"]) + ": " + x["Name"])
            print("")

            if x["Type"] == "MoveJ":

                ACTION = Action()
                ACTION.action = "MoveJ"
                ACTION.speed = x["Speed"]

                INPUT = Joints()
                INPUT.joint1 = x["Input"]["joint1"]
                INPUT.joint2 = x["Input"]["joint2"]
                INPUT.joint3 = x["Input"]["joint3"]
                INPUT.joint4 = x["Input"]["joint4"]
                INPUT.joint5 = x["Input"]["joint5"]
                INPUT.joint6 = x["Input"]["joint6"]
                ACTION.movej = INPUT

                RES = RobotClient.Move_EXECUTE(ACTION)

            elif x["Type"] == "MoveJ7":

                ACTION = Action()
                ACTION.action = "MoveJ"
                ACTION.speed = x["Speed"]

                INPUT = Joints()
                INPUT.joint1 = x["Input"]["joint1"]
                INPUT.joint2 = x["Input"]["joint2"]
                INPUT.joint3 = x["Input"]["joint3"]
                INPUT.joint4 = x["Input"]["joint4"]
                INPUT.joint5 = x["Input"]["joint5"]
                INPUT.joint6 = x["Input"]["joint6"]
                INPUT.joint7 = x["Input"]["joint7"]
                ACTION.movej = INPUT

                RES = RobotClient.Move_EXECUTE(ACTION)

            elif x["Type"] == "MoveR":

                ACTION = Action()
                ACTION.action = "MoveR"
                ACTION.speed = x["Speed"]

                INPUT = Joint()
                INPUT.joint = x["Input"]["joint"]
                INPUT.value = x["Input"]["value"]
                ACTION.mover = INPUT

                RES = RobotClient.Move_EXECUTE(ACTION)

            elif x["Type"] == "MoveL":

                ACTION = Action()
                ACTION.action = "MoveL"
                ACTION.speed = x["Speed"]

                INPUT = Xyz()
                INPUT.x = x["Input"]["x"]
                INPUT.y = x["Input"]["y"]
                INPUT.z = x["Input"]["z"]
                ACTION.movel = INPUT

                RES = RobotClient.Move_EXECUTE(ACTION)

            elif x["Type"] == "MoveROT":

                ACTION = Action()
                ACTION.action = "MoveROT"
                ACTION.speed = x["Speed"]

                INPUT = Ypr()
                INPUT.pitch = x["Input"]["pitch"]
                INPUT.yaw = x["Input"]["yaw"]
                INPUT.roll = x["Input"]["roll"]
                ACTION.moverot = INPUT

                RES = RobotClient.Move_EXECUTE(ACTION)

            elif x["Type"] == "MoveRP":

                ACTION = Action()
                ACTION.action = "MoveRP"
                ACTION.speed = x["Speed"]

                INPUT = Xyzypr()
                INPUT.x = x["Input"]["x"]
                INPUT.y = x["Input"]["y"]
                INPUT.z = x["Input"]["z"]
                INPUT.pitch = x["Input"]["pitch"]
                INPUT.yaw = x["Input"]["yaw"]
                INPUT.roll = x["Input"]["roll"]
                ACTION.moverp = INPUT

                RES = RobotClient.Move_EXECUTE(ACTION)

            elif x["Type"] == "MoveG":

                ACTION = Action()
                ACTION.action = "MoveG"
                ACTION.speed = x["Speed"]
                ACTION.moveg = x["Input"]["value"]

                RES = RobotClient.Move_EXECUTE(ACTION)

            elif x["Type"] == "RobMove":

                InputPose = Robpose()
                InputPose.x = x["Input"]["x"]
                InputPose.y = x["Input"]["y"]
                InputPose.z = x["Input"]["z"]
                InputPose.qx = x["Input"]["qx"]
                InputPose.qy = x["Input"]["qy"]
                InputPose.qz = x["Input"]["qz"]
                InputPose.qw = x["Input"]["qw"]

                RES = RobotClient.RobMove_EXECUTE(x["Movement"], x["Speed"], InputPose)

            elif x["Type"] == "ParallelGripper":

                if x["Action"] == "CLOSE":
                    RES = EEClient.CLOSE(x["Value"])
                else:
                    RES = EEClient.OPEN()
            
            elif x["Type"] == "VacuumGripper":

                if x["Action"] == "ACTIVATE":
                    RES = EEClient.ACTIVATE()
                else:
                    RES = EEClient.DEACTIVATE()

            elif x["Type"] == "EGP64/ABB":

                if x["Action"] == "CLOSE":
                    RES = EEClient.CLOSE()
                else:
                    RES = EEClient.OPEN()

            elif x["Type"] == "GPP5010NC/ABB":

                if x["Action"] == "CLOSE":
                    RES = EEClient.CLOSE()
                else:
                    RES = EEClient.OPEN()

            elif x["Type"] == "vgr/ABB":

                if x["Action"] == "ACTIVATE":
                    RES = EEClient.ACTIVATE()
                else:
                    RES = EEClient.DEACTIVATE()

            elif x["Type"] == "RobotiqHandE/UR":

                if x["Action"] == "CLOSE":
                    RES = EEClient.CLOSE()
                else:
                    RES = EEClient.OPEN()

            else:
                print("ERROR: ACTION TYPE -> " + x["Type"] + " unknown.")
                print("Closing program... BYE!")
                exit()

            # CHECK if STEP EXECUTION WAS SUCCESSFUL:
            print("")
            
            if RES["Success"] == False:
                print("ERROR: Execution FAILED!")
                print("Message -> " + RES["Message"])
                print("")
                print("Closing... BYE!")
                exit()

            else:
                print("Execution SUCCESSFUL!")
                print("Message -> " + RES["Message"])
                print("")
                
            # ADD -> DELAY:
            if x["Delay"] != 0.0:
                print("Requested a waitTime of " + str(x["Delay"]) + " seconds.")
                time.sleep(x["Delay"])
                print("")
                
        except KeyboardInterrupt:
            
            # CANCEL ANY ONGOING ROBOT MOVEMENTS:
            RobotClient.CANCEL()
            
            print("Sequence execution manually interrupted and cancelled.")
            print("Closing... BYE!")
            exit()

    # ==== FINISH ===== #
    print("")
    print("")
    print("Sequence successfully executed. Closing Program... Bye!")
    print("=======================================================")

    rclpy.shutdown()
    exit()

if __name__ == '__main__':
    main()