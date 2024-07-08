#!/usr/bin/env python3

# test_VG.py -> This python script creates an instance of the vacuumGR() class in ros2srrc_execution and operates the VACUUM GRIPPER in Gazebo.

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
from vacuumGripper import vacuumGR

# ===================================================================== #
# ===================================================================== #

def main(args=None):
    
    rclpy.init(args=args)

    # ============================ #
    # Get VARIABLES for CLIENT INIT:

    # Environment variables:
    OBJECTS = ["LamSheet"]
    ROBOT = "irb120"
    EE_link = "EE_ls_vgr"

    # ACTION:
    ACTION = "ACTIVATE"

    # ================ #
    # Initialise CLIENT:
    client = vacuumGR(OBJECTS, ROBOT, EE_link)

    # Execute:
    if ACTION == "ACTIVATE":
        RES = client.ACTIVATE()
    elif ACTION == "DEACTIVATE":
        RES = client.DEACTIVATE()
    else:
        RES = {}
        RES["Message"] = "ERROR: ACTION was not properly defined, it should be ACTIVATE/DEACTIVATE."

    print("[TEST-vGRIPPER-Gz]: Action executed! -> Requested: " + ACTION)
    print("[TEST-vGRIPPER-Gz]: Result -> " + RES["Message"])
    print("") 

    rclpy.shutdown()
    print("CLOSING PROGRAM... BYE!")
    exit()

    # ============= # 
    # NOTE: The ROBOT, END-EFFECTOR and OBJECT(S) defined in this script which are the input to vacuumGR() 
    # have been tested on an ABB IRB120 robot with a Schunk EGP64 gripper and a Lamination Sheet, therefore 
    # these might need to be adjusted if a different robot, end-effector or use-case is used.
    # ============= #

if __name__ == '__main__':
    main()