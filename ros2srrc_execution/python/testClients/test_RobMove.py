#!/usr/bin/env python3

# test_RobMove.py -> This python script creates an instance of the RBT() class in ros2srrc_execution and executes the RobMove ROS 2 Action.

# ===== IMPORT REQUIRED COMPONENTS ===== #
# System functions and classes:
import sys, os
# Required to include ROS2 and its components:
import rclpy
from ament_index_python.packages import get_package_share_directory

# IMPORT ROS2 Custom Messages:
from ros2srrc_data.msg import Robpose

# IMPORT Python classes:
PATH = os.path.join(get_package_share_directory("ros2srrc_execution"), 'python')
PATH_robot = PATH + "/robot"
# ROBOT CLASS:
sys.path.append(PATH_robot)
from robot import RBT

# ===================================================================== #
# ===================================================================== #

def main(args=None):
    
    rclpy.init(args=args)

    client = RBT()

    # ======================================== #
    # Movement1 -> PointToPoint:

    # Define -> MOVEMENT TYPE: (PTP, LIN)
    MovType = "PTP"
    # Define -> SPEED:
    Speed = 1.0

    # Define -> POSE:     
    InputPose = Robpose()
    InputPose.x = 0.0
    InputPose.y = 0.25
    InputPose.z = 0.95
    InputPose.qx = 0.0
    InputPose.qy = 1.0
    InputPose.qz = 0.0
    InputPose.qw = 0.0

    # Execute:
    RES = client.RobMove_EXECUTE(MovType, Speed, InputPose)
    print("[TEST-ROBOT-RobMove]: Action executed!")
    print("[TEST-ROBOT-RobMove]: Result -> " + RES["Message"])
    print("")

    # ======================================== #
    # Movement2 -> LINEAR:

    # Define -> MOVEMENT TYPE: (PTP, LIN)
    MovType = "LIN"
    # Define -> SPEED:
    Speed = 0.1

    # Define -> POSE:     
    InputPose = Robpose()
    InputPose.x = 0.0
    InputPose.y = 0.25
    InputPose.z = 0.86
    InputPose.qx = 0.0
    InputPose.qy = 1.0
    InputPose.qz = 0.0
    InputPose.qw = 0.0

    # Execute:
    RES = client.RobMove_EXECUTE(MovType, Speed, InputPose)
    print("[TEST-ROBOT-RobMove]: Action executed!")
    print("[TEST-ROBOT-RobMove]: Result -> " + RES["Message"])
    print("")

    rclpy.shutdown()
    print("CLOSING PROGRAM... BYE!")
    exit()

    # ============= # 
    # NOTE: The Robot Poses defined in this script which are the input to RobMove have been tested on a UR3, 
    # therefore these might need to be adjusted if a different robot is used.
    # ============= #

if __name__ == '__main__':
    main()