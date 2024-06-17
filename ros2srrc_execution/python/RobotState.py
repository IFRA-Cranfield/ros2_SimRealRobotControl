#!/usr/bin/env python3

# Generic ROS2 Python library:
import rclpy
from rclpy.node import Node

# For /joint_states:
from sensor_msgs.msg import JointState
k = 180.0/3.14159265359

######################################################################################################
# Create NODE + SUBSCRIBER for /joint_states:
class CreateSubscriber(Node):

    def __init__(self):
        # Declare NODE:
        super().__init__("r3m_SUBSCRIBER")
        # Declare SUBSCRIBER:
        self.subscription = self.create_subscription(
            JointState,                                                                                                           
            "joint_states",                                                            
            self.listener_callback,                                                   
            10)                                                             
        self.subscription # Prevent unused variable warning.

    def listener_callback(self, MSG):

        l = len(MSG.name)
        for j in range(l):

            if (MSG.name[j] == "shoulder_pan_joint" or MSG.name[j] == "joint_1"):
                J1 = MSG.position[j] * k
            if (MSG.name[j] == "shoulder_lift_joint" or MSG.name[j] == "joint_2"):
                J2 = MSG.position[j] * k
            if (MSG.name[j] == "elbow_joint" or MSG.name[j] == "joint_3"):
                J3 = MSG.position[j] * k
            if (MSG.name[j] == "wrist_1_joint" or MSG.name[j] == "joint_4"):
                J4 = MSG.position[j] * k
            if (MSG.name[j] == "wrist_2_joint" or MSG.name[j] == "joint_5"):
                J5 = MSG.position[j] * k
            if (MSG.name[j] == "wrist_3_joint" or MSG.name[j] == "joint_6"):
                J6 = MSG.position[j] * k
        
        RESULT = "JointValues are -> 'joint1': " + str(round(J1, 4)) + ", 'joint2': " + str(round(J2, 4)) + ", 'joint3': " + str(round(J3, 4)) + ", 'joint4': " + str(round(J4, 4)) + ", 'joint5': " + str(round(J5, 4)) + ", 'joint6': " + str(round(J6, 4))
        print(RESULT)

# ==================================================================================================================================== #
# ==================================================================================================================================== #
# =============================================================== MAIN =============================================================== #
# ==================================================================================================================================== #
# ==================================================================================================================================== #

def main(args=None):

    # 1. INITIALISE ROS NODE:
    rclpy.init(args=args)

    print("")
    print(" --- Cranfield University --- ")
    print("        (c) IFRA Group        ")
    print("")

    print("ros2srrc_execution --> GET ROBOT STATE")
    print("Python script -> RobotState.py")
    print("")
  
    JointValues_node = CreateSubscriber()
    rclpy.spin_once(JointValues_node)
    JointValues_node.destroy_node
    rclpy.shutdown()

if __name__ == "__main__":
    main()