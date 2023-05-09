#!/usr/bin/env python3

# Generic ROS2 Python library:
import rclpy
from rclpy.node import Node
import time

# For GAZEBO -> Link States:
from gazebo_msgs.srv import GetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

# For /joint_states:
from sensor_msgs.msg import JointState
k = 180.0/3.14159265359

######################################################################################################

# Create NODE + SERVICE CLIENT for Gz Plugin (GET STATE) service:
class serviceClientGET(Node):
    def __init__(self):
        super().__init__('GraspGET_node')                                                           
        self.cli = self.create_client(GetEntityState, "ros2_grasp/get_entity_state")               
        while not self.cli.wait_for_service(timeout_sec=1.0):                                       
            self.get_logger().info("[wait] get_entity_state not still available, waiting...")
        self.req = GetEntityState.Request()                                                           

    def GET(self, name, ref):
        self.req.name = name
        self.req.reference_frame = ref
        self.future = self.cli.call_async(self.req)    

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

    # 2. MENU:
    #error = True
    #while (error == True):
    #    print("     + Option N1: Get JointValues.")
    #    print("     + Option N2: Get EEPose.")
    #    INPUT = input ("  Please select: ")
    #    if (INPUT == "1"):
    #        error = False
    #        OPTION = "JointValues"
    #    elif (INPUT == "2"):
    #        error = False
    #        OPTION = "EEPose"
    #    else:
    #        print ("  Please select a valid option!")

    OPTION = "JointValues"
    print("")

    # 3. Execute OPERATION:

    #   3.1 - GET JOINT VALUES:
    if (OPTION == "JointValues"):
        
        JointValues_node = CreateSubscriber()
        rclpy.spin_once(JointValues_node)
        
        JointValues_node.destroy_node
        rclpy.shutdown()

    # NOT WORKING! - ROTATION between BASE_TF, TCP and world frame must be considered!
    #   3.2 - GET end-effector POSE:  --  {POSITION only, ORIENTATION to be added in the future}
    elif (OPTION == "EEPose"):
        
        EEPose_node = serviceClientGET()
        
        # A. GET TCP relative to world:
        EEPose_node.GET("TCP", "")
        while rclpy.ok():
            rclpy.spin_once(EEPose_node)
            if EEPose_node.future.done():
                try:
                    response = EEPose_node.future.result()
                except Exception as exc:
                    EEPose_node.get_logger().info("GET STATE: Service call failed: " + str(exc))
                break

        TCP_x = response.state.pose.position.x
        TCP_y = response.state.pose.position.y
        TCP_z = response.state.pose.position.z

        # B. GET BASE relative to world:
        EEPose_node.GET("BASE_TF", "")
        while rclpy.ok():
            rclpy.spin_once(EEPose_node)
            if EEPose_node.future.done():
                try:
                    response = EEPose_node.future.result()
                except Exception as exc:
                    EEPose_node.get_logger().info("GET STATE: Service call failed: " + str(exc))
                break
        
        BASE_x = response.state.pose.position.x
        BASE_y = response.state.pose.position.y
        BASE_z = response.state.pose.position.z

        # C. CALCULATE TCP relative to BASE:
        x = TCP_x - BASE_x
        y = TCP_y - BASE_y
        z = TCP_z - BASE_z

        RESULT = "EEPose is -> 'x': " + str(round(x, 4)) + ", 'y': " + str(round(y, 4)) + ", 'z': " + str(round(z, 4))
        print(RESULT)

if __name__ == "__main__":
    main()