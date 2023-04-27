/*
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
*/

#include "ros2srrc_execution/moverp.h"

// Include standard libraries:
#include <string>
#include <vector>

// Include RCLCPP and RCLCPP_ACTION:
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Include MoveIt!2:
#include <moveit/move_group_interface/move_group_interface_improved.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Include the move ROS2 ACTION:
#include "ros2srrc_data/action/move.hpp"

// Include the ROS2 MSG messages:
#include "ros2srrc_data/msg/xyzypr.hpp"

// Declaration of GLOBAL VARIABLES --> CONSTANT VALUES for angle transformation (DEG->RAD):
const double pi = 3.14159265358979;
const double k = pi/180.0;

// MoveRP:
geometry_msgs::msg::Pose MoveRPAction(ros2srrc_data::msg::Xyzypr GOAL, geometry_msgs::msg::PoseStamped POSE){

    geometry_msgs::msg::Pose TARGET_POSE;

    // Assign values:
    double x = GOAL.x;
    double y = GOAL.y;
    double z = GOAL.z;
    double pitch = GOAL.pitch;
    double yaw = GOAL.yaw;
    double roll = GOAL.roll;

    // ********** ROTATION ********** //

    // Get INITIAL ROTATION -> Quaternion:
    double Ax = POSE.pose.orientation.x;
    double Ay = POSE.pose.orientation.y;
    double Az = POSE.pose.orientation.z;
    double Aw = POSE.pose.orientation.w;
    // Get desired RELATIVE ROTATION -> Given into MoveRP by Euler Angles:
    double cy = cos(k*yaw * 0.5);
    double sy = sin(k*yaw * 0.5);
    double cp = cos(k*pitch * 0.5);
    double sp = sin(k*pitch * 0.5);
    double cr = cos(k*roll * 0.5);
    double sr = sin(k*roll * 0.5);
    double Bx = sr * cp * cy - cr * sp * sy;
    double By = cr * sp * cy + sr * cp * sy;
    double Bz = cr * cp * sy - sr * sp * cy;
    double Bw = cr * cp * cy + sr * sp * sy;
    // QUATERNION MULTIPLICATION:
    double ROTw = Aw*Bw - Ax*Bx - Ay*By - Az*Bz;
    double ROTx = Aw*Bx + Ax*Bw + Ay*Bz - Az*By;
    double ROTy = Aw*By - Ax*Bz + Ay*Bw + Az*Bx;
    double ROTz = Aw*Bz + Ax*By - Ay*Bx + Az*Bw; 


    // ********** TRANSLATION ********** //
    
    // 1. END-EFFECTOR POSE:
    // Rotation quaternion from MoveIt!2:
    // Ax, Ay, Az and Aw.
    // Position from MoveIt!2:
    double Ex = POSE.pose.position.x;
    double Ey = POSE.pose.position.y;
    double Ez = POSE.pose.position.z;
    // 2. Normalise quaternion:
    double norm = sqrt((Ax*Ax)+(Ay*Ay)+(Az*Az)+(Aw*Aw));
    double Qx = Ax/norm;
    double Qy = Ay/norm;
    double Qz = Az/norm;
    double Qw = Aw/norm;
    // 3. Obtain ROTATION MATRIX:
    double R_00 = 1 - 2*(Qy*Qy) - 2*(Qz*Qz);
    double R_01 = 2*(Qx*Qy) - 2*(Qw*Qz);
    double R_02 = 2*(Qx*Qz) + 2*(Qw*Qy);
    double R_10 = 2*(Qx*Qy) + 2*(Qw*Qz);
    double R_11 = 1 - 2*(Qx*Qx) - 2*(Qz*Qz);
    double R_12 = 2*(Qy*Qz) - 2*(Qw*Qx);
    double R_20 = 2*(Qx*Qz) - 2*(Qw*Qy);
    double R_21 = 2*(Qy*Qz) + 2*(Qw*Qx);
    double R_22 = 1 - 2*(Qx*Qx) - 2*(Qy*Qy);
    // 4. From (x,y,z) to (xR,yR,zR) -- ROTATION AROUND A POINT:
    // ROTATION for EULER -> 1.ROLL + 2.PITCH + 3.YAW:
    double rEUL00 = cos(k*yaw)*cos(k*pitch);
    double rEUL01 = cos(k*yaw)*sin(k*pitch)*sin(k*roll) - sin(k*yaw)*cos(k*roll);
    double rEUL02 = cos(k*yaw)*sin(k*pitch)*cos(k*roll) + sin(k*yaw)*sin(k*roll);
    double rEUL10 = sin(k*yaw)*cos(k*pitch);
    double rEUL11 = sin(k*yaw)*sin(k*pitch)*sin(k*roll) + cos(k*yaw)*cos(k*roll);
    double rEUL12 = sin(k*yaw)*sin(k*pitch)*cos(k*roll) - cos(k*yaw)*sin(k*roll);
    double rEUL20 = -sin(k*pitch);
    double rEUL21 = cos(k*pitch)*sin(k*roll);
    double rEUL22 = cos(k*pitch)*cos(k*roll);
    // Displaced End-Effector point -> (xR,yR,zR):
    double xR = x - rEUL00*x - rEUL01*y - rEUL02*z;
    double yR = y - rEUL10*x - rEUL11*y - rEUL12*z;
    double zR = z - rEUL20*x - rEUL21*y - rEUL22*z;
    // 5. From (xR,yR,zR) to (Px,Py,Pz) -- TRANSFORMATION FROM LOCAL (end-effector) to GLOBAL:
    double Px = Ex + R_00*xR + R_01*yR + R_02*zR;
    double Py = Ey + R_10*xR + R_11*yR + R_12*zR;
    double Pz = Ez + R_20*xR + R_21*yR + R_22*zR;

    // TARGET_POSE:
    TARGET_POSE.position.x = Px;
    TARGET_POSE.position.y = Py;
    TARGET_POSE.position.z = Pz;
    TARGET_POSE.orientation.x = ROTx;
    TARGET_POSE.orientation.y = ROTy;
    TARGET_POSE.orientation.z = ROTz;
    TARGET_POSE.orientation.w = ROTw;

    // RETURN RESULT:
    return(TARGET_POSE);

};