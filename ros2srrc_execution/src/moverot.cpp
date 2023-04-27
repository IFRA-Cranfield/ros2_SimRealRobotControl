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

#include "ros2srrc_execution/moverot.h"

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
#include "ros2srrc_data/msg/ypr.hpp"

// Declaration of GLOBAL VARIABLES --> CONSTANT VALUES for angle transformation (DEG->RAD):
const double pi = 3.14159265358979;
const double k = pi/180.0;

// MoveROT:
geometry_msgs::msg::Pose MoveROTAction(ros2srrc_data::msg::Ypr GOAL, geometry_msgs::msg::PoseStamped POSE){

    geometry_msgs::msg::Pose TARGET_POSE;

    // Get INITIAL ROTATION -> Quaternion:
    double Ax = POSE.pose.orientation.x;
    double Ay = POSE.pose.orientation.y;
    double Az = POSE.pose.orientation.z;
    double Aw = POSE.pose.orientation.w;

    // Get desired RELATIVE ROTATION -> Given into MoveROT by Euler Angles:
    double cy = cos(k*GOAL.yaw * 0.5);
    double sy = sin(k*GOAL.yaw * 0.5);
    double cp = cos(k*GOAL.pitch * 0.5);
    double sp = sin(k*GOAL.pitch * 0.5);
    double cr = cos(k*GOAL.roll * 0.5);
    double sr = sin(k*GOAL.roll * 0.5);
    double Bx = sr * cp * cy - cr * sp * sy;
    double By = cr * sp * cy + sr * cp * sy;
    double Bz = cr * cp * sy - sr * sp * cy;
    double Bw = cr * cp * cy + sr * sp * sy;

    // QUATERNION MULTIPLICATION:
    double w = Aw*Bw - Ax*Bx - Ay*By - Az*Bz;
    double x = Aw*Bx + Ax*Bw + Ay*Bz - Az*By;
    double y = Aw*By - Ax*Bz + Ay*Bw + Az*Bx;
    double z = Aw*Bz + Ax*By - Ay*Bx + Az*Bw; 

    // TARGET POSE:
    TARGET_POSE.position.x = POSE.pose.position.x;
    TARGET_POSE.position.y = POSE.pose.position.y;
    TARGET_POSE.position.z = POSE.pose.position.z;
    TARGET_POSE.orientation.x = x;
    TARGET_POSE.orientation.y = y;
    TARGET_POSE.orientation.z = z;
    TARGET_POSE.orientation.w = w;

    // RETURN RESULT:
    return(TARGET_POSE);

};