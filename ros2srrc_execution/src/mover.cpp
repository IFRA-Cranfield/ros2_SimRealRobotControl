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

#include "ros2srrc_execution/mover.h"

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
#include "ros2srrc_data/msg/joint.hpp"

// Declaration of GLOBAL VARIABLES --> CONSTANT VALUES for angle transformation (DEG->RAD):
const double pi = 3.14159265358979;
const double k = pi/180.0;

// MoveR:
MoveRSTRUCT MoveRAction (ros2srrc_data::msg::Joint GOAL, std::vector<double> JP, ros2srrc_data::msg::Specs SPECIFICATIONS){

    MoveRSTRUCT RESULT;
    
    // 1. VARIABLES:
    auto joint = GOAL.joint;
    auto value = GOAL.value;
    std::vector<double> CURRENT;

    // 2. CALCULATIONS:
    // Obtain current joint values:
    for (int i=0; i<JP.size(); i++){
        
        CURRENT.push_back(JP[i] * (1/k));

    };

    // Joint Limits:
    auto inputOK = true;
    int j;
    if (joint == "joint1"){
        j = 0;
    } else if (joint == "joint2"){
        j = 1;
    } else if (joint == "joint3"){
        j = 2;
    } else if (joint == "joint4"){
        j = 3;
    } else if (joint == "joint5"){
        j = 4;
    } else if (joint == "joint6"){
        j = 5;
    } else if (joint == "joint7"){
        j = 6;
    } else {
        inputOK = false;
    };

    auto VAL = CURRENT[j] + value;
    auto LimitsOK = true;
    if (VAL <= SPECIFICATIONS.robot_max[j] && VAL >= SPECIFICATIONS.robot_min[j]) {
        CURRENT[j] = VAL;
    } else {
        LimitsOK = false;
    };

    // 3. SET TARGET and RETURN:
    if (LimitsOK && inputOK){
        
        for (int i=0; i<JP.size(); i++){
            JP[i] = CURRENT[i] * k;
        };

        RESULT.RES = "LIMITS: OK";
        RESULT.JP = JP;

    } else if (inputOK == false){
        RESULT.RES = "LIMITS: JointName INPUT ERROR";
        RESULT.JP = JP;
    } else {
        RESULT.RES = "LIMITS: ERROR";
        RESULT.JP = JP;
    }

    // 5. RETURN RESULT:
    return(RESULT);

};