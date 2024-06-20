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

#include "ros2srrc_execution/movej.h"

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
#include "ros2srrc_data/msg/joints.hpp"

// Declaration of GLOBAL VARIABLES --> CONSTANT VALUES for angle transformation (DEG->RAD):
const double pi = 3.14159265358979;
const double k = pi/180.0;

// MoveJ:
MoveJSTRUCT MoveJAction (ros2srrc_data::msg::Joints JOINTS, std::vector<double> JP, ros2srrc_data::msg::Specs SPECIFICATIONS){
    
    MoveJSTRUCT RESULT;

    // 1. Obtain variables -> Convert to VECTOR:
    std::vector<double> GOAL;
    GOAL.push_back(JOINTS.joint1);
    GOAL.push_back(JOINTS.joint2);
    GOAL.push_back(JOINTS.joint3);
    if (JP.size() >= 4){
        GOAL.push_back(JOINTS.joint4);
    };
    if (JP.size() >= 5){
        GOAL.push_back(JOINTS.joint5);
    };
    if (JP.size() >= 6){
        GOAL.push_back(JOINTS.joint6);
    };
    if (JP.size() >= 7){
        GOAL.push_back(JOINTS.joint7);
    };

    // 2. CALCULATIONS -> Joint Limits:
    auto LimitsOK = true;
    std::vector<std::string> jointLIST;
    for (int i=0; i<JP.size(); i++){
        
        if (GOAL[i] <= SPECIFICATIONS.robot_max[i] && GOAL[i] >= SPECIFICATIONS.robot_min[i]) {
        // Do nothing, check complete.
        } else {
            LimitsOK = false;
            jointLIST.push_back("joint" + std::to_string(i+1));
        };

    };

    // 3. SET TARGET and RETURN:
    if (LimitsOK == true){

        for (int i=0; i<JP.size(); i++){
            JP[i] = GOAL[i] * k;
        };

        RESULT.RES = "LIMITS: OK";
        RESULT.JP = JP;

    } else {

        std::string OUTPUT = "[";
        for (int k=0; k<jointLIST.size(); k++){
            OUTPUT = OUTPUT + jointLIST[k] + ", ";
        };
        OUTPUT.pop_back();
        OUTPUT.pop_back();
        OUTPUT = OUTPUT + "]";

        RESULT.RES = "For the requested input, the following joints are outside their limit -> " + OUTPUT;
        RESULT.JP = JP;
    }

    // 4. RETURN RESULT:
    return(RESULT);

};