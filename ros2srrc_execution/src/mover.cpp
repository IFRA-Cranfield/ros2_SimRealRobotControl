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
MoveRSTRUCT MoveRAction (ros2srrc_data::msg::Joint GOAL, std::vector<double> JP, std::string param_ROB){

    MoveRSTRUCT RESULT;
    
    // 1. Obtain variables:
    auto joint = GOAL.joint;
    auto value = GOAL.value;

    // 2. CALCULATIONS:
    // Obtain current joint values:
    double j1 = JP[0] * (1/k);
    double j2 = JP[1] * (1/k);
    double j3 = JP[2] * (1/k);
    double j4 = JP[3] * (1/k);
    double j5 = JP[4] * (1/k);
    double j6 = JP[5] * (1/k);
    
    double j1UL, j1LL, j2UL, j2LL, j3UL, j3LL, j4UL, j4LL, j5UL, j5LL, j6UL, j6LL = 0.0;

    // ROBOTS in ros2_SimRealRobotControl repository:
    //  - ABB IRB-120 industrial robot manipulator. NAME -> "irb120"

    // ***** JOINT VALUES (MAX/MIN) ***** //
    if (param_ROB == "irb120"){
        j1UL = 165;
        j1LL = -165;
        j2UL = 110;
        j2LL = -110;
        j3UL = 70;
        j3LL = -110;
        j4UL = 160;
        j4LL = -160;
        j5UL = 120;
        j5LL = -120;
        j6UL = 400;
        j6LL = -400;
    };

    // Check if INPUT JOINT VALUES are within the JOINT LIMIT VALUES:
    bool LimitCheck = false;
    
    auto InputJoint = "Valid";

    if (joint == "joint1"){
        j1 = j1 + value;
        if (j1 <= j1UL && j1 >= j1LL && LimitCheck == false) {
            // Do nothing, check complete.
        } else {
            LimitCheck = true;
        }
    } else if (joint == "joint2"){
        j2 = j2 + value;
        if (j2 <= j2UL && j2 >= j2LL && LimitCheck == false) {
            // Do nothing, check complete.
        } else {
            LimitCheck = true;
        }
    } else if (joint == "joint3"){
        j3 = j3 + value;
        if (j3 <= j3UL && j3 >= j3LL && LimitCheck == false) {
            // Do nothing, check complete.
        } else {
            LimitCheck = true;
        }
    } else if (joint == "joint4"){
        j4 = j4 + value;
        if (j4 <= j4UL && j4 >= j4LL && LimitCheck == false) {
            // Do nothing, check complete.
        } else {
            LimitCheck = true;
        }
    } else if (joint == "joint5"){
        j5 = j5 + value;
        if (j5 <= j5UL && j5 >= j5LL && LimitCheck == false) {
            // Do nothing, check complete.
        } else {
            LimitCheck = true;
        }
    } else if (joint == "joint6"){
        j6 = j6 + value;
        if (j6 <= j6UL && j6 >= j6LL && LimitCheck == false) {
            // Do nothing, check complete.
        } else {
            LimitCheck = true;
        }
    } else {
        LimitCheck = true;
        InputJoint = "NotValid";
    }

    // 4. SET TARGET and RETURN:
    if (LimitCheck == false){
        
        JP[0] = j1 * k;
        JP[1] = j2 * k;
        JP[2] = j3 * k;
        JP[3] = j4 * k;
        JP[4] = j5 * k;
        JP[5] = j6 * k;

        RESULT.RES = "LIMITS: OK";
        RESULT.JP = JP;
    } else if (InputJoint = "NotValid"){
        RESULT.RES = "LIMITS: JointName INPUT ERROR";
        RESULT.JP = JP;
    } else {
        RESULT.RES = "LIMITS: ERROR";
        RESULT.JP = JP;
    }

    // 5. RETURN RESULT:
    return(RESULT);

};