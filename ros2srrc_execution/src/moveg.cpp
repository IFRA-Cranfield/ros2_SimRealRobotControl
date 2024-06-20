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

#include "ros2srrc_execution/moveg.h"

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

// ROS 2 MSG -> SPECIFICATIONS:
#include "ros2srrc_data/msg/specs.hpp"

// MoveG:
MoveGSTRUCT MoveGAction (double VAL, std::vector<double> JP, ros2srrc_data::msg::Specs SPECIFICATIONS){

    MoveGSTRUCT RESULT;
    double GPMax, GPMin;
    std::vector<double> JointsVector;

    GPMax = SPECIFICATIONS.ee_max;
    GPMin = SPECIFICATIONS.ee_min;
    JointsVector = SPECIFICATIONS.ee_vector;

    // 1. CALCULATIONS -> Check VALUE is between 0 and 100:
    if (VAL < 0 || VAL > 100){
        
        RESULT.RES = "Gripper INPUT VALUE is not correct! It should be [0, 100]. Try again.";
        RESULT.JP = JP;

        return(RESULT); // RETURN ERROR message.
    };

    // 2. CONVERT -> VAL to GripperPose value (GP):
    double GP = (GPMax - GPMin) * (VAL/100.0);

    // 3. SET GRIPPER POSE vector:
    for (int i=0; i<JP.size(); i++){
        JP[i] = GP*JointsVector[i];
    };

    RESULT.RES = "LIMITS: OK";
    RESULT.JP = JP;

    // 4. RETURN -> RESULT:
    return(RESULT);

};