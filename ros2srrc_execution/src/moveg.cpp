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

// MoveG:
MoveGSTRUCT MoveGAction (double VAL, std::vector<double> JP, std::string param_EE){

    MoveGSTRUCT RESULT;
    
    // 1. Obtain variables:
    double GP = VAL;

    // 2. CALCULATIONS:

    // GRIPPERS in ros2_SimRealRobotControl repository:
    //  - Schunk EGP-64 parallel gripper. NAME -> "egp64"

    // Check GRIPPER LIMITS:
    double GPupper, GPlower = 0.0;
    bool LimitCheck = false;
    if (param_EE == "egp64"){
        GPupper = 0.025;
        GPlower = 0.0;
    };
    if (GP <= GPupper && GP >= GPlower) {
        // Do nothing, check complete.
    } else {
        LimitCheck = true;
    }

    // 3. SET TARGET and RETURN:
    if (LimitCheck == false){
        
        JP[0] = GP;
        JP[1] = GP;

        RESULT.RES = "LIMITS: OK";
        RESULT.JP = JP;

    } else {
        RESULT.RES = "LIMITS: ERROR";
        RESULT.JP = JP;
    }

    // 4. RETURN RESULT:
    return(RESULT);

};