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
MoveJSTRUCT MoveJAction (ros2srrc_data::msg::Joints JOINTS, std::vector<double> JP, std::string param_ROB){

    MoveJSTRUCT RESULT;
    
    // 1. Obtain variables:
    auto MoveJgoal = JOINTS;

    // 2. CALCULATIONS:
    // Declare joint value variables:
    double j1 = MoveJgoal.joint1;
    double j2 = MoveJgoal.joint2;
    double j3 = MoveJgoal.joint3;
    double j4 = MoveJgoal.joint4;
    double j5 = MoveJgoal.joint5;
    double j6 = MoveJgoal.joint6;
    
    double j1UL, j1LL, j2UL, j2LL, j3UL, j3LL, j4UL, j4LL, j5UL, j5LL, j6UL, j6LL = 0.0;

    // ROBOTS in ros2_SimRealRobotControl repository:
    //  - ABB IRB-120 industrial robot manipulator. NAME -> "irb120"

    // ***** JOINT VALUES (MAX/MIN) ***** //
    if (param_ROB == "irb120_arm"){
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
    
    if (j1 <= j1UL && j1 >= j1LL && LimitCheck == false) {
        // Do nothing, check complete.
    } else {
        LimitCheck = true;
    }
    if (j2 <= j2UL && j2 >= j2LL && LimitCheck == false) {
        // Do nothing, check complete.
    } else {
        LimitCheck = true;
    }
    if (j3 <= j3UL && j3 >= j3LL && LimitCheck == false) {
        // Do nothing, check complete.
    } else {
        LimitCheck = true;
    }
    if (j4 <= j4UL && j4 >= j4LL && LimitCheck == false) {
        // Do nothing, check complete.
    } else {
        LimitCheck = true;
    }
    if (j5 <= j5UL && j5 >= j5LL && LimitCheck == false) {
        // Do nothing, check complete.
    } else {
        LimitCheck = true;
    }
    if (j6 <= j6UL && j6 >= j6LL && LimitCheck == false) {
        // Do nothing, check complete.
    } else {
        LimitCheck = true;
    }

    // 4. SET TARGET and RETURN:
    if (LimitCheck == false){
        
        JP[0] = MoveJgoal.joint1 * k;
        JP[1] = MoveJgoal.joint2 * k;
        JP[2] = MoveJgoal.joint3 * k;
        JP[3] = MoveJgoal.joint4 * k;
        JP[4] = MoveJgoal.joint5 * k;
        JP[5] = MoveJgoal.joint6 * k; 

        RESULT.RES = "LIMITS: OK";
        RESULT.JP = JP;

    } else {
        RESULT.RES = "LIMITS: ERROR";
        RESULT.JP = JP;
    }

    // 5. RETURN RESULT:
    return(RESULT);

};