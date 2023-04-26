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