#include "ros2srrc_execution/movexyz.h"

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
#include "ros2srrc_data/msg/xyz.hpp"

// Declaration of GLOBAL VARIABLES --> CONSTANT VALUES for angle transformation (DEG->RAD):
const double pi = 3.14159265358979;
const double k = pi/180.0;

// MoveXYZ:
geometry_msgs::msg::Pose MoveXYZAction(ros2srrc_data::msg::Xyz GOAL, geometry_msgs::msg::PoseStamped POSE){

    geometry_msgs::msg::Pose TARGET_POSE;

    // TARGET POSE:
    TARGET_POSE.position.x = GOAL.x;
    TARGET_POSE.position.y = GOAL.y;
    TARGET_POSE.position.z = GOAL.z;
    TARGET_POSE.orientation.x = POSE.pose.orientation.x;
    TARGET_POSE.orientation.y = POSE.pose.orientation.y;
    TARGET_POSE.orientation.z = POSE.pose.orientation.z;
    TARGET_POSE.orientation.w = POSE.pose.orientation.w;

    // RETURN RESULT:
    return(TARGET_POSE);

};