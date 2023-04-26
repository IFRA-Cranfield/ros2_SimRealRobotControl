#include "ros2srrc_execution/movel.h"

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

// MoveL:
geometry_msgs::msg::Pose MoveLAction(ros2srrc_data::msg::Xyz GOAL, geometry_msgs::msg::PoseStamped POSE){

    geometry_msgs::msg::Pose TARGET_POSE = POSE.pose;

    // CALCULATIONS:
    TARGET_POSE.position.x = TARGET_POSE.position.x + GOAL.x;
    TARGET_POSE.position.y = TARGET_POSE.position.y + GOAL.y;
    TARGET_POSE.position.z = TARGET_POSE.position.z + GOAL.z;

    // RETURN RESULT:
    return(TARGET_POSE);

};