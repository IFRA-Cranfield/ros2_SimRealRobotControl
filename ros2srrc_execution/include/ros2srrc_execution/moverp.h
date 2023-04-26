#ifndef MOVERP_H
#define MOVERP_H

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
#include "ros2srrc_data/msg/xyzypr.hpp"

geometry_msgs::msg::Pose MoveRPAction(ros2srrc_data::msg::Xyzypr GOAL, geometry_msgs::msg::PoseStamped POSE);

#endif /* MOVERP_H */