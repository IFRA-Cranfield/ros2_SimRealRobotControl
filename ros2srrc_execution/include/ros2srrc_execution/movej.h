#ifndef MOVEJ_H
#define MOVEJ_H

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

struct MoveJSTRUCT {
  std::string RES;
  std::vector<double> JP;
};

MoveJSTRUCT MoveJAction(ros2srrc_data::msg::Joints JOINTS, std::vector<double> JP, std::string param_ROB);

#endif /* MOVEJ_H */