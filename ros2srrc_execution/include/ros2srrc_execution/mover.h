#ifndef MOVER_H
#define MOVER_H

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

struct MoveRSTRUCT {
  std::string RES;
  std::vector<double> JP;
};

MoveRSTRUCT MoveRAction(ros2srrc_data::msg::Joint GOAL, std::vector<double> JP, std::string param_ROB);

#endif /* MOVER_H */