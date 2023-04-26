#ifndef MOVEG_H
#define MOVEG_H

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

struct MoveGSTRUCT {
  std::string RES;
  std::vector<double> JP;
};

MoveGSTRUCT MoveGAction(double VAL, std::vector<double> JP, std::string param_EE);

#endif /* MOVEG_H */