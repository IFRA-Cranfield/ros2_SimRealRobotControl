#include "ros2srrc_execution/movexyzw.h"

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

// Declaration of GLOBAL VARIABLES --> CONSTANT VALUES for angle transformation (DEG->RAD):
const double pi = 3.14159265358979;
const double k = pi/180.0;

// MoveXYZW:
geometry_msgs::msg::Pose MoveXYZWAction(ros2srrc_data::msg::Xyzypr GOAL){

    geometry_msgs::msg::Pose TARGET_POSE;

    // EULER to QUATERNION CONVERSION:
    double cy = cos(k*GOAL.yaw * 0.5);
    double sy = sin(k*GOAL.yaw * 0.5);
    double cp = cos(k*GOAL.pitch * 0.5);
    double sp = sin(k*GOAL.pitch * 0.5);
    double cr = cos(k*GOAL.roll * 0.5);
    double sr = sin(k*GOAL.roll * 0.5);
    double orientationX = sr * cp * cy - cr * sp * sy;
    double orientationY = cr * sp * cy + sr * cp * sy;
    double orientationZ = cr * cp * sy - sr * sp * cy;
    double orientationW = cr * cp * cy + sr * sp * sy;

    // TARGET POSE:
    TARGET_POSE.position.x = GOAL.x;
    TARGET_POSE.position.y = GOAL.y;
    TARGET_POSE.position.z = GOAL.z;
    TARGET_POSE.orientation.x = orientationX;
    TARGET_POSE.orientation.y = orientationY;
    TARGET_POSE.orientation.z = orientationZ;
    TARGET_POSE.orientation.w = orientationW;

    // RETURN RESULT:
    return(TARGET_POSE);

};