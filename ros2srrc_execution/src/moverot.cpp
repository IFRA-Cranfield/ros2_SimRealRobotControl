#include "ros2srrc_execution/moverot.h"

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
#include "ros2srrc_data/msg/ypr.hpp"

// Declaration of GLOBAL VARIABLES --> CONSTANT VALUES for angle transformation (DEG->RAD):
const double pi = 3.14159265358979;
const double k = pi/180.0;

// MoveROT:
geometry_msgs::msg::Pose MoveROTAction(ros2srrc_data::msg::Ypr GOAL, geometry_msgs::msg::PoseStamped POSE){

    geometry_msgs::msg::Pose TARGET_POSE;

    // Get INITIAL ROTATION -> Quaternion:
    double Ax = POSE.pose.orientation.x;
    double Ay = POSE.pose.orientation.y;
    double Az = POSE.pose.orientation.z;
    double Aw = POSE.pose.orientation.w;

    // Get desired RELATIVE ROTATION -> Given into MoveROT by Euler Angles:
    double cy = cos(k*GOAL.yaw * 0.5);
    double sy = sin(k*GOAL.yaw * 0.5);
    double cp = cos(k*GOAL.pitch * 0.5);
    double sp = sin(k*GOAL.pitch * 0.5);
    double cr = cos(k*GOAL.roll * 0.5);
    double sr = sin(k*GOAL.roll * 0.5);
    double Bx = sr * cp * cy - cr * sp * sy;
    double By = cr * sp * cy + sr * cp * sy;
    double Bz = cr * cp * sy - sr * sp * cy;
    double Bw = cr * cp * cy + sr * sp * sy;

    // QUATERNION MULTIPLICATION:
    double w = Aw*Bw - Ax*Bx - Ay*By - Az*Bz;
    double x = Aw*Bx + Ax*Bw + Ay*Bz - Az*By;
    double y = Aw*By - Ax*Bz + Ay*Bw + Az*Bx;
    double z = Aw*Bz + Ax*By - Ay*Bx + Az*Bw; 

    // TARGET POSE:
    TARGET_POSE.position.x = POSE.pose.position.x;
    TARGET_POSE.position.y = POSE.pose.position.y;
    TARGET_POSE.position.z = POSE.pose.position.z;
    TARGET_POSE.orientation.x = x;
    TARGET_POSE.orientation.y = y;
    TARGET_POSE.orientation.z = z;
    TARGET_POSE.orientation.w = w;

    // RETURN RESULT:
    return(TARGET_POSE);

};