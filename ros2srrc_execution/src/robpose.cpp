/*
# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Dr. Seemal Asif  - s.asif@cranfield.ac.uk                                   #
#           Prof. Phil Webb  - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: August, 2023.                                                                  #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023) ROS 2 Sim-to-Real Robot Control. URL: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl.
*/

// RobPose.cpp:

// Required to include ROS2 (C++):
#include "rclcpp/rclcpp.hpp"

// Required for timer:
#include <chrono>
#include <functional>
#include <memory>
#include <string>
using namespace std::chrono_literals;

// Include MoveIt!2:
#include <moveit/move_group_interface/move_group_interface_improved.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Include the Robpose ROS2 Message:
#include "ros2srrc_data/msg/robpose.hpp"

// Declaration of GLOBAL VARIABLE --> MoveIt!2 Interface:
moveit::planning_interface::MoveGroupInterface move_group_interface_ROB;

// Declaration of GLOBAL VARIABLE --> ROBOT PARAMETER:
std::string param_ROB = "none";

// Declaration of GLOBAL VARIABLE --> ROBOT POSE:
ros2srrc_data::msg::Robpose POSE; 

// =============================================================================== //
//  PARAM -> ROBOT:

class ros2_RobotParam : public rclcpp::Node
{
public:
    ros2_RobotParam() : Node("ros2_RobotParam") 
    {
        this->declare_parameter("ROB_PARAM", "none");
        param_ROB = this->get_parameter("ROB_PARAM").get_parameter_value().get<std::string>();
        RCLCPP_INFO(this->get_logger(), "ROB_PARAM received -> %s", param_ROB.c_str());
    }
private:
};

// =============================================================================== //
//  PARAM -> ROBOT:

class RobPose_PUB : public rclcpp::Node
{
public:
  RobPose_PUB()
  : Node("ros2srrc_RobPosePUB"), count_(0)
  {
    publisher_ = this->create_publisher<ros2srrc_data::msg::Robpose>("Robpose", 10);
    timer_ = this->create_wall_timer(50ms, std::bind(&RobPose_PUB::timer_callback, this));
  }

private:

  void timer_callback()
  {

    auto CP_INFO = move_group_interface_ROB.getCurrentPose();

    POSE.x = CP_INFO.pose.position.x;
    POSE.y = CP_INFO.pose.position.y;
    POSE.z = CP_INFO.pose.position.z;
    POSE.qx = CP_INFO.pose.orientation.x;
    POSE.qy = CP_INFO.pose.orientation.y;
    POSE.qz = CP_INFO.pose.orientation.z;
    POSE.qw = CP_INFO.pose.orientation.w;

    publisher_->publish(POSE);

  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ros2srrc_data::msg::Robpose>::SharedPtr publisher_;
  size_t count_;

};

// ===================================================================================== //
// ======================================= MAIN ======================================== //
// ===================================================================================== //

int main(int argc, char **argv)
{

    // Initialise MAIN NODE:
    rclcpp::init(argc, argv);
    
    // Obtain ROBOT parameter:
    auto node_PARAM_ROB = std::make_shared<ros2_RobotParam>();
    rclcpp::spin_some(node_PARAM_ROB);

    // Launch and spin (EXECUTOR) MoveIt!2 Interface node:
    auto name = "ros2srrc_RobPose";
    auto const MoveIt2_NODE = std::make_shared<rclcpp::Node>(name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    rclcpp::executors::SingleThreadedExecutor executor; 
    executor.add_node(MoveIt2_NODE);
    std::thread([&executor]() { executor.spin(); }).detach();

    // MoveGroupInterface_ROB:
    using moveit::planning_interface::MoveGroupInterface;
    auto ROBname = param_ROB + "_arm";
    move_group_interface_ROB = MoveGroupInterface(MoveIt2_NODE, ROBname);

    // SPIN PUBLISHER:
    rclcpp::spin(std::make_shared<RobPose_PUB>());

    rclcpp::shutdown();
    return 0;

}