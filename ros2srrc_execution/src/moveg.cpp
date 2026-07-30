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
#  Date: April, 2023.                                                                   #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023) ROS 2 Sim-to-Real Robot Control. URL: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl.
*/

#include "ros2srrc_execution/moveg.h"

// Include standard libraries:
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <string>
#include <vector>

// Include RCLCPP and RCLCPP_ACTION:
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/gripper_command.hpp"

// Include MoveIt!2:
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

// Include the move ROS2 ACTION:
#include "ros2srrc_data/action/move.hpp"

// ROS 2 MSG -> SPECIFICATIONS:
#include "ros2srrc_data/msg/specs.hpp"

// MoveG:
MoveGSTRUCT MoveGAction (double VAL, std::vector<double> JP, ros2srrc_data::msg::Specs SPECIFICATIONS){

    MoveGSTRUCT RESULT;
    double GPMax, GPMin;
    std::vector<double> JointsVector;

    GPMax = SPECIFICATIONS.ee_max;
    GPMin = SPECIFICATIONS.ee_min;
    JointsVector = SPECIFICATIONS.ee_vector;

    // 1. CALCULATIONS -> Check VALUE is between 0 and 100:
    if (VAL < 0 || VAL > 100){
        
        RESULT.RES = "Gripper INPUT VALUE is not correct! It should be [0, 100]. Try again.";
        RESULT.JP = JP;

        return(RESULT); // RETURN ERROR message.
    };

    // 2. CONVERT -> VAL to GripperPose value (GP):
    double GPRange = GPMax - GPMin;
    double GPLimitMargin = std::abs(GPRange) * 0.001;
    double GP = GPMin + GPRange * (VAL/100.0);
    if (VAL >= 100.0){
        GP = GPMax - GPLimitMargin;
    } else if (VAL <= 0.0){
        GP = GPMin + GPLimitMargin;
    }

    // 3. SET GRIPPER POSE vector:
    for (std::size_t i = 0; i < JP.size(); ++i){
        JP[i] = GP * JointsVector[i];
    };

    RESULT.RES = "LIMITS: OK";
    RESULT.JP = JP;

    // 4. RETURN -> RESULT:
    return(RESULT);

};

bool send_gripper_commands(
    rclcpp::Node* node,
    const std::vector<std::string>& controller_names,
    const std::vector<std::string>& action_namespaces,
    const std::vector<double>& positions,
    double max_effort)
{
    using GripperCommand = control_msgs::action::GripperCommand;
    using GripperGoalHandle = rclcpp_action::ClientGoalHandle<GripperCommand>;

    std::vector<rclcpp_action::Client<GripperCommand>::SharedPtr> clients;
    std::vector<std::shared_future<typename GripperGoalHandle::SharedPtr>> goal_futures;

    if (controller_names.size() != action_namespaces.size() || controller_names.size() != positions.size()){
        RCLCPP_ERROR(node->get_logger(), "Gripper command inputs do not have matching sizes.");
        return false;
    }

    for (std::size_t i = 0; i < controller_names.size(); ++i){
        auto client = rclcpp_action::create_client<GripperCommand>(
            node,
            "/" + controller_names[i] + "/" + action_namespaces[i]);

        if (!client->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(node->get_logger(), "Gripper action server not available: %s", controller_names[i].c_str());
            return false;
        }

        GripperCommand::Goal command_goal;
        command_goal.command.position = positions[i];
        command_goal.command.max_effort = max_effort;

        clients.push_back(client);
        goal_futures.push_back(client->async_send_goal(command_goal));
    }

    struct AcceptedGoal {
        rclcpp_action::Client<GripperCommand>::SharedPtr client;
        typename GripperGoalHandle::SharedPtr goal_handle;
        std::string controller_name;
    };

    std::vector<AcceptedGoal> accepted_goals;
    bool success = true;

    for (std::size_t i = 0; i < goal_futures.size(); ++i){
        if (goal_futures[i].wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            RCLCPP_ERROR(node->get_logger(), "Timed out sending gripper goal: %s", controller_names[i].c_str());
            success = false;
            continue;
        }

        auto goal_handle = goal_futures[i].get();
        if (!goal_handle) {
            RCLCPP_ERROR(node->get_logger(), "Gripper goal rejected: %s", controller_names[i].c_str());
            success = false;
            continue;
        }

        accepted_goals.push_back({clients[i], goal_handle, controller_names[i]});
    }

    std::vector<std::shared_future<typename GripperGoalHandle::WrappedResult>> result_futures;
    for (const auto& accepted_goal : accepted_goals){
        result_futures.push_back(accepted_goal.client->async_get_result(accepted_goal.goal_handle));
    }

    for (std::size_t i = 0; i < result_futures.size(); ++i){
        if (result_futures[i].wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
            RCLCPP_ERROR(node->get_logger(), "Timed out waiting for gripper result: %s", accepted_goals[i].controller_name.c_str());
            accepted_goals[i].client->async_cancel_goal(accepted_goals[i].goal_handle);
            success = false;
            continue;
        }

        success = (result_futures[i].get().code == rclcpp_action::ResultCode::SUCCEEDED) && success;
    }

    return success;
}
