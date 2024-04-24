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

// RobMove.cpp:

// Required to include ROS2 and ROS2 Action Server:
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Include the /Robmove ROS2 Action:
#include "ros2srrc_data/action/robmove.hpp"

// Include MoveIt!2:
#include <moveit/move_group_interface/move_group_interface_improved.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Declaration of GLOBAL VARIABLE --> MoveIt!2 Interface:
moveit::planning_interface::MoveGroupInterface move_group_interface_ROB;

// Declaration of GLOBAL VARIABLE --> ROBOT PARAMETER:
std::string param_ROB = "none";

// Declaration of GLOBAL VARIABLE --> RES:
auto RES = "none";

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
// MoveIt!2 -> MoveGroupInterface/Plan function:

moveit::planning_interface::MoveGroupInterface::Plan plan_ROB() {
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface_ROB.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Execute the plan
    if (success)
    {
        RES = "PLANNING: OK";
        return(my_plan);
    }
    else
    {
        RES = "PLANNING: ERROR";
        return(my_plan);
    }

};

// =============================================================================== //
// ROS2 Action Server to move the ROBOT:

class ActionServer : public rclcpp::Node
{

public:
    using Robmove = ros2srrc_data::action::Robmove;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Robmove>;

    explicit ActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("ros2srrc_RobMove", options){

        action_server_ = rclcpp_action::create_server<Robmove>(
            this,
            "/Robmove",
            std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1)
            );

    }

private:
    rclcpp_action::Server<Robmove>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Robmove::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "RobMove (/Robmove) -> RECEIVED A ROBOT MOVEMENT REQUEST:");
        RCLCPP_INFO(get_logger(), "Movement TYPE -> %s", goal->type.c_str());
        RCLCPP_INFO(get_logger(), "Movement SPEED -> %.2f", goal->speed);
        RCLCPP_INFO(get_logger(), "Desired POSITION -> (x: %.3f, y: %.3f, z: %.3f)", goal->x, goal->y, goal->z);
        RCLCPP_INFO(get_logger(), "DESIRED ORIENTATION -> (qx: %.3f, qy: %.3f, qz: %.3f, qw: %.3f)", goal->qx, goal->qy, goal->qz, goal->qw);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread(
            [this, goal_handle]() {
                execute(goal_handle);
            }).detach();
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received a cancel request.");
        move_group_interface_ROB.stop();
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {

        // 0. INFORMATION -> Current Robot Pose:
        auto CP_INFO = move_group_interface_ROB.getCurrentPose();
        RCLCPP_INFO(get_logger(), "INFORMATION -> Current Robot Pose:");
        RCLCPP_INFO(get_logger(), "POSITION -> (x: %.3f, y: %.3f, z: %.3f)", CP_INFO.pose.position.x, CP_INFO.pose.position.y, CP_INFO.pose.position.z);
        RCLCPP_INFO(get_logger(), "ORIENTATION -> (qx: %.3f, qy: %.3f, qz: %.3f, qw: %.3f)", CP_INFO.pose.orientation.x, CP_INFO.pose.orientation.y, CP_INFO.pose.orientation.z, CP_INFO.pose.orientation.w);
        
        // 1. OBTAIN INPUT PARAMETERS:
        const auto GOAL = goal_handle->get_goal();

        // 2. DECLARE RESULT:
        auto RESULT = std::make_shared<Robmove::Result>();

        // 3. Robot Movement -> EXECUTION:

        moveit::planning_interface::MoveGroupInterface::Plan MyPlan;
        
        auto CURRENT_POSE = move_group_interface_ROB.getCurrentPose();

        geometry_msgs::msg::Pose TARGET_POSE;
        TARGET_POSE.position.x = GOAL->x;
        TARGET_POSE.position.y = GOAL->y;
        TARGET_POSE.position.z = GOAL->z;
        TARGET_POSE.orientation.x = GOAL->qx;
        TARGET_POSE.orientation.y = GOAL->qy;
        TARGET_POSE.orientation.z = GOAL->qz;
        TARGET_POSE.orientation.w = GOAL->qw;

        move_group_interface_ROB.setPoseTarget(TARGET_POSE);

        move_group_interface_ROB.setPlannerId(GOAL->type);
        move_group_interface_ROB.setMaxVelocityScalingFactor(GOAL->speed);

        MyPlan = plan_ROB();

        if (RES == "PLANNING: OK"){

            bool ExecSUCCESS = (move_group_interface_ROB.execute(MyPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "ROBOT MOVEMENT (%s) has been CANCELED.", GOAL->type.c_str());
                RESULT->success = false;
                RESULT->message = "RobMove: CANCELED";
                goal_handle->canceled(RESULT);
                return;
            } 
            
            if (ExecSUCCESS){
                RCLCPP_INFO(this->get_logger(), "ROBOT MOVEMENT (%s) successfully executed.", GOAL->type.c_str());
                RESULT->success = true;
                RESULT->message = "RobMove: SUCCESS";
                goal_handle->succeed(RESULT);
            } else {
                RCLCPP_INFO(this->get_logger(), "ROBOT MOVEMENT (%s) failed. Reason -> EXECUTION failure.", GOAL->type.c_str());
                RESULT->success = false;
                RESULT->message = "RobMove: EXECUTION FAILED";
                goal_handle->succeed(RESULT);
            }

        } else {
            RCLCPP_INFO(this->get_logger(), "ROBOT MOVEMENT (%s) failed. Reason -> PLANNING failure.", GOAL->type.c_str());
            RESULT->success = false;
            RESULT->message = "RobMove: PLANNING FAILED";
            goal_handle->succeed(RESULT);
        }

        RES = "none";

    }

};

// ===================================================================================== //
// ======================================= MAIN ======================================== //
// ===================================================================================== //

int main(int argc, char **argv)
{

    // Initialise MAIN NODE:
    rclcpp::init(argc, argv);
    auto const logger = rclcpp::get_logger("RobMove_INTERFACE");

    // Obtain ROBOT parameter:
    auto node_PARAM_ROB = std::make_shared<ros2_RobotParam>();
    rclcpp::spin_some(node_PARAM_ROB);

    // Launch and spin (EXECUTOR) MoveIt!2 Interface node:
    auto name = "ros2srrc_RobMove";
    auto const MoveIt2_NODE = std::make_shared<rclcpp::Node>(name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    rclcpp::executors::SingleThreadedExecutor executor; 
    executor.add_node(MoveIt2_NODE);
    std::thread([&executor]() { executor.spin(); }).detach();

    // MoveGroupInterface_ROB:
    using moveit::planning_interface::MoveGroupInterface;
    auto ROBname = param_ROB + "_arm";
    move_group_interface_ROB = MoveGroupInterface(MoveIt2_NODE, ROBname);
    move_group_interface_ROB.setPlanningPipelineId("move_group");

    move_group_interface_ROB.setMaxVelocityScalingFactor(1.0);
    move_group_interface_ROB.setMaxAccelerationScalingFactor(1.0);
    
    RCLCPP_INFO(logger, "MoveGroupInterface object created for ROBOT: %s", ROBname);

    // CREATE -> PlanningSceneInterface:
    using moveit::planning_interface::PlanningSceneInterface;
    auto planning_scene_interface = PlanningSceneInterface();

    // Declare and spin ACTION SERVER:
    auto action_server = std::make_shared<ActionServer>();
    rclcpp::spin(action_server);

    rclcpp::shutdown();
    return 0;

}