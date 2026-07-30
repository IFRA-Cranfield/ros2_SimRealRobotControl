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

// Include standard libraries:
#include <string>
#include <vector>
#include <memory>  // for std::unique_ptr

// Include -> YAML file parser:
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

// INCLUDE -> FUNCTIONS:
#include "ros2srrc_execution/movej.h"
#include "ros2srrc_execution/movel.h"
#include "ros2srrc_execution/mover.h"
#include "ros2srrc_execution/moverot.h"
#include "ros2srrc_execution/moverp.h"
#include "ros2srrc_execution/moveg.h"

// Include RCLCPP and RCLCPP_ACTION:
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Include MoveIt!2:
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

// For result codes:
#include "moveit_msgs/msg/move_it_error_codes.hpp"

// Include the move ROS2 ACTION:
#include "ros2srrc_data/action/move.hpp"

// Include the ROS2 MSG messages:
#include "ros2srrc_data/msg/joint.hpp"
#include "ros2srrc_data/msg/joints.hpp"
#include "ros2srrc_data/msg/xyz.hpp"
#include "ros2srrc_data/msg/xyzypr.hpp"
#include "ros2srrc_data/msg/ypr.hpp"
#include "ros2srrc_data/msg/specs.hpp"

// Declaration of GLOBAL VARIABLES --> ROBOT / END-EFFECTOR / MoveGroup_NS PARAMETERS:
std::string param_ROB = "none";
std::string param_EE = "none";
std::string param_mgNS = "";

// Declaration of GLOBAL VARIABLES --> MoveIt!2 Interface:
std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_ROB;

// Declaration of GLOBAL VARIABLES --> JointModelGroup:
const moveit::core::JointModelGroup* joint_model_group_ROB;

// Declaration of GLOBAL VARIABLE --> RES:
std::string RES = "none";

// Declaration of GLOBAL VARIABLES --> robotSPECS and eeSPECS:
ros2srrc_data::msg::Specs robotSPECS;
ros2srrc_data::msg::Specs eeSPECS;
std::vector<std::string> ee_controller_names;
std::vector<std::string> ee_controller_action_namespaces;

// ======================================================================================================================== //
// ==================== PARAM: ROBOT + END-EFFECTOR + MoveGroup_NS ==================== //

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

class ros2_EEParam : public rclcpp::Node
{
public:
    ros2_EEParam() : Node("ros2_EEParam") 
    {
        const std::vector<std::string> empty_string_vector;
        this->declare_parameter("EE_PARAM", "none");
        this->declare_parameter("EE_CONTROLLER_NAMES", empty_string_vector);
        this->declare_parameter("EE_CONTROLLER_ACTION_NAMESPACES", empty_string_vector);
        param_EE = this->get_parameter("EE_PARAM").get_parameter_value().get<std::string>();
        ee_controller_names = this->get_parameter("EE_CONTROLLER_NAMES").get_parameter_value().get<std::vector<std::string>>();
        ee_controller_action_namespaces = this->get_parameter("EE_CONTROLLER_ACTION_NAMESPACES").get_parameter_value().get<std::vector<std::string>>();
        if (ee_controller_action_namespaces.empty() && !ee_controller_names.empty()){
            ee_controller_action_namespaces.assign(ee_controller_names.size(), "gripper_cmd");
        }
        RCLCPP_INFO(this->get_logger(), "EE_PARAM received -> %s", param_EE.c_str());
    }
private:
};

class ros2_mgNSParam : public rclcpp::Node
{
public:
    ros2_mgNSParam() : Node("ros2_mgNSParam")
    {
        this->declare_parameter("move_group_ns", "");
        param_mgNS = this->get_parameter("move_group_ns").get_parameter_value().get<std::string>();
        RCLCPP_INFO(this->get_logger(), "mgNS_PARAM received -> %s", param_mgNS.c_str());
    }
private:
};

// ======================================================================================================================== //
// ==================== FUNCTIONS ==================== //

// ===== PLAN ===== //
// ROBOT:
moveit::planning_interface::MoveGroupInterface::Plan plan_ROB() {
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface_ROB->plan(my_plan) == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

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

// ======================================================================================================================== //
// ==================== ACTION SERVER CLASS ==================== //

class ActionServer : public rclcpp::Node
{
public:
    using Move = ros2srrc_data::action::Move;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Move>;

    explicit ActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("ros2srrc_Move_" + param_mgNS, options)
    {

        action_server_ = rclcpp_action::create_server<Move>(
            this,
            param_mgNS + "/Move",
            std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1));

    }

private:
    rclcpp_action::Server<Move>::SharedPtr action_server_;
    
    // ACCEPT GOAL and NOTIFY which ACTION is going to be exectuted:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Move::Goal> goal)
    {
        (void)uuid;

        // 1. Obtain ACTION type + speed:
        std::string action;
        action = goal->action;
        double speed = goal->speed;

        // 2. Assign VARIABLE TYPE accordingly, and notify:
        if (action == "MoveJ"){
            auto MoveJGoal = goal->movej;
            RCLCPP_INFO(
                this->get_logger(),
                "Received a GOAL REQUEST: MoveJ Action -> speed: %.2f, joints: (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f)",
                speed,
                MoveJGoal.joint1,
                MoveJGoal.joint2,
                MoveJGoal.joint3,
                MoveJGoal.joint4,
                MoveJGoal.joint5,
                MoveJGoal.joint6,
                MoveJGoal.joint7);
        } else if (action == "MoveL"){
            auto MoveLGoal = goal->movel;
            RCLCPP_INFO(
                this->get_logger(),
                "Received a GOAL REQUEST: MoveL Action -> speed: %.2f, xyz: (%.2f, %.2f, %.2f)",
                speed,
                MoveLGoal.x,
                MoveLGoal.y,
                MoveLGoal.z);
        } else if (action == "MoveR"){
            auto MoveRGoal = goal->mover;
            RCLCPP_INFO(
                this->get_logger(),
                "Received a GOAL REQUEST: MoveR Action -> speed: %.2f, joint: %s, value: %.2f",
                speed,
                MoveRGoal.joint.c_str(),
                MoveRGoal.value);
        } else if (action == "MoveROT"){
            auto MoveROTGoal = goal->moverot;
            RCLCPP_INFO(
                this->get_logger(),
                "Received a GOAL REQUEST: MoveROT Action -> speed: %.2f, ypr: (%.2f, %.2f, %.2f)",
                speed,
                MoveROTGoal.yaw,
                MoveROTGoal.pitch,
                MoveROTGoal.roll);
        } else if (action == "MoveRP"){
            auto MoveRPGoal = goal->moverp;
            RCLCPP_INFO(
                this->get_logger(),
                "Received a GOAL REQUEST: MoveRP Action -> speed: %.2f, xyzypr: (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)",
                speed,
                MoveRPGoal.x,
                MoveRPGoal.y,
                MoveRPGoal.z,
                MoveRPGoal.yaw,
                MoveRPGoal.pitch,
                MoveRPGoal.roll);
        } else if (action == "MoveG"){
            RCLCPP_INFO(
                this->get_logger(),
                "Received a GOAL REQUEST: MoveG Action -> speed: %.2f, value: %.2f",
                speed,
                goal->moveg);
        } else {
            RCLCPP_INFO(
                this->get_logger(),
                "Received a GOAL REQUEST: %s Action -> speed: %.2f",
                action.c_str(),
                speed);
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; 
    }

    // No idea about what this function does:
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        // This needs to return quickly to avoid blocking the executor, so spin up a new thread:
        std::thread(
            [this, goal_handle]() {
                execute(goal_handle);
            }).detach();
        
    }

    // Function that cancels the goal request:
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received a cancel request.");

        // We call the -> void moveit::planning_interface::MoveGroupInterface::stop(void) method,
        // which stops any trajectory execution, if one is active.
        if (param_ROB != "none"){
            move_group_interface_ROB->stop();
        }

        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // MAIN LOOP OF THE ACTION SERVER -> EXECUTION:
    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {

        // Obtain ACTION type:
        const auto goal = goal_handle->get_goal();
        std::string action = goal->action;

        // DECLARE RESULT:
        auto result = std::make_shared<Move::Result>();

        // ===== ACTION EXECUTION ===== //
        moveit::planning_interface::MoveGroupInterface::Plan MyPlan;
        
        if (action == "MoveJ" && param_ROB != "none"){
            
            // 1. Define JP VECTOR:
            std::vector<double> JP;
            moveit::core::RobotStatePtr current_state = move_group_interface_ROB->getCurrentState(10);
            current_state->copyJointGroupPositions(joint_model_group_ROB, JP);
            
            // 2. CALL MoveJAction for CALCULATIONS:
            MoveJSTRUCT MoveJRES = MoveJAction(goal->movej, JP, robotSPECS);
            JP = MoveJRES.JP;
            move_group_interface_ROB->setJointValueTarget(JP);
            
            // 3. Assign SPEED and PLANNING METHOD (PTP, LIN, CIRC):
            move_group_interface_ROB->setMaxVelocityScalingFactor(goal->speed);
            move_group_interface_ROB->setPlannerId("PTP");

            // 4. PLAN:
            if (MoveJRES.RES == "LIMITS: OK"){
                MyPlan = plan_ROB();
            } else {
                RES = MoveJRES.RES;
            }

        } else if (action == "MoveL" && param_ROB != "none"){
            
            // 1. Define POSE VECTOR:
            auto POSE = move_group_interface_ROB->getCurrentPose();
            
            // 2. CALL MoveLAction for CALCULATIONS:
            auto TARGET_POSE = MoveLAction(goal->movel, POSE);
            move_group_interface_ROB->setPoseTarget(TARGET_POSE);
            
            // 3. Assign SPEED and PLANNING METHOD (PTP, LIN, CIRC):
            move_group_interface_ROB->setMaxVelocityScalingFactor(goal->speed);
            move_group_interface_ROB->setPlannerId("LIN");

            // 4. PLAN:
            MyPlan = plan_ROB();

        } else if (action == "MoveR" && param_ROB != "none"){

            // 1. Define JP VECTOR:
            std::vector<double> JP;
            moveit::core::RobotStatePtr current_state = move_group_interface_ROB->getCurrentState(10);
            current_state->copyJointGroupPositions(joint_model_group_ROB, JP);
            
            // 2. CALL MoveRAction for CALCULATIONS:
            MoveRSTRUCT MoveRRES = MoveRAction(goal->mover, JP, robotSPECS);
            JP = MoveRRES.JP;
            move_group_interface_ROB->setJointValueTarget(JP);
            
            // 3. Assign SPEED and PLANNING METHOD (PTP, LIN, CIRC):
            move_group_interface_ROB->setMaxVelocityScalingFactor(goal->speed);
            move_group_interface_ROB->setPlannerId("PTP");

            // 4. PLAN:
            if (MoveRRES.RES == "LIMITS: OK"){
                MyPlan = plan_ROB();
            } else {
                RES = MoveRRES.RES;
            }

        } else if (action == "MoveROT" && param_ROB != "none"){
            
            // 1. Define POSE VECTOR:
            auto POSE = move_group_interface_ROB->getCurrentPose();
            
            // 2. CALL MoveROTAction for CALCULATIONS:
            auto TARGET_POSE = MoveROTAction(goal->moverot, POSE);
            move_group_interface_ROB->setPoseTarget(TARGET_POSE);
            
            // 3. Assign SPEED and PLANNING METHOD (PTP, LIN, CIRC):
            move_group_interface_ROB->setMaxVelocityScalingFactor(goal->speed);
            move_group_interface_ROB->setPlannerId("PTP");

            // 4. PLAN:
            MyPlan = plan_ROB();
        
        } else if (action == "MoveRP" && param_ROB != "none"){
            
            // 1. Define POSE VECTOR:
            auto POSE = move_group_interface_ROB->getCurrentPose();
            
            // 2. CALL MoveRPAction for CALCULATIONS:
            auto TARGET_POSE = MoveRPAction(goal->moverp, POSE);
            move_group_interface_ROB->setPoseTarget(TARGET_POSE);
            
            // 3. Assign SPEED and PLANNING METHOD (PTP, LIN, CIRC):
            move_group_interface_ROB->setMaxVelocityScalingFactor(goal->speed);
            move_group_interface_ROB->setPlannerId("PTP");

            // 4. PLAN:
            MyPlan = plan_ROB();
        
        } else if (action == "MoveG" && param_EE != "none"){
            
            // 1. Define JP VECTOR:
            std::vector<double> JP(eeSPECS.ee_vector.size(), 0.0);

            // 2. CALL MoveGAction for CALCULATIONS:
            MoveGSTRUCT MoveGRES = MoveGAction(goal->moveg, JP, eeSPECS);
            JP = MoveGRES.JP;

            if (MoveGRES.RES == "LIMITS: OK"){
                if (ee_controller_names.empty()) {
                    RES = "MoveG direct gripper control is not configured for this end-effector.";
                } else if (ee_controller_action_namespaces.size() != ee_controller_names.size()) {
                    RES = "MoveG controller action namespace count does not match controller count.";
                } else if (ee_controller_names.size() != JP.size()) {
                    RES = "MoveG controller count does not match end-effector joint specification count.";
                } else {
                    auto controller_names = ee_controller_names;
                    if (!param_mgNS.empty()) {
                        for (auto& controller_name : controller_names) {
                            controller_name = param_mgNS + "/" + controller_name;
                        }
                    }

                    bool ExecSUCCESS = send_gripper_commands(
                        this,
                        controller_names,
                        ee_controller_action_namespaces,
                        JP,
                        0.0);

                    if (ExecSUCCESS) {
                        RCLCPP_INFO(this->get_logger(), "%s - %s: Movement executed!", param_EE.c_str(), action.c_str());
                        result->result = action + ":SUCCESS";
                    } else {
                        RCLCPP_INFO(this->get_logger(), "%s - %s: Movement execution failed!", param_EE.c_str(), action.c_str());
                        result->result = action + ":FAILED. Reason -> Gripper action execution error.";
                    }
                    goal_handle->succeed(result);
                    RES = "none";
                    return;
                }
            } else {
                RES = MoveGRES.RES;
            }
        
        }

        // EXECUTE:
        if (RES == "PLANNING: OK"){

            bool ExecSUCCESS = (move_group_interface_ROB->execute(MyPlan) == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal canceled.");
                result->result = action + ":CANCELED";
                goal_handle->canceled(result);
                return;
            } 
            
            if (ExecSUCCESS){
                RCLCPP_INFO(this->get_logger(), "%s - %s: Movement executed!", param_ROB.c_str(), action.c_str());
                result->result = action + ":SUCCESS";
                goal_handle->succeed(result);
            } else {
                RCLCPP_INFO(this->get_logger(), "%s - %s: Movement execution failed!", param_ROB.c_str(), action.c_str());
                result->result = action + ":FAILED. Reason -> Execution error.";
                goal_handle->succeed(result);
            }

        } else if (RES == "PLANNING: ERROR"){
            RCLCPP_INFO(this->get_logger(), "%s - %s: Planning failed!", param_ROB.c_str(), action.c_str());
            result->result = action + ":FAILED. Reason -> Planning failed.";
            goal_handle->succeed(result);

        } else {

            RCLCPP_INFO(this->get_logger(), "ERROR: %s", RES.c_str());
            result->result = action + ":FAILED. Reason -> " + RES;
            goal_handle->succeed(result);

        };

        // RE-INITIALISE RES variable:
        RES = "none";

    }

};


// ==================== MAIN ==================== //

int main(int argc, char ** argv)
{
    // Initialise MAIN NODE:
    rclcpp::init(argc, argv);

    auto node_LOGGER = std::make_shared<rclcpp::Node>("MOVE_INTERFACE_log");

    // Obtain ROBOT + END-EFFECTOR + MG_NS parameters:
    auto node_PARAM_ROB = std::make_shared<ros2_RobotParam>();
    rclcpp::spin_some(node_PARAM_ROB);
    auto node_PARAM_EE = std::make_shared<ros2_EEParam>();
    rclcpp::spin_some(node_PARAM_EE);
    auto node_PARAM_mgNS = std::make_shared<ros2_mgNSParam>();
    rclcpp::spin_some(node_PARAM_mgNS);

    // DEFINE -> RobotSPECS + eeSPECS variables:
    // Robot SPECIFICATIONS:
    if (param_ROB != "none"){
        std::string pkgPATH_R = ament_index_cpp::get_package_share_directory("ros2srrc_robots");
        std::string PATH_R = pkgPATH_R + "/" + param_ROB + "/config/joint_specifications.yaml";
        YAML::Node SPECIFICATIONS_R = YAML::LoadFile(PATH_R);
        robotSPECS.robot_max = SPECIFICATIONS_R["Limits"]["Max"].as<std::vector<double>>();
        robotSPECS.robot_min = SPECIFICATIONS_R["Limits"]["Min"].as<std::vector<double>>();
    };
    // End-Effector SPECIFICATIONS:
    if (param_EE != "none"){
        std::string pkgPATH = ament_index_cpp::get_package_share_directory("ros2srrc_endeffectors");
        std::string PATH = pkgPATH + "/" + param_EE + "/config/joint_specifications.yaml";
        YAML::Node SPECIFICATIONS = YAML::LoadFile(PATH);
        eeSPECS.ee_max = SPECIFICATIONS["Limits"]["Max"].as<double>();
        eeSPECS.ee_min = SPECIFICATIONS["Limits"]["Min"].as<double>();
        eeSPECS.ee_vector =  SPECIFICATIONS["JointsVector"].as<std::vector<double>>();

    };

    // Launch and spin (EXECUTOR) MoveIt!2 Interface node:
    auto name = "ros2srrc_move";
    auto const node2 = std::make_shared<rclcpp::Node>(name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    rclcpp::executors::SingleThreadedExecutor executor; 
    executor.add_node(node2);
    std::thread([&executor]() { executor.spin(); }).detach();

    // CREATE -> MoveGroupInterface(s):
    using moveit::planning_interface::MoveGroupInterface;

    std::string prefix = "";
    if (param_mgNS != ""){
        prefix = param_mgNS + "_";
    }

    // 1. ROBOT:
    if (param_ROB != "none"){
        auto name = prefix + param_ROB + "_arm";
        
        MoveGroupInterface::Options opts(name, "robot_description", param_mgNS);
        move_group_interface_ROB = std::make_unique<MoveGroupInterface>(node2, opts);
        move_group_interface_ROB->setPlanningPipelineId("pilz_industrial_motion_planner");

        move_group_interface_ROB->setMaxVelocityScalingFactor(1.0);
        move_group_interface_ROB->setMaxAccelerationScalingFactor(1.0);

        joint_model_group_ROB = move_group_interface_ROB->getCurrentState()->getJointModelGroup(name);
        RCLCPP_INFO(node_LOGGER->get_logger(), "MoveGroupInterface object created for ROBOT: %s", name.c_str());
    }
    
    // CREATE -> PlanningSceneInterface:
    using moveit::planning_interface::PlanningSceneInterface;
    auto planning_scene_interface = PlanningSceneInterface(param_mgNS);

    // Declare and spin ACTION SERVER:
    auto action_server = std::make_shared<ActionServer>();
    rclcpp::spin(action_server);

    rclcpp::shutdown();
    return 0;
}
