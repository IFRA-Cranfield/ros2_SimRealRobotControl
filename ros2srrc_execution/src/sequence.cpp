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
#include "std_msgs/msg/string.hpp"
#include <vector>
#include <unistd.h> 
#include <ctime>

// Include -> YAML file parser:
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Include -> FUNCTIONS:
#include "ros2srrc_execution/movej.h"
#include "ros2srrc_execution/movel.h"
#include "ros2srrc_execution/mover.h"
#include "ros2srrc_execution/moverot.h"
#include "ros2srrc_execution/moverp.h"
#include "ros2srrc_execution/moveg.h"

// Include for ATTACHER/DETACHER:
#include <linkattacher_msgs/srv/attach_link.hpp>   
#include <linkattacher_msgs/srv/detach_link.hpp>

// Include for ABB I/O manipulation:
#include <abb_robot_msgs/srv/set_io_signal.hpp>
#include <ros2_robotiqgripper/srv/robotiq_gripper.hpp>

// Include RCLCPP and RCLCPP_ACTION:
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Include MoveIt!2:
#include <moveit/move_group_interface/move_group_interface_improved.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Include the SEQUENCE() ROS2 ACTION:
#include "ros2srrc_data/action/sequence.hpp"

// Include the ROS2 MSG messages:
#include "ros2srrc_data/msg/joint.hpp"
#include "ros2srrc_data/msg/joints.hpp"
#include "ros2srrc_data/msg/xyz.hpp"
#include "ros2srrc_data/msg/xyzypr.hpp"
#include "ros2srrc_data/msg/ypr.hpp"
#include "ros2srrc_data/msg/linkattacher.hpp"
#include "ros2srrc_data/msg/specs.hpp"

// Declaration of GLOBAL VARIABLES --> ROBOT / END-EFFECTOR / ENVIRONMENT PARAMETERS:
std::string param_ROB = "none";
std::string param_EE = "none";
std::string param_ENV = "none";

// Declaration of GLOBAL VARIABLES --> MoveIt!2 Interface:
moveit::planning_interface::MoveGroupInterface move_group_interface_ROB;
moveit::planning_interface::MoveGroupInterface move_group_interface_EE;

// Declaration of GLOBAL VARIABLES --> JointModelGroup:
const moveit::core::JointModelGroup* joint_model_group_ROB;
const moveit::core::JointModelGroup* joint_model_group_EE;

// Declaration of GLOBAL VARIABLE --> RES:
std::string RES = "none";

// Declaration of GLOBAL VARIABLES --> Attacher & Detacher / Gripper nodes:
std::shared_ptr<rclcpp::Node> AttacherNode;
std::shared_ptr<rclcpp::Node> DetacherNode;
std::shared_ptr<rclcpp::Node> ABBGripperNode;
std::shared_ptr<rclcpp::Node> URRobotiqGripperNode;

// Declaration of GLOBAL VARIABLES --> robotSPECS and eeSPECS:
ros2srrc_data::msg::Specs robotSPECS;
ros2srrc_data::msg::Specs eeSPECS;

// ======================================================================================================================== //
// ==================== PARAM: ROBOT + END-EFFECTOR ==================== //

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
        this->declare_parameter("EE_PARAM", "none");
        param_EE = this->get_parameter("EE_PARAM").get_parameter_value().get<std::string>();
        RCLCPP_INFO(this->get_logger(), "EE_PARAM received -> %s", param_EE.c_str());
    }
private:
};

class ros2_EnvironmentParam : public rclcpp::Node
{
public:
    ros2_EnvironmentParam() : Node("ros2_EnvironmentParam") 
    {
        this->declare_parameter("ENV_PARAM", "none");
        param_ENV = this->get_parameter("ENV_PARAM").get_parameter_value().get<std::string>();
        RCLCPP_INFO(this->get_logger(), "ENV_PARAM received -> %s", param_ENV.c_str());
    }
private:
};


// ======================================================================================================================== //
// ==================== FUNCTIONS ==================== //

// ===== PLAN ===== //
// ROBOT:
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
// END-EFFECTOR:
moveit::planning_interface::MoveGroupInterface::Plan plan_EE() {
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface_EE.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Execute the plan
    if (success)
    {
        RES = "PLANNING: OK";
        return(my_plan);
    }
    else
    {
        RES = "PLANNING: ERROR (EE)";
        return(my_plan);
    }
    
};


// ======================================================================================================================== //
// ==================== ATTACHER/DETACHER ==================== //

void AttachDetach_NODE(){

    AttacherNode = rclcpp::Node::make_shared("ATTACHER_SC_node");
    DetacherNode = rclcpp::Node::make_shared("DETACHER_SC_node");

}

bool ATTACH(ros2srrc_data::msg::Linkattacher REQ){

    auto ATTACHER_SC = AttacherNode->create_client<linkattacher_msgs::srv::AttachLink>("ATTACHLINK");
    auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();

    request->model1_name = REQ.model1_name;
    request->link1_name = REQ.link1_name;
    request->model2_name = REQ.model2_name;
    request->link2_name = REQ.link2_name;

    auto result = ATTACHER_SC->async_send_request(request);

    if (rclcpp::spin_until_future_complete(AttacherNode, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto RES = result.get();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MSG: %s", RES->message.c_str());
        if (bool attachOK = RES->success) {
            return true;
        } else {
            return false;
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service /ATTACHLINK");
        return false;
    }

}

bool DETACH(ros2srrc_data::msg::Linkattacher REQ){
    
    auto DETACHER_SC = DetacherNode->create_client<linkattacher_msgs::srv::DetachLink>("DETACHLINK");
    auto request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();

    request->model1_name = REQ.model1_name;
    request->link1_name = REQ.link1_name;
    request->model2_name = REQ.model2_name;
    request->link2_name = REQ.link2_name;

    auto result = DETACHER_SC->async_send_request(request);

    if (rclcpp::spin_until_future_complete(DetacherNode, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto RES = result.get();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MSG: %s", RES->message.c_str());
        if (bool detachOK = RES->success) {
            return true;
        } else {
            return false;
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service /DETACHLINK");
        return false;
    }
    
}


// ======================================================================================================================== //
// ==================== ABB GRIPPER: Open/Close ==================== //

void ABBGripper_NODE(){

    ABBGripperNode = rclcpp::Node::make_shared("ABBGripper_SC_node");

}

void GripperOpenABB(){

    auto GRIPPER_SC = ABBGripperNode->create_client<abb_robot_msgs::srv::SetIOSignal>("/rws_client/set_io_signal");
    auto req_msg = std::make_shared<abb_robot_msgs::srv::SetIOSignal::Request>();
    
    // 1. GripperClose -> "0":
    req_msg->signal = "CloseGripper";
    req_msg->value = "0";
    auto REQUEST = GRIPPER_SC->async_send_request(req_msg);
    rclcpp::spin_some(ABBGripperNode);

    // 1. GripperOpen -> "1":
    req_msg->signal = "OpenGripper";
    req_msg->value = "1";
    REQUEST = GRIPPER_SC->async_send_request(req_msg);
    rclcpp::spin_some(ABBGripperNode);

}

void GripperCloseABB(){
    
    auto GRIPPER_SC = ABBGripperNode->create_client<abb_robot_msgs::srv::SetIOSignal>("/rws_client/set_io_signal");
    auto req_msg = std::make_shared<abb_robot_msgs::srv::SetIOSignal::Request>();

    // 1. GripperOpen -> "0":
    req_msg->signal = "OpenGripper";
    req_msg->value = "0";
    auto REQUEST = GRIPPER_SC->async_send_request(req_msg);
    rclcpp::spin_some(ABBGripperNode);

    // 1. GripperClose -> "1":
    req_msg->signal = "CloseGripper";
    req_msg->value = "1";
    REQUEST = GRIPPER_SC->async_send_request(req_msg);
    rclcpp::spin_some(ABBGripperNode);
    
}


// ======================================================================================================================== //
// ==================== UR Robotiq GRIPPER: Open/Close ==================== //

void URRobotiqGripper_NODE(){

    URRobotiqGripperNode = rclcpp::Node::make_shared("URRobotiqGripper_SC_node");

}

void GripperOpenURRobotiq(){

    auto GRIPPER_SC = URRobotiqGripperNode->create_client<ros2_robotiqgripper::srv::RobotiqGripper>("/Robotiq_Gripper");
    auto req_msg = std::make_shared<ros2_robotiqgripper::srv::RobotiqGripper::Request>();

    // GripperOpen:
    req_msg->action = "OPEN";
    auto REQUEST = GRIPPER_SC->async_send_request(req_msg);
    rclcpp::spin_some(URRobotiqGripperNode);

}

void GripperCloseURRobotiq(){

    auto GRIPPER_SC = URRobotiqGripperNode->create_client<ros2_robotiqgripper::srv::RobotiqGripper>("/Robotiq_Gripper");
    auto req_msg = std::make_shared<ros2_robotiqgripper::srv::RobotiqGripper::Request>();

    // GripperCLose:
    req_msg->action = "CLOSE";
    auto REQUEST = GRIPPER_SC->async_send_request(req_msg);
    rclcpp::spin_some(URRobotiqGripperNode);

}


// ======================================================================================================================== //
// ==================== ACTION SERVER CLASS ==================== //

class ActionServer : public rclcpp::Node
{
public:
    using Sequence = ros2srrc_data::action::Sequence;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Sequence>;

    explicit ActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("SEQUENCE_INTERFACE", options)
    {

        action_server_ = rclcpp_action::create_server<Sequence>(
            this,
            "/Sequence",
            std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1));

    }

private:
    rclcpp_action::Server<Sequence>::SharedPtr action_server_;
    
    // ACCEPT GOAL and NOTIFY which ACTION is going to be exectuted:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Sequence::Goal> goal)
    {
        // Obtain ROBOT and END-EFFECTOR -> Check and accept/reject:
        std::string ROB = goal->robot;
        std::string EE = goal->endeffector;
        std::string ENV = goal->environment;

        if (ROB != param_ROB || EE != param_EE || ENV != param_ENV){
            RCLCPP_INFO(this->get_logger(), "ERROR: The following parameters must match in both the sequence request and the action server:");
            RCLCPP_INFO(this->get_logger(), "   - ROBOT: Sequence request -> %s / Action Server -> %s", ROB.c_str(), param_ROB.c_str());
            RCLCPP_INFO(this->get_logger(), "   - End-effector: Sequence request -> %s / Action Server -> %s", EE.c_str(), param_EE.c_str());
            RCLCPP_INFO(this->get_logger(), "   - Environment: Sequence request -> %s / Action Server -> %s", ENV.c_str(), param_ENV.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        } else {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; 
        }

        
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
            move_group_interface_ROB.stop();
        }
        if (param_EE != "none" && param_ENV != "bringup"){
            move_group_interface_EE.stop();
        }

        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // MAIN LOOP OF THE ACTION SERVER -> EXECUTION:
    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {

        // Obtain SEQUENCE:
        const auto goal = goal_handle->get_goal();
        std::vector<ros2srrc_data::msg::Action> SEQ;
        SEQ = goal->sequence;

        // DECLARE FEEDBACK AND RESULT:
        auto feedback = std::make_shared<Sequence::Feedback>();
        auto & feedback_msg = feedback->feedback;
        auto result = std::make_shared<Sequence::Result>();

        // DECLARE PLAN:
        moveit::planning_interface::MoveGroupInterface::Plan MyPlan;

        // ===== SEQUENCE EXECUTION ===== //
        int i = 1;
        bool CONTINUE = true;
        for (ros2srrc_data::msg::Action STEP : SEQ){

            if (CONTINUE == true){

                // a) Publish feedback -> MOTION to be executed:
                std::string ACTION = STEP.action;
                feedback_msg = "";
                goal_handle->publish_feedback(feedback);
                feedback_msg = " ==================== {STEP " + std::to_string(i) + "}: " + ACTION + " ==================== ";
                goal_handle->publish_feedback(feedback);

                // b) PLAN:
                if (ACTION == "MoveJ"){
                
                    // 1. Define JP VECTOR:
                    std::vector<double> JP;
                    moveit::core::RobotStatePtr current_state = move_group_interface_ROB.getCurrentState(10);
                    current_state->copyJointGroupPositions(joint_model_group_ROB, JP);
                    
                    // 2. CALL MoveJAction for CALCULATIONS:
                    MoveJSTRUCT MoveJRES = MoveJAction(STEP.movej, JP, robotSPECS);
                    JP = MoveJRES.JP;
                    move_group_interface_ROB.setJointValueTarget(JP);
                    
                    // 3. Assign SPEED and PLANNING METHOD (PTP, LIN, CIRC):
                    move_group_interface_ROB.setMaxVelocityScalingFactor(STEP.speed);
                    move_group_interface_ROB.setPlannerId("PTP");

                    // 4. PLAN:
                    if (MoveJRES.RES == "LIMITS: OK"){
                        MyPlan = plan_ROB();
                    } else {
                        RES = "LIMITS: ERROR";
                        sleep(1.0); 
                    }
                
                } else if (ACTION == "MoveL"){
            
                    // 1. Define POSE VECTOR:
                    auto POSE = move_group_interface_ROB.getCurrentPose();
                    
                    // 2. CALL MoveLAction for CALCULATIONS:
                    auto TARGET_POSE = MoveLAction(STEP.movel, POSE);
                    move_group_interface_ROB.setPoseTarget(TARGET_POSE);
                    
                    // 3. Assign SPEED and PLANNING METHOD (PTP, LIN, CIRC):
                    move_group_interface_ROB.setMaxVelocityScalingFactor(STEP.speed);
                    move_group_interface_ROB.setPlannerId("LIN");

                    // 4. PLAN:
                    MyPlan = plan_ROB();

                } else if (ACTION == "MoveR"){

                    // 1. Define JP VECTOR:
                    std::vector<double> JP;
                    moveit::core::RobotStatePtr current_state = move_group_interface_ROB.getCurrentState(10);
                    current_state->copyJointGroupPositions(joint_model_group_ROB, JP);
                    
                    // 2. CALL MoveRAction for CALCULATIONS:
                    MoveRSTRUCT MoveRRES = MoveRAction(STEP.mover, JP, robotSPECS);
                    JP = MoveRRES.JP;
                    move_group_interface_ROB.setJointValueTarget(JP);
                    
                    // 3. Assign SPEED and PLANNING METHOD (PTP, LIN, CIRC):
                    move_group_interface_ROB.setMaxVelocityScalingFactor(STEP.speed);
                    move_group_interface_ROB.setPlannerId("PTP");

                    // 4. PLAN:
                    if (MoveRRES.RES == "LIMITS: OK"){
                        MyPlan = plan_ROB();
                    } else {
                        RES = "LIMITS: ERROR";
                        sleep(1.0);
                    }

                } else if (ACTION == "MoveROT"){
                    
                    // 1. Define POSE VECTOR:
                    auto POSE = move_group_interface_ROB.getCurrentPose();
                    
                    // 2. CALL MoveROTAction for CALCULATIONS:
                    auto TARGET_POSE = MoveROTAction(STEP.moverot, POSE);
                    move_group_interface_ROB.setPoseTarget(TARGET_POSE);
                    
                    // 3. Assign SPEED and PLANNING METHOD (PTP, LIN, CIRC):
                    move_group_interface_ROB.setMaxVelocityScalingFactor(STEP.speed);
                    move_group_interface_ROB.setPlannerId("PTP");

                    // 4. PLAN:
                    MyPlan = plan_ROB();
                
                } else if (ACTION == "MoveRP"){
                    
                    // 1. Define POSE VECTOR:
                    auto POSE = move_group_interface_ROB.getCurrentPose();
                    
                    // 2. CALL MoveRPAction for CALCULATIONS:
                    auto TARGET_POSE = MoveRPAction(STEP.moverp, POSE);
                    move_group_interface_ROB.setPoseTarget(TARGET_POSE);
                    
                    // 3. Assign SPEED and PLANNING METHOD (PTP, LIN, CIRC):
                    move_group_interface_ROB.setMaxVelocityScalingFactor(STEP.speed);
                    move_group_interface_ROB.setPlannerId("PTP");

                    // 4. PLAN:
                    MyPlan = plan_ROB();
                
                } else if (ACTION == "MoveG"){
                    
                    // 1. Define JP VECTOR:
                    std::vector<double> JP;
                    moveit::core::RobotStatePtr current_state = move_group_interface_EE.getCurrentState(10);
                    current_state->copyJointGroupPositions(joint_model_group_EE, JP);
                    
                    // 2. CALL MoveGAction for CALCULATIONS:
                    MoveGSTRUCT MoveGRES = MoveGAction(STEP.moveg, JP, eeSPECS);
                    JP = MoveGRES.JP;
                    move_group_interface_EE.setJointValueTarget(JP);
                    
                    // 3. Assign SPEED and PLANNING METHOD (PTP, LIN, CIRC):
                    move_group_interface_EE.setMaxVelocityScalingFactor(STEP.speed);
                    move_group_interface_EE.setPlannerId("PTP");

                    // 4. PLAN:
                    if (MoveGRES.RES == "LIMITS: OK"){
                        MyPlan = plan_EE();
                    } else {
                        RES = "LIMITS: ERROR (EE)";
                        sleep(1.0);
                    }
                
                } else if (ACTION == "Attach"){

                    // ATTACH/DETACH:
                    // This happens when an object needs to be attached to an end-effector in Gazebo Simulation (using IFRA_LinkAttacher):

                    bool success = ATTACH(STEP.attach);

                    if (success){
                        feedback_msg = "{STEP " + std::to_string(i) + "}: " + ACTION + ":Object attached successfully.";
                        goal_handle->publish_feedback(feedback);
                    } else {
                        feedback_msg = "{STEP " + std::to_string(i) + "}: " + ACTION + ":ERROR attaching object.";
                        goal_handle->publish_feedback(feedback);
                        CONTINUE = false;
                    }

                } else if (ACTION == "Detach"){

                    bool success = DETACH(STEP.detach);

                    if (success){
                        feedback_msg = "{STEP " + std::to_string(i) + "}: " + ACTION + ":Object detached successfully.";
                        goal_handle->publish_feedback(feedback);
                    } else {
                        feedback_msg = "{STEP " + std::to_string(i) + "}: " + ACTION + ":ERROR detaching object.";
                        goal_handle->publish_feedback(feedback);
                        CONTINUE = false;
                    }
                    
                }else if (ACTION == "ABB/EGP64 - OPEN"){

                    // Schunk EGP Gripper OPEN/CLOSE through ABB Robot Controller I/O:

                    usleep(200000);
                    GripperOpenABB();
                    usleep(500000);
                    feedback_msg = "{STEP " + std::to_string(i) + "}: " + ACTION + ":Gripper opened successfully.";
                    goal_handle->publish_feedback(feedback);

                } else if (ACTION == "ABB/EGP64 - CLOSE"){

                    usleep(200000);
                    GripperCloseABB();
                    usleep(500000);
                    feedback_msg = "{STEP " + std::to_string(i) + "}: " + ACTION + ":Gripper closed successfully.";
                    goal_handle->publish_feedback(feedback);

                } else if (ACTION == "UR/RobotiqHandE - OPEN"){

                    // Robotiq HandE Gripper OPEN/CLOSE through UR Robot Controller I/O:

                    usleep(200000);
                    GripperOpenURRobotiq();
                    usleep(500000);
                    feedback_msg = "{STEP " + std::to_string(i) + "}: " + ACTION + ":Gripper opened successfully.";
                    goal_handle->publish_feedback(feedback);

                } else if (ACTION == "UR/RobotiqHandE - CLOSE"){

                    usleep(200000);
                    GripperCloseURRobotiq();
                    usleep(500000);
                    feedback_msg = "{STEP " + std::to_string(i) + "}: " + ACTION + ":Gripper closed successfully.";
                    goal_handle->publish_feedback(feedback);

                } else {

                    feedback_msg = "{STEP " + std::to_string(i) + "}: " + ACTION + ":Defined ACTION is not valid, ERROR.";
                    goal_handle->publish_feedback(feedback);
                    CONTINUE = false;

                };

                // c) EXECUTE and RETURN RESULT (step feedback):
                if (RES == "PLANNING: OK" || RES == "PLANNING: OK (EE)"){

                    feedback_msg = "{STEP " + std::to_string(i) + "}: " + ACTION + ":Planning OK.";
                    goal_handle->publish_feedback(feedback);

                    bool ExecSUCCESS = (move_group_interface_ROB.execute(MyPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

                    if (goal_handle->is_canceling()) {
                        feedback_msg = "{STEP " + std::to_string(i) + "}: " + ACTION + ":Canceled.";
                        goal_handle->publish_feedback(feedback);
                        goal_handle->canceled(result);
                        CONTINUE = false;
                        return;
                    } 
                    
                    if (ExecSUCCESS){
                        feedback_msg = "{STEP " + std::to_string(i) + "}: " + ACTION + ":Movement executed, SUCCESS.";
                        goal_handle->publish_feedback(feedback);
                    } else {
                        feedback_msg = "{STEP " + std::to_string(i) + "}: " + ACTION + ":Movement execution failed, ERROR.";
                        goal_handle->publish_feedback(feedback);
                        CONTINUE = false;
                    }
                    
                } else if (RES == "PLANNING: ERROR" || RES == "PLANNING: ERROR (EE)"){
                    feedback_msg = "{STEP " + std::to_string(i) + "}: " + ACTION + ":Planning ERROR.";
                    goal_handle->publish_feedback(feedback);
                    CONTINUE = false;

                } else if (RES == "LIMITS: ERROR" || RES == "LIMITS: ERROR (EE)"){
                    feedback_msg = "{STEP " + std::to_string(i) + "}: " + ACTION + ":Joint limits ERROR.";
                    goal_handle->publish_feedback(feedback);
                    CONTINUE = false;
                
                };

                // RE-INITIALISE RES variable:
                RES = "none";

                // z) Increment (i);
                i = i + 1;

            }

        }

        // RETURN -> RESULT:
        result->result = "EXECUTION FINISHED.";
        goal_handle->succeed(result);
        
    }

};


// ==================== MAIN ==================== //

int main(int argc, char ** argv)
{
    // Initialise MAIN NODE:
    rclcpp::init(argc, argv);
    auto const logger = rclcpp::get_logger("ros2srrc_SEQUENCE");

    // Obtain ROBOT + END-EFFECTOR parameters:
    auto node_PARAM_ROB = std::make_shared<ros2_RobotParam>();
    rclcpp::spin_some(node_PARAM_ROB);
    auto node_PARAM_EE = std::make_shared<ros2_EEParam>();
    rclcpp::spin_some(node_PARAM_EE);
    auto node_PARAM_ENV = std::make_shared<ros2_EnvironmentParam>();
    rclcpp::spin_some(node_PARAM_ENV);

    // DEFINE -> RobotSPECS + eeSPECS variables:
    // Robot SPECIFICATIONS:
    std::string pkgPATH_R = ament_index_cpp::get_package_share_directory("ros2srrc_robots");
    std::string PATH_R = pkgPATH_R + "/" + param_ROB + "/config/joint_specifications.yaml";
    YAML::Node SPECIFICATIONS_R = YAML::LoadFile(PATH_R);
    robotSPECS.robot_max = SPECIFICATIONS_R["Limits"]["Max"].as<std::vector<double>>();
    robotSPECS.robot_min = SPECIFICATIONS_R["Limits"]["Min"].as<std::vector<double>>();
    // End-Effector SPECIFICATIONS:
    std::string pkgPATH_EE = ament_index_cpp::get_package_share_directory("ros2srrc_endeffectors");
    std::string PATH_EE = pkgPATH_EE + "/" + param_EE + "/config/joint_specifications.yaml";
    YAML::Node SPECIFICATIONS_EE = YAML::LoadFile(PATH_EE);
    eeSPECS.ee_max = SPECIFICATIONS_EE["Limits"]["Max"].as<double>();
    eeSPECS.ee_min = SPECIFICATIONS_EE["Limits"]["Min"].as<double>();
    eeSPECS.ee_vector =  SPECIFICATIONS_EE["JointsVector"].as<std::vector<double>>();

    // Declare ATTACH and DETACH nodes:
    if (param_EE != "none" && param_ENV == "gazebo"){
        AttachDetach_NODE();
        RCLCPP_INFO(logger, "Attacher/Detacher NODES initialised.");
    }

    // Declare ABB/EGP64 Gripper node:
    if (param_ROB == "irb120" && param_EE == "egp64" && param_ENV == "bringup"){
        ABBGripper_NODE();
        RCLCPP_INFO(logger, "ABB Gripper NODE initialised.");
    }

    // Declare UR Robotiq-HandE/2f85 Gripper node:
    if (param_ROB == "ur3" && param_EE == "robotiq_hande" && param_ENV == "bringup"){
        URRobotiqGripper_NODE();
        RCLCPP_INFO(logger, "UR-RobotiqHandE NODE initialised.");
    } else if (param_ROB == "ur3" && param_EE == "robotiq_2f85" && param_ENV == "bringup"){
        URRobotiqGripper_NODE();
        RCLCPP_INFO(logger, "UR-Robotiq2f85 NODE initialised.");
    }
         
    // Launch and spin (EXECUTOR) MoveIt!2 Interface node:
    auto name = "ros2srrc_sequence";
    auto const node2 = std::make_shared<rclcpp::Node>(name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    rclcpp::executors::SingleThreadedExecutor executor; 
    executor.add_node(node2);
    std::thread([&executor]() { executor.spin(); }).detach();

    // CREATE -> MoveGroupInterface(s):
    using moveit::planning_interface::MoveGroupInterface;
    // 1. ROBOT:
    if (param_ROB != "none"){
        auto name = param_ROB + "_arm";
        
        move_group_interface_ROB = MoveGroupInterface(node2, name);
        move_group_interface_ROB.setPlanningPipelineId("move_group");

        move_group_interface_ROB.setMaxVelocityScalingFactor(1.0);
    
        // ACCELERATION SCALING FACTOR:
        // This value needs to be tuned for the robots, since joint speed/acceleration limits are exceeded otherwise.
        // IRB120:
        if (param_ROB == "irb120" && param_ENV == "bringup"){
            move_group_interface_ROB.setMaxAccelerationScalingFactor(0.5); // AFTER TUNING VALUES in ABB RobotStudio, 0.5 is a reasonable value for the IRB120.
        } else if (param_ROB == "irb120" && param_ENV == "gazebo") {
            move_group_interface_ROB.setMaxAccelerationScalingFactor(0.5); // Equaled in order to have same speed in IRB120 Real Robot = Gazebo.
        } 
        // UR3 + UR10e:
        else if (param_ROB == "ur3" || param_ROB == "ur10e") {
            move_group_interface_ROB.setMaxAccelerationScalingFactor(1.0);
        }

        joint_model_group_ROB = move_group_interface_ROB.getCurrentState()->getJointModelGroup(name);
        RCLCPP_INFO(logger, "MoveGroupInterface object created for ROBOT: %s", param_ROB.c_str());
    }
    // 2. END-EFFECTOR:
    if (param_EE != "none" && param_ENV != "bringup"){
        move_group_interface_EE = MoveGroupInterface(node2, param_EE);
        move_group_interface_EE.setPlanningPipelineId("move_group");
        move_group_interface_EE.setMaxVelocityScalingFactor(1.0);
        move_group_interface_EE.setMaxAccelerationScalingFactor(1.0);
        joint_model_group_EE = move_group_interface_EE.getCurrentState()->getJointModelGroup(param_EE);
        RCLCPP_INFO(logger, "MoveGroupInterface object created for END-EFFECTOR: %s", param_EE.c_str());
    }

    // CREATE -> PlanningSceneInterface:
    using moveit::planning_interface::PlanningSceneInterface;
    auto planning_scene_interface = PlanningSceneInterface();

    // Declare and spin ACTION SERVER:
    auto action_server = std::make_shared<ActionServer>();
    rclcpp::spin(action_server);

    rclcpp::shutdown();
    return 0;
}