#include "ros2srrc_execution/movej.h"
#include "ros2srrc_execution/movel.h"
#include "ros2srrc_execution/mover.h"
#include "ros2srrc_execution/movexyzw.h"
#include "ros2srrc_execution/movexyz.h"
#include "ros2srrc_execution/moveypr.h"
#include "ros2srrc_execution/moverot.h"
#include "ros2srrc_execution/moverp.h"
#include "ros2srrc_execution/moveg.h"

// Include for ATTACHER/DETACHER:
#include "std_msgs/msg/string.hpp"
#include "ros2_grasping/action/attacher.hpp"

// Include standard libraries:
#include <string>
#include <vector>
#include <unistd.h> 
#include <ctime>

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

// Declaration of GLOBAL VARIABLES --> Attacher & Detacher nodes:
std::shared_ptr<rclcpp::Node> AttacherNode;
std::shared_ptr<rclcpp::Node> DetacherNode;

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

    AttacherNode = rclcpp::Node::make_shared("ATTACHER_AC_node");
    DetacherNode = rclcpp::Node::make_shared("DETACHER_P_node");

}

void ATTACH(std::string OBJECT){

    auto ATTACHER_AC = rclcpp_action::create_client<ros2_grasping::action::Attacher>(AttacherNode, "Attacher");
    
    auto goal_msg = ros2_grasping::action::Attacher::Goal();
    goal_msg.object = OBJECT;
    goal_msg.endeffector = "EE_" + param_EE;

    auto GOAL = ATTACHER_AC->async_send_goal(goal_msg);
    rclcpp::spin_some(AttacherNode);

}

void DETACH(){
    
    auto publisher = DetacherNode->create_publisher<std_msgs::msg::String>("ros2_Detach", 10);
    
    std_msgs::msg::String message;
    message.data = "True";
     
    int i = 0;
    while (i < 1000) {
        publisher->publish(message);
        rclcpp::spin_some(DetacherNode);
        i = i + 1;
    }
    
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
        std::string ROB = goal->robot + "_arm";
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
        if (param_EE != "none"){
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
                    MoveJSTRUCT MoveJRES = MoveJAction(STEP.movej, JP, param_ROB);
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
                    MoveRSTRUCT MoveRRES = MoveRAction(STEP.mover, JP, param_ROB);
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

                } else if (ACTION == "MoveXYZW"){
                    
                    // 1. CALL MoveXYZWAction for CALCULATIONS:
                    auto TARGET_POSE = MoveXYZWAction(STEP.movexyzw);
                    move_group_interface_ROB.setPoseTarget(TARGET_POSE);
                    
                    // 2. Assign SPEED and PLANNING METHOD (PTP, LIN, CIRC):
                    move_group_interface_ROB.setMaxVelocityScalingFactor(STEP.speed);
                    move_group_interface_ROB.setPlannerId("PTP");

                    // 3. PLAN:
                    MyPlan = plan_ROB();
                
                } else if (ACTION == "MoveXYZ"){
                    
                    // 1. Define POSE VECTOR:
                    auto POSE = move_group_interface_ROB.getCurrentPose();
                    
                    // 2. CALL MoveXYZAction for CALCULATIONS:
                    auto TARGET_POSE = MoveXYZAction(STEP.movexyz, POSE);
                    move_group_interface_ROB.setPoseTarget(TARGET_POSE);
                    
                    // 3. Assign SPEED and PLANNING METHOD (PTP, LIN, CIRC):
                    move_group_interface_ROB.setMaxVelocityScalingFactor(STEP.speed);
                    move_group_interface_ROB.setPlannerId("PTP");

                    // 4. PLAN:
                    MyPlan = plan_ROB();
                
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
                    MoveGSTRUCT MoveGRES = MoveGAction(STEP.moveg, JP, param_EE);
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
                
                }

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
                
                }

                // SPECIFIC CASE -> ATTACH/DETACH:
                // This happens when an object needs to be attached to an end-effector in Gazebo Simulation (using ros2_grasping):
                if (ACTION == "Attach"){

                    ATTACH(STEP.attach);
                    
                    feedback_msg = "{STEP " + std::to_string(i) + "}: " + ACTION + ":Object attached successfully.";
                    goal_handle->publish_feedback(feedback);

                } else if (ACTION == "Detach"){

                    DETACH();

                    feedback_msg = "{STEP " + std::to_string(i) + "}: " + ACTION + ":Object detached successfully.";
                    goal_handle->publish_feedback(feedback);
                    
                }

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
    auto const logger = rclcpp::get_logger("SEQUENCE_INTERFACE");

    // Obtain ROBOT + END-EFFECTOR parameters:
    auto node_PARAM_ROB = std::make_shared<ros2_RobotParam>();
    rclcpp::spin_some(node_PARAM_ROB);
    auto node_PARAM_EE = std::make_shared<ros2_EEParam>();
    rclcpp::spin_some(node_PARAM_EE);
    auto node_PARAM_ENV = std::make_shared<ros2_EnvironmentParam>();
    rclcpp::spin_some(node_PARAM_ENV);

    // Declare ATTACH and DETACH nodes:
    if (param_EE != "none" && param_ENV == "gazebo"){
        AttachDetach_NODE();
    }
    
    // Launch and spin (EXECUTOR) MoveIt!2 Interface node:
    auto name = "ros2srrc_sequence";
    auto const node2 = std::make_shared<rclcpp::Node>(
        name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    rclcpp::executors::SingleThreadedExecutor executor; 
    executor.add_node(node2);
    std::thread([&executor]() { executor.spin(); }).detach();

    // CREATE -> MoveGroupInterface(s):
    using moveit::planning_interface::MoveGroupInterface;
    // 1. ROBOT:
    if (param_ROB != "none"){
        move_group_interface_ROB = MoveGroupInterface(node2, param_ROB);
        move_group_interface_ROB.setPlanningPipelineId("move_group");
        move_group_interface_ROB.setMaxVelocityScalingFactor(1.0);
    
        // ACCELERATION SCALING FACTOR FOR ROBOT:
        // This value needs to be tuned for the real ABB Robot, since joint speed/acceleration limits are exceeded otherwise.
        if (param_ROB == "irb120" && param_ENV == "bringup"){
            move_group_interface_ROB.setMaxAccelerationScalingFactor(0.5); // AFTER TUNING VALUES in ABB RobotStudio, 0.5 is a reasonable value for the IRB120.
        } else {
            move_group_interface_ROB.setMaxAccelerationScalingFactor(0.5); // Equaled in order to have same speed in IRB120 Real Robot = Gazebo.
        }

        joint_model_group_ROB = move_group_interface_ROB.getCurrentState()->getJointModelGroup(param_ROB);
        RCLCPP_INFO(logger, "MoveGroupInterface object created for ROBOT: %s", param_ROB.c_str());
    }
    // 2. END-EFFECTOR:
    if (param_EE != "none"){
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