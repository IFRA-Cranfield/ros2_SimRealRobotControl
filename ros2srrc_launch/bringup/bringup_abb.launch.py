#!/usr/bin/python3

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
#  Date: June, 2024.                                                                    #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023) ROS 2 Sim-to-Real Robot Control. URL: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl.

# bringup.launch.py:
# Launch file for the Robot's BRINGUP ROS 2 DRIVER + MoveIt!2 Framework in ROS2 Humble:

# Import libraries:
import os, sys, xacro, yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit

# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None
# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

# ===== REQUIRED TO GET THE ROBOT CONFIGURATION === #

# EVALUATE INPUT ARGUMENTS:
def AssignArgument(ARGUMENT):
    ARGUMENTS = sys.argv
    for y in ARGUMENTS:
        if (ARGUMENT + ":=") in y:
            ARG = y.replace((ARGUMENT + ":="),"")
            return(ARG)

# GET CONFIGURATION from YAML:
def GetCONFIG(CONFIGURATION, PKG_PATH):
    
    RESULT = {"Success": False, "ID": "", "Name": "", "urdf": "", "ee": ""}
    
    YAML_PATH = PKG_PATH + "/config/configurations.yaml"
    
    if not os.path.exists(YAML_PATH):
        return (RESULT)
    
    with open(YAML_PATH, 'r') as YAML:
        cYAML = yaml.safe_load(YAML)

    for x in cYAML["Configurations"]:

        if x["ID"] == CONFIGURATION:
            RESULT["Success"] = True
            RESULT["ID"] = x["ID"]
            RESULT["Name"] = x["Name"]
            RESULT["urdf"] = x["urdf"]
            RESULT["rob"] = x["rob"]
            RESULT["ee"] = x["ee"]

    return(RESULT)

# GET EE-Controllers LIST:
def GetEEctr(EEName):
    
    RESULT = []

    PATH = os.path.join(os.path.expanduser('~'), 'dev_ws', 'src', 'ros2_SimRealRobotControl', 'ros2srrc_endeffectors', EEName, 'config')
    YAML_PATH = PATH + "/controller_moveit2.yaml"
    
    with open(YAML_PATH, 'r') as YAML:
        cYAML = yaml.safe_load(YAML)

    for x in cYAML["controller_names"]:
        RESULT.append(x)

    return(RESULT)

# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():

    LD = LaunchDescription()

    # === INPUT ARGUMENT: robot_ip === #
    robot_ip = AssignArgument("robot_ip")
    if robot_ip != None:
        None
    else:
        print("")
        print("ERROR: robot_ip INPUT ARGUMENT has not been defined. Please try again.")
        print("Closing... BYE!")
        exit()
    
    # === INPUT ARGUMENT: ROS 2 PACKAGE === #
    PACKAGE_NAME = AssignArgument("package")
    if PACKAGE_NAME != None:
        None
    else:
        print("")
        print("ERROR: package INPUT ARGUMENT has not been defined. Please try again.")
        print("Closing... BYE!")
        exit()
        
    # CHECK if -> PACKAGE EXISTS, and GET PATH:
    try:
        PKG_PATH = get_package_share_directory(PACKAGE_NAME + "_gazebo")
    except PackageNotFoundError:
        print("")
        print("ERROR: The defined ROS 2 Package was not found. Please try again.")
        print("Closing... BYE!")
        exit()
    except ValueError:
        print("")
        print("ERROR: The defined ROS 2 Package name is not valid. Please try again.")
        print("Closing... BYE!")
        exit()
    
    # === INPUT ARGUMENT: CONFIGURATION === #
    CONFIG = AssignArgument("config")
    CONFIGURATION = GetCONFIG(CONFIG, PKG_PATH)

    if CONFIGURATION["Success"] == False:
        print("")
        print("ERROR: config INPUT ARGUMENT has not been correctly defined. Please try again.")
        print("Closing... BYE!")
        exit()   

    if CONFIGURATION["ee"] == "none":
        EE = "false"
    else: 
        EE = "true"

    # ========== CELL INFORMATION ========== #
    print("")
    print("===== " + CONFIGURATION["rob"] + ": Robot Bringup + MoveIt!2 Framework (" + PACKAGE_NAME + "_bringup) =====")
    print("Robot IP Address -> " + robot_ip)
    print("Robot configuration:")
    print(CONFIGURATION["ID"] + " -> " + CONFIGURATION["Name"])
    print("")

    # ***** ROBOT DESCRIPTION ***** #
    # Robot Description file package:
    robot_description_path = os.path.join(get_package_share_directory(PACKAGE_NAME + '_gazebo'))
    # ROBOT urdf file path:
    xacro_file = os.path.join(robot_description_path,'urdf',CONFIGURATION["urdf"])
    # Generate ROBOT_DESCRIPTION variable:
    doc = xacro.parse(open(xacro_file))
    
    if CONFIGURATION["ee"] == "none":
        EE = "false"
    else: 
        EE = "true"
    
    xacro.process_doc(doc, mappings={
        "EE": EE,
        "EE_name": CONFIGURATION["ee"],

        "robot_ip": robot_ip,
        "bringup": "true"
    })
    
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    # ROBOT STATE PUBLISHER NODE:
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {"use_sim_time": True}
        ]
    )
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # ***** CONTROLLERS ***** #

    # ros2_control:
    ros2_controllers_path = os.path.join(get_package_share_directory("ros2srrc_robots"), CONFIGURATION["rob"], "config", "controller.yaml")
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="both"
    )

    # Joint STATE BROADCASTER:
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    # Joint TRAJECTORY Controller:
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    # *********************** MoveIt!2 *********************** #   

    # *** PLANNING CONTEXT *** #
    # Robot description, SRDF:
    if EE == "false":
        robot_description_semantic_config = load_file(PACKAGE_NAME + "_moveit2", "config/" + CONFIGURATION["rob"] + ".srdf")
    else:
        robot_description_semantic_config = load_file(PACKAGE_NAME + "_moveit2", "config/" + CONFIGURATION["rob"] + CONFIGURATION["ee"] + ".srdf")
    
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

    # Kinematics.yaml file:
    kinematics_yaml = load_yaml("ros2srrc_robots", CONFIGURATION["rob"] + "/config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # joint_limits.yaml file:
    joint_limits_yaml = load_yaml("ros2srrc_robots", CONFIGURATION["rob"] + "/config/joint_limits.yaml")
    joint_limits = {'robot_description_planning': joint_limits_yaml}

    # pilz_planning_pipeline_config.yaml file:
    pilz_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
            "request_adapters": """ """,
            "start_state_max_bounds_error": 0.1,
            "default_planner_config": "PTP",
        }
    }
    pilz_cartesian_limits_yaml = load_yaml("ros2srrc_robots", CONFIGURATION["rob"] + "/config/pilz_cartesian_limits.yaml")
    pilz_cartesian_limits = {'robot_description_planning': pilz_cartesian_limits_yaml}

    # MoveIt!2 Controllers:
    moveit_simple_controllers_yaml = load_yaml("ros2srrc_robots", CONFIGURATION["rob"] + "/config/controller_moveit2.yaml")

    # MoveIt!2 Parameters:
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    move_group_capabilities = {
        "capabilities": """pilz_industrial_motion_planner/MoveGroupSequenceAction \
            pilz_industrial_motion_planner/MoveGroupSequenceService"""
    }

    # MoveGroup Node:
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            
            pilz_planning_pipeline_config,

            joint_limits,
            pilz_cartesian_limits,

            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_capabilities,
        ],
    )

    # RVIZ:
    rviz_base = os.path.join(get_package_share_directory(PACKAGE_NAME + "_moveit2"), "config")
    if EE == "false":
        rviz_full_config = os.path.join(rviz_base, CONFIGURATION["rob"] + "_moveit2.rviz")
    else:
        rviz_full_config = os.path.join(rviz_base, CONFIGURATION["rob"] + CONFIGURATION["ee"] + "_moveit2.rviz")

    rviz_node_full = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            
            pilz_planning_pipeline_config,

            joint_limits,
            pilz_cartesian_limits,

            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_capabilities,
        ]
    )

    # ============================================= #
    # ============== ABB: RWS CLIENT ============== #
    rws_client = Node(
        package="abb_rws_client",
        executable="rws_client",
        name="rws_client",
        output="screen",
        parameters=[
            {"robot_ip": robot_ip},
            {"robot_port": 80},
            {"robot_nickname": "ROB_1"},
            {"polling_rate": 4.0},
            {"no_connection_timeout": False},
        ],
    )

    # =================================================================================================== #
    # ============================= ros2srrc_execution -> CUSTOM INTERFACES ============================= #

    # Move:
    MoveInterface = Node(
        name="move",
        package="ros2srrc_execution",
        executable="move",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"ROB_PARAM": CONFIGURATION["rob"]}, {"EE_PARAM": "none"}, {"ENV_PARAM": "bringup"}],
    )
    # RobMove and RobPose:
    RobMoveInterface = Node(
        name="robmove",
        package="ros2srrc_execution",
        executable="robmove",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"ROB_PARAM": CONFIGURATION["rob"]}],
    )
    RobPoseInterface = Node(
        name="robpose",
        package="ros2srrc_execution",
        executable="robpose",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"ROB_PARAM": CONFIGURATION["rob"]}],
    )

    # =============================================== #
    # ========== RETURN LAUNCH DESCRIPTION ========== #

    # Add ROS 2 Nodes to LaunchDescription() element:
    LD.add_action(node_robot_state_publisher)
    LD.add_action(static_tf)
    LD.add_action(rws_client)
    
    LD.add_action(ros2_control_node)
    LD.add_action(joint_state_broadcaster_spawner)
    LD.add_action(joint_trajectory_controller_spawner)

    LD.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action = joint_trajectory_controller_spawner,
            on_exit = [
                
                # MoveIt!2:
                TimerAction(
                    period=2.0,
                    actions=[
                        rviz_node_full,
                        run_move_group_node,
                    ]
                ),
                
                ]
            )
        )
    )

    LD.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action = joint_trajectory_controller_spawner,
            on_exit = [
                
                # Interfaces:
                TimerAction(
                    period=5.0,
                    actions=[
                        MoveInterface,
                        RobMoveInterface,
                        RobPoseInterface,
                    ]
                ),
                
                ]
            )
        )
    )

    # ***** RETURN  ***** #
    return(LD)