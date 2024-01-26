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
#  Date: April, 2023.                                                                   #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023) ROS 2 Sim-to-Real Robot Control. URL: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl.

# irb120_interface.launch.py:
# Launch file for the ABB-IRB120 Robot GAZEBO + MoveIt!2 SIMULATION (+ Robot/Gripper triggers) in ROS2 Humble:

# Import libraries:
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml

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
    
# ========== **INPUT ARGUMENTS** ========== #
#  layout -> Cell layout.

# EVALUATE INPUT ARGUMENTS:
def AssignArgument(ARGUMENT):
    
    ARGUMENTS = sys.argv
    for y in ARGUMENTS:
        if (ARGUMENT + ":=") in y:
            ARG = y.replace((ARGUMENT + ":="),"")
            return(ARG)

# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():
    
    # ***** GAZEBO ***** #   
    # DECLARE Gazebo WORLD file:
    ros2srrc_irb120_gazebo = os.path.join(
        get_package_share_directory('ros2srrc_irb120_gazebo'),
        'worlds',
        'irb120.world')
    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={'world': ros2srrc_irb120_gazebo}.items(),
             )
    
    # ========== INPUT ARGUMENTS ========== #
    # Cell layout:
    layout = AssignArgument("layout")
    if layout != None:
        None
    else:
        print("")
        print("ERROR: layout INPUT ARGUMENT has not been defined. Please try again.")
        print("Closing... BYE!")
        exit()

    if layout == "ros2srrc_irb120_1":
        LYT = "ABB IRB-120 alone."
        ros2srrc_irb120_1 = "true"
        ros2srrc_irb120_2 = "false"
        ros2srrc_irb120_3 = "false"
    elif layout == "ros2srrc_irb120_2":
        LYT = "Cranfield University IA Lab enclosure."
        ros2srrc_irb120_1 = "false"
        ros2srrc_irb120_2 = "true"
        ros2srrc_irb120_3 = "false"
    elif layout == "ros2srrc_irb120_3":
        LYT = "Pick and Place Use-Case."
        ros2srrc_irb120_1 = "false"
        ros2srrc_irb120_2 = "false"
        ros2srrc_irb120_3 = "true"
    else:
        print("")
        print("ERROR: layout INPUT ARGUMENT has not been defined properly. Please try again.")
        print("Options: {ros2srrc_irb120_1, ros2srrc_irb120_2, ros2srrc_irb120_3}")
        print("Closing... BYE!")
        exit()

    # End effector:
    endeffector = AssignArgument("endeffector")
    if endeffector != None:
        None
    else:
        print("")
        print("ERROR: endeffector INPUT ARGUMENT has not been defined. Please try again.")
        print("Closing... BYE!")
        exit()

    if endeffector == "RobAlone":
        EE = "ABB IRB-120 alone."
        RobAlone = "true"
        egp64 = "false"
    elif endeffector == "egp64":
        EE = "Cranfield University IA Lab enclosure."
        RobAlone = "false"
        egp64 = "true"
    else:
        print("")
        print("ERROR: endeffector INPUT ARGUMENT has not been defined properly. Please try again.")
        print("Options: {RobAlone, egp64}")
        print("Closing... BYE!")
        exit()

    # ========== CELL INFORMATION ========== #
    print("")
    print("===== ABB IRB-120: Robot Simulation (ros2srrc_irb120_moveit2) =====")
    print("Robot configuration:")
    print("")
    # Cell Layout:
    print("- Cell layout: " + LYT)
    # End-Effector:
    print("- End-effector: " + EE)
    print("")

    # ***** ROBOT DESCRIPTION ***** #
    # ABB-IRB120 Description file package:
    irb120_description_path = os.path.join(
        get_package_share_directory('ros2srrc_irb120_gazebo'))
    # ABB-IRB120 ROBOT urdf file path:
    xacro_file = os.path.join(irb120_description_path,
                              'urdf',
                              'irb120.urdf.xacro')
    # Generate ROBOT_DESCRIPTION for ABB-IRB120:
    doc = xacro.parse(open(xacro_file))
    
    xacro.process_doc(doc, mappings={
        "cell_layout_1": ros2srrc_irb120_1,
        "cell_layout_2": ros2srrc_irb120_2,
        "cell_layout_3": ros2srrc_irb120_3,
        "EE_no": RobAlone,
        "EE_schunk": egp64,
    })

    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'irb120'],
                        output='screen')

    # ***** STATIC TRANSFORM ***** #
    # NODE -> Static TF:
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )
    # Publish TF:
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {"use_sim_time": True}
        ]
    )

    # ***** ROS2_CONTROL -> LOAD CONTROLLERS ***** #

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
        arguments=["irb120_controller", "-c", "/controller_manager"],
    )
    
    # === SCHUNK EGP-64 === #
    egp64left_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["egp64_finger_left_controller", "-c", "/controller_manager"],
    )
    egp64right_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["egp64_finger_right_controller", "-c", "/controller_manager"],
    )
    # === SCHUNK EGP-64 === #


    # *********************** MoveIt!2 *********************** #   
    
    # Command-line argument: RVIZ file?
    rviz_arg = DeclareLaunchArgument(
        "rviz_file", default_value="False", description="Load RVIZ file."
    )

    # *** PLANNING CONTEXT *** #
    # Robot description, SRDF:
    if (endeffector == "RobAlone"):
        robot_description_semantic_config = load_file(
            "ros2srrc_irb120_moveit2", "config/irb120.srdf"
        )
    # === SCHUNK EGP-64 === #
    elif (endeffector == "egp64"):
        robot_description_semantic_config = load_file(
            "ros2srrc_irb120_moveit2", "config/irb120egp64.srdf"
        )
    # === SCHUNK EGP-64 === #

    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # Kinematics.yaml file:
    kinematics_yaml = load_yaml(
        "ros2srrc_irb120_moveit2", "config/kinematics.yaml"
    )
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # joint_limits.yaml file:
    joint_limits_yaml = load_yaml(
        "ros2srrc_irb120_moveit2", "config/joint_limits.yaml"
    )
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
    pilz_cartesian_limits_yaml = load_yaml(
        "ros2srrc_irb120_moveit2", "config/pilz_cartesian_limits.yaml"
    )
    pilz_cartesian_limits = {'robot_description_planning': pilz_cartesian_limits_yaml}

    # MoveIt!2 Controllers:
    if (endeffector == "RobAlone"):
        moveit_simple_controllers_yaml = load_yaml(
            "ros2srrc_irb120_moveit2", "config/irb120_controllers.yaml"  
        )
    # === SCHUNK EGP-64 === #
    elif (endeffector == "egp64"):
        moveit_simple_controllers_yaml = load_yaml(
            "ros2srrc_irb120_moveit2", "config/irb120egp64_controllers.yaml"  
        )
    # === SCHUNK EGP-64 === #

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

    # START NODE -> MOVE GROUP:
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
            {"use_sim_time": True},
        ],
    )

    # RVIZ:
    load_RVIZfile = LaunchConfiguration("rviz_file")
    rviz_base = os.path.join(get_package_share_directory("ros2srrc_irb120_moveit2"), "config")

    if (endeffector == "RobAlone"):
        rviz_full_config = os.path.join(rviz_base, "irb120_moveit2.rviz")
    # === SCHUNK EGP-64 === #
    elif (endeffector == "egp64"):
        rviz_full_config = os.path.join(rviz_base, "irb120egp64_moveit2.rviz")
    # === SCHUNK EGP-64 === #

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
            {"use_sim_time": True},
        ],
        condition=UnlessCondition(load_RVIZfile),
    )
    
    if (endeffector == "RobAlone"):
        
        MoveInterface = Node(
            name="move",
            package="ros2srrc_execution",
            executable="move",
            output="screen",
            parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ROB_PARAM": "irb120"}, {"EE_PARAM": "none"}, {"ENV_PARAM": "gazebo"}],
        )
        SequenceInterface = Node(
            name="sequence",
            package="ros2srrc_execution",
            executable="sequence",
            output="screen",
            parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ROB_PARAM": "irb120"}, {"EE_PARAM": "none"}, {"ENV_PARAM": "gazebo"}],
        )
    
    elif (endeffector == "egp64"):
    
        MoveInterface = Node(
            name="move",
            package="ros2srrc_execution",
            executable="move",
            output="screen",
            parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ROB_PARAM": "irb120"}, {"EE_PARAM": "egp64"}, {"ENV_PARAM": "gazebo"}],
        )
        SequenceInterface = Node(
            name="sequence",
            package="ros2srrc_execution",
            executable="sequence",
            output="screen",
            parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ROB_PARAM": "irb120"}, {"EE_PARAM": "egp64"}, {"ENV_PARAM": "gazebo"}],
        )
    
    RobMoveInterface = Node(
        name="robmove",
        package="ros2srrc_execution",
        executable="robmove",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ROB_PARAM": "irb120"}],
    )
    RobPoseInterface = Node(
        name="robpose",
        package="ros2srrc_execution",
        executable="robpose",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ROB_PARAM": "irb120"}],
    )
    
    return LaunchDescription(
        [
            # Gazebo nodes:
            gazebo, 
            spawn_entity,
            # ROS2_CONTROL:
            static_tf,
            robot_state_publisher,
            
            # ROS2 Controllers:
            RegisterEventHandler(
                OnProcessExit(
                    target_action = spawn_entity,
                    on_exit = [
                        joint_state_broadcaster_spawner,
                    ]
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action = joint_state_broadcaster_spawner,
                    on_exit = [
                        joint_trajectory_controller_spawner,
                    ]
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action = joint_trajectory_controller_spawner,
                    on_exit = [
                        egp64left_controller_spawner,
                    ]
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action = egp64left_controller_spawner,
                    on_exit = [
                        egp64right_controller_spawner,
                    ]
                )
            ),

            RegisterEventHandler(
                OnProcessExit(
                    target_action = egp64right_controller_spawner,
                    on_exit = [

                        # MoveIt!2:
                        TimerAction(
                            period=2.0,
                            actions=[
                                rviz_arg,
                                rviz_node_full,
                                run_move_group_node
                            ]
                        ),

                        # TEST:
                        TimerAction(
                            period=5.0,
                            actions=[
                                MoveInterface,
                                SequenceInterface,
                                RobMoveInterface,
                                RobPoseInterface,
                            ]
                        ),

                    ]
                )
            )
        ]
    )