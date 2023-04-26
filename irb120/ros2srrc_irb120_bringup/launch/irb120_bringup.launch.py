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
#           Seemal Asif      - s.asif@cranfield.ac.uk                                   #
#           Phil Webb        - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: August, 2022.                                                                  #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA (2022) ROS 2 ROBOT CONTROL. URL: --

# irb120_bringup.launch.py:
# Launch file for the ABB-IRB120 Robot CONTROL BRINGUP in ROS2 Humble:

# Import libraries:
import os
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

# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():

    # ========== COMMAND LINE ARGUMENTS ========== #
    print("")
    print("===== ABB IRB-120: Robot Bringup (ros2srrc_irb120_bringup) =====")
    print("Robot configuration:")
    print("")
    # robot_ip:
    print("- IP Address:")
    robot_ip = input("  Please input the IP Address of the Robot: ")
    print("")
    # Cell Layout:
    print("- Cell layout:")
    error = True
    while (error == True):
        print("     + Option N1: ABB IRB-120 alone.")
        print("     + Option N2: Cranfield University: IA Lab enclosure.")
        cell_layout = input ("  Please select: ")
        if (cell_layout == "1"):
            error = False
            cell_layout_1 = "true"
            cell_layout_2 = "false"
        elif (cell_layout == "2"):
            error = False
            cell_layout_1 = "false"
            cell_layout_2 = "true"
        else:
            print ("  Please select a valid option!")
    print("")
    # End-Effector:
    print("- End-effector:")
    error = True
    while (error == True):
        print("     + Option N1: No end-effector.")
        print("     + Option N2: Schunk EGP-64 parallel gripper.")
        end_effector = input ("  Please select: ")
        if (end_effector == "1"):
            error = False
            EE_no = "true"
            EE_schunk = "false"
        elif (end_effector == "2"):
            error = False
            EE_no = "false"
            EE_schunk = "true"
        else:
            print ("  Please select a valid option!")
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
        "robot_ip": robot_ip, 
        "bringup": "true",

        "cell_layout_1": cell_layout_1,
        "cell_layout_2": cell_layout_2,
        "EE_no": EE_no,
        "EE_schunk": EE_schunk,
        })
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    # ROBOT STATE PUBLISHER NODE:
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )
    # Static TF:
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # ***** CONTROLLERS ***** #
    # ros2_control:
    ros2_controllers_path = os.path.join(
        get_package_share_directory("ros2srrc_irb120_bringup"),
        "config",
        "abb_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="both",
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


    # ***** MoveIt!2 ***** #

    # Command-line argument: RVIZ file?
    rviz_arg = DeclareLaunchArgument(
        "rviz_file", default_value="False", description="Load RVIZ file."
    )

    # *** PLANNING CONTEXT *** #
    # Robot description, SRDF:
    if (EE_no == "true"):
        robot_description_semantic_config = load_file(
            "ros2srrc_irb120_moveit2", "config/irb120.srdf"
        )
    # === SCHUNK EGP-64 === #
    elif (EE_schunk == "true"):
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
        "ros2srrc_irb120_moveit2", "config/joint_limits_bringup.yaml"
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

    # Move group: OMPL Planning.
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        "ros2srrc_irb120_moveit2", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # MoveIt!2 Controllers:
    moveit_simple_controllers_yaml = load_yaml(
        "ros2srrc_irb120_bringup", "config/moveit_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.1,
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
            #ompl_planning_pipeline_config,

            joint_limits,
            pilz_cartesian_limits,

            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_capabilities,

        ],
    )

    # RVIZ:
    load_RVIZfile = LaunchConfiguration("rviz_file")
    rviz_base = os.path.join(get_package_share_directory("ros2srrc_irb120_moveit2"), "config")
    rviz_full_config = os.path.join(rviz_base, "irb120_moveit2.rviz")
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
            #ompl_planning_pipeline_config,

            joint_limits,
            pilz_cartesian_limits,

            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_capabilities,
        ],
        condition=UnlessCondition(load_RVIZfile),
    )

    if (EE_no == "true"):
        
        MoveInterface = Node(
            name="move",
            package="ros2srrc_execution",
            executable="move",
            output="screen",
            parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ROB_PARAM": "irb120_arm"}, {"EE_PARAM": "none"}, {"ENV_PARAM": "bringup"}],
        )
        SequenceInterface = Node(
            name="sequence",
            package="ros2srrc_execution",
            executable="sequence",
            output="screen",
            parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ROB_PARAM": "irb120_arm"}, {"EE_PARAM": "none"}, {"ENV_PARAM": "bringup"}],
        )
    
    elif (EE_schunk == "true"):
    
        MoveInterface = Node(
            name="move",
            package="ros2srrc_execution",
            executable="move",
            output="screen",
            parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ROB_PARAM": "irb120_arm"}, {"EE_PARAM": "egp64"}, {"ENV_PARAM": "bringup"}],
        )
        SequenceInterface = Node(
            name="sequence",
            package="ros2srrc_execution",
            executable="sequence",
            output="screen",
            parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ROB_PARAM": "irb120_arm"}, {"EE_PARAM": "egp64"}, {"ENV_PARAM": "bringup"}],
        )

    # ========== ABB RWS CLIENT ========== #
    
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

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        
        # 1. Step: Connect to ROBOT:
        ros2_control_node,
        node_robot_state_publisher,
        static_tf,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        rws_client,
        
        # 2. Step: Launch MoveIt!2:
        RegisterEventHandler(
            OnProcessExit(
                target_action = joint_trajectory_controller_spawner,
                on_exit = [
                    rviz_arg,
                    run_move_group_node,

                    TimerAction(
                        period=5.0,
                        actions=[rviz_node_full],
                    ),

                    MoveInterface,
                    SequenceInterface,
                ]
            )
        ),
    ]
)