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

# moveit2.launch.py:
# Launch file for the Robot's GAZEBO SIMULATION + MoveIt!2 Framework in ROS2 Humble:

# Import libraries:
import os, sys, xacro, yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

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

    YAML_PATH = os.path.join(
        get_package_share_directory("ros2srrc_endeffectors"),
        EEName,
        "config",
        "controller.yaml"
    )
    
    with open(YAML_PATH, 'r') as YAML:
        cYAML = yaml.safe_load(YAML)

    ros_parameters = cYAML["controller_manager"]["ros__parameters"]
    for controller_name, controller_config in ros_parameters.items():
        if isinstance(controller_config, dict):
            controller_type = controller_config.get("type", "")
            if "GripperActionController" in controller_type:
                RESULT.append(controller_name)

    return(RESULT)

# CHECK if CONTROLLER file exists for EE:
def EEctrlEXISTS(EEName):
    
    PATH = os.path.join(os.path.expanduser('~'), 'dev_ws', 'src', 'ros2_SimRealRobotControl', 'ros2srrc_endeffectors', EEName, 'config')
    YAML_PATH = PATH + "/controller.yaml"
    
    RES = os.path.exists(YAML_PATH)
    return(RES)

# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():

    LD = LaunchDescription()
    
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
        PKG_PATH = get_package_share_directory(PACKAGE_NAME)
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

    # ========== CELL INFORMATION ========== #
    print("")
    print("===== GAZEBO: Robot Simulation + MoveIt!2 Framework (" + PACKAGE_NAME + ") =====")
    print("Robot configuration:")
    print(CONFIGURATION["ID"] + " -> " + CONFIGURATION["Name"])
    print("")
    
    # ***** GAZEBO ***** #   
    # DECLARE GAZEBO WORLD file:
    world_gz = os.path.join(
        get_package_share_directory('ros2srrc_gz'),
        'worlds',
        'ros2srrc_gz.sdf')
    # DECLARE Gazebo LAUNCH file:
    gzSIM = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]
        ),
        launch_arguments={
            'gz_args': f'-r -v 1 "{world_gz}"',
            'on_exit_shutdown': 'true'
        }.items(),
    )

    # ROS 2 Gz Clock:
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # ***** ROBOT DESCRIPTION ***** #
    # Robot Description file package:
    robot_description_path = os.path.join(get_package_share_directory(PACKAGE_NAME))
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
        "prefix": "",
    })
    
    # EE -> Controller file needed?
    if EE == "true":
        if EEctrlEXISTS(CONFIGURATION["ee"]) == False:
            EE = "true-NOctr"
    
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

    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', CONFIGURATION["rob"],
            '-x', '0',
            '-y', '0',
            '-z', '0',
        ],
        output='both')

    # ***** CONTROLLERS ***** #
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

    # EE CONTROLLERS:
    if EE == "true":
        CONTROLLERS = GetEEctr(CONFIGURATION["ee"])
        CONTROLLER_NODES = []

        for x in CONTROLLERS:
            CONTROLLER_NODES.append(
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[x, "-c", "/controller_manager"],
                )
            )

    # SpawnEntity service bridge for world "ros2srrc_GzWorld":
    gzSERVICE_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_spawn_service_bridge',
        arguments=['/world/ros2srrc_GzWorld/create@ros_gz_interfaces/srv/SpawnEntity'],
        output='screen'
    )

    # Gazebo TOPIC BRIDGE for the camera:
    gzTOPIC_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='camera_image_bridge',
        arguments=['/camera/image_raw'],  # CAMERA TOPIC.
        output='screen'
    )

    # *********************** MoveIt!2 *********************** #   

    # *** PLANNING CONTEXT *** #
    # Robot description, SRDF:
    if (EE == "false"):
        srdf_file = os.path.join(get_package_share_directory("ros2srrc_moveit"), "config", CONFIGURATION["rob"] + ".srdf")
    else:
        srdf_file = os.path.join(get_package_share_directory("ros2srrc_moveit"), "config", CONFIGURATION["rob"] + "_" + CONFIGURATION["ee"] + ".srdf")

    srdf_doc = xacro.parse(open(srdf_file))
    xacro.process_doc(srdf_doc, mappings={"prefix": "", "name": CONFIGURATION["rob"]})
    srdf_doc.documentElement.setAttribute("name", CONFIGURATION["rob"])
    robot_description_semantic_config = srdf_doc.toxml()
    
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

    # Kinematics.yaml file:
    kinematics_yaml = load_yaml("ros2srrc_robots", CONFIGURATION["rob"] + "/config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # joint_limits.yaml file & pilz_cartesian_limits.yaml file:
    joint_limits_yaml = load_yaml("ros2srrc_robots", CONFIGURATION["rob"] + "/config/joint_limits.yaml")
    
    pilz_cartesian_limits_yaml = load_yaml("ros2srrc_robots", CONFIGURATION["rob"] + "/config/pilz_cartesian_limits.yaml")

    robot_description_planning = {
        'robot_description_planning': {
            **(joint_limits_yaml or {}),          
            **(pilz_cartesian_limits_yaml or {}), 
        }
    }

    # pilz_planning_pipeline_config.yaml file:
    planning_pipelines = {
        "planning_pipelines": ["pilz_industrial_motion_planner"],  # default-only list is fine
        "planning_pipeline": "pilz_industrial_motion_planner",     # legacy top-level; harmless
    }

    # Namespace block for the pilz pipeline (Jazzy expects `planning_plugins`, plural):
    pilz_planning_pipeline_config = {
        "pilz_industrial_motion_planner": {
            "planning_plugins": ["pilz_industrial_motion_planner/CommandPlanner"],
            "default_planner_config": "PTP",
            "start_state_max_bounds_error": 0.1,
            # "request_adapters": [],  # optional
        }
    }

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
        "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction "
                        "pilz_industrial_motion_planner/MoveGroupSequenceService"
    }   

    # MoveGroup Node:
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            
            planning_pipelines,
            pilz_planning_pipeline_config, 

            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_capabilities,
            {"use_sim_time": True},
        ],
    )

    # RVIZ:
    rviz_full_config = os.path.join(
        get_package_share_directory("ros2srrc_moveit"),
        "config",
        "ros2srrc.rviz",
    )

    rviz_node_full = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            
            planning_pipelines,
            pilz_planning_pipeline_config, 

            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_capabilities,
            {"use_sim_time": True},

            {"move_group/planning_plugin": "pilz_industrial_motion_planner"},
            {"move_group/default_planner_config": "PTP"},
        ]
    )

    # =================================================================================================== #
    # ============================= ros2srrc_execution -> CUSTOM INTERFACES ============================= #

    # Move and Sequence:
    if EE == "true":

        MoveInterface = Node(
            name="move",
            package="ros2srrc_execution",
            executable="move",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                
                {"use_sim_time": True},
                {"ROB_PARAM": CONFIGURATION["rob"]},
                {"EE_PARAM": CONFIGURATION["ee"]},
                {"ENV_PARAM": "gazebo"},
                {"move_group_ns": ""},
                {"EE_CONTROLLER_NAMES": CONTROLLERS},
                {"EE_CONTROLLER_ACTION_NAMESPACES": ["gripper_cmd"] * len(CONTROLLERS)},
            ],
        )

    else:

        MoveInterface = Node(
            name="move",
            package="ros2srrc_execution",
            executable="move",
            output="screen",
            parameters=[
                robot_description, 
                robot_description_semantic, 
                robot_description_kinematics, 
                
                {"use_sim_time": True}, 
                {"ROB_PARAM": CONFIGURATION["rob"]}, 
                {"EE_PARAM": "none"}, 
                {"ENV_PARAM": "gazebo"},
                {"move_group_ns": ""}
            ],
        )

    # RobMove and RobPose:
    RobMoveInterface = Node(
        name="robmove",
        package="ros2srrc_execution",
        executable="robmove",
        output="screen",
        parameters=[
            robot_description, 
            robot_description_semantic, 
            robot_description_kinematics, 
            
            {"use_sim_time": True}, 
            {"ROB_PARAM": CONFIGURATION["rob"]},
            {"move_group_ns": ""}
        ],
    )
    
    RobPoseInterface = Node(
        name="robpose",
        package="ros2srrc_execution",
        executable="robpose",
        output="screen",
        parameters=[
            robot_description, 
            robot_description_semantic, 
            robot_description_kinematics, 
        
            {"use_sim_time": True}, 
            {"ROB_PARAM": CONFIGURATION["rob"]},
            {"move_group_ns": ""}
        ],
    )
    
    # =============================================== #
    # ========== RETURN LAUNCH DESCRIPTION ========== #

    # Add ROS 2 Nodes to LaunchDescription() element:
    LD.add_action(gzSIM)
    LD.add_action(gzSERVICE_bridge)
    LD.add_action(gzTOPIC_bridge)
    LD.add_action(clock_bridge)
    LD.add_action(node_robot_state_publisher)
    LD.add_action(static_tf)
    LD.add_action(spawn_entity)

    LD.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action = spawn_entity,
            on_exit = [
                joint_state_broadcaster_spawner,
                ]
            )
        )
    )

    LD.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action = spawn_entity,
            on_exit = [
                joint_trajectory_controller_spawner,
                ]
            )
        )
    )

    if EE == "true":

        for x in CONTROLLER_NODES:

            LD.add_action(RegisterEventHandler(
                OnProcessExit(
                    target_action = joint_trajectory_controller_spawner,
                    on_exit = [
                        x,
                        ]
                    )
                )
            )

    LD.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action = spawn_entity,
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
            target_action = spawn_entity,
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
