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
#  Date: March, 2026.                                                                   #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023) ROS 2 Sim-to-Real Robot Control. URL: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl.

# moveit2.launch.py:
# Launch file for the (2) ROBOT's GZ SIM / Gazebo Fortress simulation + MoveIt!2 Framework in ROS 2 Humble:

# Import libraries:
import os, sys, xacro, yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

TMPFILE_PATH = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "python"))
if TMPFILE_PATH not in sys.path:
    sys.path.append(TMPFILE_PATH)

from tmpCTRLfile import CreateTMPControllerFile
from tmpRVizfile import CreateTMPRVizFile

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
    
# PREFIX - Kinematics.yaml:
def PrefixKinematicsKeys(kinematics_yaml, prefix):
    # Top-level keys are planning group names (e.g. ur3_arm)
    RESULT = {}
    for GROUP, VALUE in kinematics_yaml.items():
        RESULT[prefix + GROUP] = VALUE
    return RESULT
# PREFIX - Joint_limits.yaml:
def PrefixJointLimits(joint_limits_yaml, prefix):
    # Input format: {"joint_limits": {...}}
    RESULT = {"joint_limits": {}}
    for JOINT, VALUE in joint_limits_yaml["joint_limits"].items():
        RESULT["joint_limits"][prefix + JOINT] = VALUE
    return RESULT
# PREFIX - MoveIt!2 controllers .yaml:
def PrefixMoveItControllers(moveit_yaml, prefix, use_absolute_controller_names=True):
    OUT = {"controller_names": []}

    for CTRL in moveit_yaml["controller_names"]:
        CTRL_NEW = prefix + CTRL
        CTRL_KEY = "/" + CTRL_NEW if use_absolute_controller_names else CTRL_NEW

        OUT["controller_names"].append(CTRL_KEY)

        CFG = dict(moveit_yaml[CTRL])
        if "joints" in CFG:
            CFG["joints"] = [prefix + J for J in CFG["joints"]]

        OUT[CTRL_KEY] = CFG

    return OUT

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
            RESULT["name"] = x["name"]
            RESULT["urdf"] = x["urdf"]

            RESULT["ROB1_id"] = x["ROB_1"]["id"]
            RESULT["ROB1_rob"] = x["ROB_1"]["rob"]
            RESULT["ROB1_ee"] = x["ROB_1"]["ee"]

            RESULT["ROB2_id"] = x["ROB_2"]["id"]
            RESULT["ROB2_rob"] = x["ROB_2"]["rob"]
            RESULT["ROB2_ee"] = x["ROB_2"]["ee"]

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
    print("===== GZ SIM + MoveIt!2: Robot Simulation (" + PACKAGE_NAME + ") =====")
    print("Robot configuration:")
    print(CONFIGURATION["ID"] + " -> " + CONFIGURATION["name"])
    print("")
    
    # ***** GZ SIM ***** #
    # DECLARE GZ SIM WORLD file:
    world_gz = os.path.join(
        get_package_share_directory('ros2srrc_gz'),
        'worlds',
        'ros2srrc_gz.sdf')
    # DECLARE GZ SIM LAUNCH file:
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
        name="gz_clock_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],
    )

    # ***** ROBOT DESCRIPTION ***** #
    # Robot Description file package:
    robot_description_path = os.path.join(get_package_share_directory(PACKAGE_NAME))
    # ROBOT urdf file path:
    xacro_file = os.path.join(robot_description_path,'urdf',CONFIGURATION["urdf"])
    # Generate ROBOT_DESCRIPTION variable:
    doc = xacro.parse(open(xacro_file))
    
    # END-EFFECTOR(s) -> Check if defined, and set xacro arguments:
    if CONFIGURATION["ROB1_ee"] == "none":
        EE_1 = "false"
    else:
        EE_1 = "true"
    if CONFIGURATION["ROB2_ee"] == "none":
        EE_2 = "false"
    else:
        EE_2 = "true"

    # Generate tmp/controller.yaml and tmp/RViz files:
    ROBOTS = [CONFIGURATION["ROB1_rob"], CONFIGURATION["ROB2_rob"]]
    E1 = "none"
    E2 = "none"
    if EE_1 == "true":
        if EEctrlEXISTS(CONFIGURATION["ROB1_ee"]) == True:
            E1 = CONFIGURATION["ROB1_ee"]
        else:
            E1 = "none"
    if EE_2 == "true":
        if EEctrlEXISTS(CONFIGURATION["ROB2_ee"]) == True:
            E2 = CONFIGURATION["ROB2_ee"]
        else:
            E2 = "none"
    EES = [E1, E2]
    PREFIXES = [CONFIGURATION["ROB1_id"] + "_", CONFIGURATION["ROB2_id"] + "_"]
    
    TMP_CONTROLLER_PATH = CreateTMPControllerFile(ROBOTS, EES, PREFIXES)
    TMP_RVIZ_PATH_1 = CreateTMPRVizFile(
        CONFIGURATION["ROB1_rob"],
        CONFIGURATION["ROB1_ee"] if EE_1 == "true" else "none",
        CONFIGURATION["ROB1_id"] + "_",
        CONFIGURATION["ROB1_id"],
    )
    TMP_RVIZ_PATH_2 = CreateTMPRVizFile(
        CONFIGURATION["ROB2_rob"],
        CONFIGURATION["ROB2_ee"] if EE_2 == "true" else "none",
        CONFIGURATION["ROB2_id"] + "_",
        CONFIGURATION["ROB2_id"],
    )
    
    # PROCESS xacro file with ROBOT and EE information:
    xacro.process_doc(doc, mappings={

        "EE_1": EE_1,
        "EE_name_1": CONFIGURATION["ROB1_ee"],
        "prefix_1": CONFIGURATION["ROB1_id"] + "_",
    
        "EE_2": EE_2,
        "EE_name_2": CONFIGURATION["ROB2_ee"],
        "prefix_2": CONFIGURATION["ROB2_id"] + "_",

    })
    
    # END-EFFECTOR(s) -> Check if CONTROLLER file exists for EE, and set EE variable for LAUNCH DESCRIPTION:
    if EE_1 == "true":
        if EEctrlEXISTS(CONFIGURATION["ROB1_ee"]) == False:
            EE_1 = "true-NOctr"
    if EE_2 == "true":
        if EEctrlEXISTS(CONFIGURATION["ROB2_ee"]) == False:
            EE_2 = "true-NOctr"
    
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

    # SPAWN ROBOT TO GZ SIM:
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', CONFIGURATION["ID"],
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
    # Joint TRAJECTORY Controllers for ROB1:
    joint_trajectory_controller_spawner_1 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[CONFIGURATION["ROB1_id"] + "_joint_trajectory_controller", "-c", "/controller_manager"],
    )
    # Joint TRAJECTORY Controllers for ROB2:
    joint_trajectory_controller_spawner_2 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[CONFIGURATION["ROB2_id"] + "_joint_trajectory_controller", "-c", "/controller_manager"],
    )

    # EE CONTROLLERS for ROB1:
    CONTROLLER_NODES_1 = []
    if EE_1 == "true":
        CONTROLLERS_1 = GetEEctr(CONFIGURATION["ROB1_ee"])
        for x in CONTROLLERS_1:
            CONTROLLER_NODES_1.append(
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[CONFIGURATION["ROB1_id"] + "_" + x, "-c", "/controller_manager"],
                )
            )

    # EE CONTROLLERS for ROB2:
    CONTROLLER_NODES_2 = []
    if EE_2 == "true":
        CONTROLLERS_2 = GetEEctr(CONFIGURATION["ROB2_ee"])
        for x in CONTROLLERS_2:
            CONTROLLER_NODES_2.append(
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[CONFIGURATION["ROB2_id"] + "_" + x, "-c", "/controller_manager"],
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

    # =============================================== #
    # ================== MoveIt!2 =================== #

    # *** PLANNING CONTEXT *** #

    # ROBOT 1 - SRDF:
    if (EE_1 == "false"):
        srdf_file_1 = os.path.join(get_package_share_directory("ros2srrc_moveit"), "config", CONFIGURATION["ROB1_rob"] + ".srdf")
    else:
        srdf_file_1 = os.path.join(get_package_share_directory("ros2srrc_moveit"), "config", CONFIGURATION["ROB1_rob"] + "_" + CONFIGURATION["ROB1_ee"] + ".srdf")

    # ROBOT 2 - SRDF:
    if (EE_2 == "false"):
        srdf_file_2 = os.path.join(get_package_share_directory("ros2srrc_moveit"), "config", CONFIGURATION["ROB2_rob"] + ".srdf")
    else:
        srdf_file_2 = os.path.join(get_package_share_directory("ros2srrc_moveit"), "config", CONFIGURATION["ROB2_rob"] + "_" + CONFIGURATION["ROB2_ee"] + ".srdf")

    # Process SRDF files:
    srdf_doc_1 = xacro.parse(open(srdf_file_1))
    xacro.process_doc(srdf_doc_1, mappings={"prefix": CONFIGURATION["ROB1_id"] + "_", "name": CONFIGURATION["ID"]})
    srdf_doc_1.documentElement.setAttribute("name", CONFIGURATION["ID"])
    robot_description_semantic_config_1 = srdf_doc_1.toxml()
    robot_description_semantic_1 = {"robot_description_semantic": robot_description_semantic_config_1}
    
    srdf_doc_2 = xacro.parse(open(srdf_file_2))
    xacro.process_doc(srdf_doc_2, mappings={"prefix": CONFIGURATION["ROB2_id"] + "_", "name": CONFIGURATION["ID"]})
    srdf_doc_2.documentElement.setAttribute("name", CONFIGURATION["ID"])
    robot_description_semantic_config_2 = srdf_doc_2.toxml()
    robot_description_semantic_2 = {"robot_description_semantic": robot_description_semantic_config_2}

    # KINEMATICS CONFIGURATION:
    kinematics_yaml_1 = load_yaml("ros2srrc_robots", CONFIGURATION["ROB1_rob"] + "/config/kinematics.yaml")
    kinematics_yaml_1 = PrefixKinematicsKeys(kinematics_yaml_1, CONFIGURATION["ROB1_id"] + "_")
    kinematics_yaml_2 = load_yaml("ros2srrc_robots", CONFIGURATION["ROB2_rob"] + "/config/kinematics.yaml")
    kinematics_yaml_2 = PrefixKinematicsKeys(kinematics_yaml_2, CONFIGURATION["ROB2_id"] + "_")

    # JOINT LIMITS CONFIGURATION:
    # Get joint_limits.yaml for Robot1:
    if (EE_1 == "false") or (EE_1 == "true-NOctr"):
        joint_limits_yaml_1 = load_yaml("ros2srrc_robots", CONFIGURATION["ROB1_rob"] + "/config/joint_limits.yaml")
    else:
        YAML_ROB_1 = load_yaml("ros2srrc_robots", CONFIGURATION["ROB1_rob"] + "/config/joint_limits.yaml")["joint_limits"]
        YAML_EE_1 = load_yaml("ros2srrc_endeffectors", CONFIGURATION["ROB1_ee"] + "/config/joint_limits.yaml")["joint_limits"]
        joint_limits_yaml_1 = {}
        joint_limits_yaml_1["joint_limits"] = YAML_ROB_1 | YAML_EE_1
    # Get joint_limits.yaml for Robot2:
    if (EE_2 == "false") or (EE_2 == "true-NOctr"):
        joint_limits_yaml_2 = load_yaml("ros2srrc_robots", CONFIGURATION["ROB2_rob"] + "/config/joint_limits.yaml")
    else:
        YAML_ROB_2 = load_yaml("ros2srrc_robots", CONFIGURATION["ROB2_rob"] + "/config/joint_limits.yaml")["joint_limits"]
        YAML_EE_2 = load_yaml("ros2srrc_endeffectors", CONFIGURATION["ROB2_ee"] + "/config/joint_limits.yaml")["joint_limits"]
        joint_limits_yaml_2 = {}
        joint_limits_yaml_2["joint_limits"] = YAML_ROB_2 | YAML_EE_2
    
    # Process joint_limits for MoveIt!2:
    joint_limits_yaml_1 = PrefixJointLimits(joint_limits_yaml_1, CONFIGURATION["ROB1_id"] + "_")
    joint_limits_yaml_2 = PrefixJointLimits(joint_limits_yaml_2, CONFIGURATION["ROB2_id"] + "_")
    joint_limits_1 = {'robot_description_planning': joint_limits_yaml_1}
    joint_limits_2 = {'robot_description_planning': joint_limits_yaml_2}

    # PILZ PLANNING PIPELINE configuration:
    pilz_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
            "request_adapters": """ """,
            "start_state_max_bounds_error": 0.1,
            "default_planner_config": "PTP",
        }
    }

    # Pilz Planner Limits - Robot1:
    pilz_cartesian_limits_yaml_1 = load_yaml("ros2srrc_robots", CONFIGURATION["ROB1_rob"] + "/config/pilz_cartesian_limits.yaml")
    pilz_cartesian_limits_1 = {'robot_description_planning': pilz_cartesian_limits_yaml_1}
    # Pilz Planner Limits - Robot2:
    pilz_cartesian_limits_yaml_2 = load_yaml("ros2srrc_robots", CONFIGURATION["ROB2_rob"] + "/config/pilz_cartesian_limits.yaml")
    pilz_cartesian_limits_2 = {'robot_description_planning': pilz_cartesian_limits_yaml_2}

    # MoveIt!2 Controllers configuration:
    # Get controller.yaml for Robot1:
    RAW_1 = load_yaml("ros2srrc_robots", CONFIGURATION["ROB1_rob"] + "/config/controller_moveit2.yaml")
    if (EE_1 == "false") or (EE_1 == "true-NOctr"):
        MERGED_1 = RAW_1
    else:
        RAW_EE_1 = load_yaml("ros2srrc_endeffectors", CONFIGURATION["ROB1_ee"] + "/config/controller_moveit2.yaml")
        MERGED_1 = {"controller_names": RAW_1["controller_names"] + RAW_EE_1["controller_names"]}
        for C in RAW_1["controller_names"]:
            MERGED_1[C] = RAW_1[C]
        for C in RAW_EE_1["controller_names"]:
            MERGED_1[C] = RAW_EE_1[C]
    
    # Get controller.yaml for Robot2:
    RAW_2 = load_yaml("ros2srrc_robots", CONFIGURATION["ROB2_rob"] + "/config/controller_moveit2.yaml")
    if (EE_2 == "false") or (EE_2 == "true-NOctr"):
        MERGED_2 = RAW_2
    else:
        RAW_EE_2 = load_yaml("ros2srrc_endeffectors", CONFIGURATION["ROB2_ee"] + "/config/controller_moveit2.yaml")
        MERGED_2 = {"controller_names": RAW_2["controller_names"] + RAW_EE_2["controller_names"]}
        for C in RAW_2["controller_names"]:
            MERGED_2[C] = RAW_2[C]
        for C in RAW_EE_2["controller_names"]:
            MERGED_2[C] = RAW_EE_2[C]

    moveit_simple_controllers_yaml_1 = PrefixMoveItControllers(
        MERGED_1,
        CONFIGURATION["ROB1_id"] + "_",
        use_absolute_controller_names=True
    )

    moveit_simple_controllers_yaml_2 = PrefixMoveItControllers(
        MERGED_2,
        CONFIGURATION["ROB2_id"] + "_",
        use_absolute_controller_names=True
    )
    
    # Process MoveIt!2 controllers:
    moveit_controllers_1 = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml_1,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    moveit_controllers_2 = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml_2,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    # MoveIt!2 Parameters - GENERIC:
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

    # ===== MOVE GROUP ROS 2 Nodes ===== #

    # ROBOT 1:
    run_move_group_node_1 = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=CONFIGURATION["ROB1_id"],
        output="screen",
        remappings=[
            ("joint_states", "/joint_states"),
            ("/tf", "/tf"),
            ("/tf_static", "/tf_static"),
        ],
        parameters=[
            robot_description,
            robot_description_semantic_1,
            kinematics_yaml_1,
            
            pilz_planning_pipeline_config,

            joint_limits_1,
            pilz_cartesian_limits_1,

            trajectory_execution,
            moveit_controllers_1,
            planning_scene_monitor_parameters,
            move_group_capabilities,
            {"use_sim_time": True},
        ],
    )

    # ROBOT 2:
    run_move_group_node_2 = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=CONFIGURATION["ROB2_id"],
        output="screen",
        remappings=[
            ("joint_states", "/joint_states"),
            ("/tf", "/tf"),
            ("/tf_static", "/tf_static"),
        ],
        parameters=[
            robot_description,
            robot_description_semantic_2,
            kinematics_yaml_2,
            
            pilz_planning_pipeline_config,

            joint_limits_2,
            pilz_cartesian_limits_2,

            trajectory_execution,
            moveit_controllers_2,
            planning_scene_monitor_parameters,
            move_group_capabilities,
            {"use_sim_time": True},
        ],
    )

    # RViz - ROBOT 1:
    rviz_node_1 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_" + CONFIGURATION["ROB1_id"],
        output="log",
        arguments=["-d", TMP_RVIZ_PATH_1],
        parameters=[
            robot_description,
            robot_description_semantic_1,
            kinematics_yaml_1,
            pilz_planning_pipeline_config,
            joint_limits_1,
            pilz_cartesian_limits_1,
            trajectory_execution,
            moveit_controllers_1,
            planning_scene_monitor_parameters,
            move_group_capabilities,
            {"use_sim_time": True},
        ],
    )

    # RViz - ROBOT 2:
    rviz_node_2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_" + CONFIGURATION["ROB2_id"],
        output="log",
        arguments=["-d", TMP_RVIZ_PATH_2],
        parameters=[
            robot_description,
            robot_description_semantic_2,
            kinematics_yaml_2,
            pilz_planning_pipeline_config,
            joint_limits_2,
            pilz_cartesian_limits_2,
            trajectory_execution,
            moveit_controllers_2,
            planning_scene_monitor_parameters,
            move_group_capabilities,
            {"use_sim_time": True},
        ],
    )

    # =================================================================================================== #
    # ============================= ros2srrc_execution -> CUSTOM INTERFACES ============================= #

    # Move:
    if EE_1 == "true":
        MoveInterface_1 = Node(
            name="move_" + CONFIGURATION["ROB1_id"],
            package="ros2srrc_execution",
            executable="move",
            output="screen",
            parameters=[robot_description, 
                        robot_description_semantic_1, 
                        kinematics_yaml_1, 
    
                        {"use_sim_time": True}, 
                        {"ROB_PARAM": CONFIGURATION["ROB1_rob"]}, 
                        {"EE_PARAM": CONFIGURATION["ROB1_ee"]}, 
                        {"move_group_ns": CONFIGURATION["ROB1_id"]}, 
                        
                        {"ENV_PARAM": "gazebo"}
                        ],
        )
    else:
        MoveInterface_1 = Node(
            name="move_" + CONFIGURATION["ROB1_id"],
            package="ros2srrc_execution",
            executable="move",
            output="screen",
            parameters=[robot_description, 
                        robot_description_semantic_1, 
                        kinematics_yaml_1, 
    
                        {"use_sim_time": True}, 
                        {"ROB_PARAM": CONFIGURATION["ROB1_rob"]}, 
                        {"EE_PARAM": "none"},
                        {"move_group_ns": CONFIGURATION["ROB1_id"]},  

                        {"ENV_PARAM": "gazebo"}
                        ],
        )

    if EE_2 == "true":
        MoveInterface_2 = Node(
            name="move_" + CONFIGURATION["ROB2_id"],
            package="ros2srrc_execution",
            executable="move",
            output="screen",
            parameters=[robot_description, 
                        robot_description_semantic_2, 
                        kinematics_yaml_2, 
    
                        {"use_sim_time": True}, 
                        {"ROB_PARAM": CONFIGURATION["ROB2_rob"]}, 
                        {"EE_PARAM": CONFIGURATION["ROB2_ee"]}, 
                        {"move_group_ns": CONFIGURATION["ROB2_id"]},  

                        {"ENV_PARAM": "gazebo"}
                        ],
        )
    else:
        MoveInterface_2 = Node(
            name="move_" + CONFIGURATION["ROB2_id"],
            package="ros2srrc_execution",
            executable="move",
            output="screen",
            parameters=[robot_description, 
                        robot_description_semantic_2, 
                        kinematics_yaml_2, 
    
                        {"use_sim_time": True}, 
                        {"ROB_PARAM": CONFIGURATION["ROB2_rob"]}, 
                        {"EE_PARAM": "none"}, 
                        {"move_group_ns": CONFIGURATION["ROB2_id"]},  

                        {"ENV_PARAM": "gazebo"}
                        ],
        )

    # RobMove:
    RobMoveInterface_1 = Node(
        name="robmove_" + CONFIGURATION["ROB1_id"],
        package="ros2srrc_execution",
        executable="robmove",
        output="screen",
        parameters=[robot_description, 
                    robot_description_semantic_1, 
                    kinematics_yaml_1, 
                    
                    {"use_sim_time": True}, 
                    {"ROB_PARAM": CONFIGURATION["ROB1_rob"]},
                    {"move_group_ns": CONFIGURATION["ROB1_id"]},  
                    ],
    )
    RobMoveInterface_2 = Node(
        name="robmove_" + CONFIGURATION["ROB2_id"],
        package="ros2srrc_execution",
        executable="robmove",
        output="screen",
        parameters=[robot_description, 
                    robot_description_semantic_2, 
                    kinematics_yaml_2, 
                    
                    {"use_sim_time": True}, 
                    {"ROB_PARAM": CONFIGURATION["ROB2_rob"]},
                    {"move_group_ns": CONFIGURATION["ROB2_id"]}, 
                    ],
    )

    # RobPose:
    RobPoseInterface_1 = Node(
        name="robpose_" + CONFIGURATION["ROB1_id"],
        package="ros2srrc_execution",
        executable="robpose",
        output="screen",
        parameters=[robot_description, 
                    robot_description_semantic_1, 
                    kinematics_yaml_1, 
                    
                    {"use_sim_time": True}, 
                    {"ROB_PARAM": CONFIGURATION["ROB1_rob"]},
                    {"move_group_ns": CONFIGURATION["ROB1_id"]}, 
                    ],
    )
    RobPoseInterface_2 = Node(
        name="robpose_" + CONFIGURATION["ROB2_id"],
        package="ros2srrc_execution",
        executable="robpose",
        output="screen",
        parameters=[robot_description, 
                    robot_description_semantic_2, 
                    kinematics_yaml_2, 
                    
                    {"use_sim_time": True}, 
                    {"ROB_PARAM": CONFIGURATION["ROB2_rob"]},
                    {"move_group_ns": CONFIGURATION["ROB2_id"]}, 
                    ],
    )

    # =============================================== #
    # ========== RETURN LAUNCH DESCRIPTION ========== #

    # Add ROS 2 Nodes to LaunchDescription() element:
    LD.add_action(gzSIM)
    LD.add_action(clock_bridge)
    LD.add_action(gzSERVICE_bridge)
    LD.add_action(node_robot_state_publisher)
    LD.add_action(spawn_entity)

    LD.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action = spawn_entity,
            on_exit = [
                joint_state_broadcaster_spawner,
            ]
        )
    ))

    LD.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action = joint_state_broadcaster_spawner,
            on_exit = [
                joint_trajectory_controller_spawner_1,
            ]
        )
    ))

    LD.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action = joint_trajectory_controller_spawner_1,
            on_exit = [
                joint_trajectory_controller_spawner_2,
            ]
        )
    ))

    COND = joint_trajectory_controller_spawner_2

    if EE_1 == "true":
        for x in CONTROLLER_NODES_1:
            LD.add_action(RegisterEventHandler(
                OnProcessExit(
                    target_action = COND,
                    on_exit = [
                        x,
                        ]
                    )
                )
            )
            COND = x

    if EE_2 == "true":
        for x in CONTROLLER_NODES_2:
            LD.add_action(RegisterEventHandler(
                OnProcessExit(
                    target_action = COND,
                    on_exit = [
                        x,
                        ]
                    )
                )
            )
            COND = x

    LD.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=COND,
            on_exit=[
                TimerAction(
                    period=1.0,
                    actions=[
                        run_move_group_node_1,
                        run_move_group_node_2,
                    ]
                )
            ]
        )
    ))

    LD.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=COND,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[
                        rviz_node_1,
                        rviz_node_2,
                    ]
                )
            ]
        )
    ))

    LD.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=COND,
            on_exit=[
                TimerAction(
                    period=5.0,
                    actions=[
                        MoveInterface_1,
                        MoveInterface_2,
                        RobMoveInterface_1,
                        RobMoveInterface_2,
                        RobPoseInterface_1,
                        RobPoseInterface_2,
                    ]
                )
            ]
        )
    ))

    # ***** RETURN  ***** #
    return(LD)
