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

# simulation.launch.py:
# Launch file for the (2) ROBOT's GZ SIM / Gazebo Fortress simulation in ROS 2 Humble:

# Import libraries:
import os, sys, xacro, yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

TMPCTRLFILE_PATH = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "python"))
if TMPCTRLFILE_PATH not in sys.path:
    sys.path.append(TMPCTRLFILE_PATH)

from tmpCTRLfile import CreateTMPControllerFile

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
    print("===== GZ SIM: Robot Simulation (" + PACKAGE_NAME + ") =====")
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

    # Generate tmp/controller.yaml file:
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
    # ========== RETURN LAUNCH DESCRIPTION ========== #

    # Add ROS 2 Nodes to LaunchDescription() element:
    LD.add_action(gzSIM)
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

    # ***** RETURN  ***** #
    return(LD)