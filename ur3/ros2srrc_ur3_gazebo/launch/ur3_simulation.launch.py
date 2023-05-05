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
#  Date: May, 2023.                                                                     #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023) ROS 2 Sim-to-Real Robot Control. URL: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl.

# ur3_simulation.launch.py:
# Launch file for the UR3 Robot GAZEBO SIMULATION in ROS2 Humble:

# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
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
    
    # ***** GAZEBO ***** #   
    # DECLARE Gazebo WORLD file:
    ros2srrc_ur3_gazebo = os.path.join(
        get_package_share_directory('ros2srrc_ur3_gazebo'),
        'worlds',
        'ur3.world')
    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={'world': ros2srrc_ur3_gazebo}.items(),
             )


    # ========== COMMAND LINE ARGUMENTS ========== #
    print("")
    print("===== Universal Robots - UR3: Robot Simulation (ros2srrc_ur3_gazebo) =====")
    print("Robot configuration:")
    print("")
    # Cell Layout:
    print("- Cell layout:")
    error = True
    while (error == True):
        print("     + Option N1: UR3 alone.")
        print("     + Option N2: UR3 in Cylindric Stand.")
        print("     + Option N3: UR3 in Cranfield University IA Lab.")
        cell_layout = input ("  Please select: ")
        if (cell_layout == "1"):
            error = False
            cell_layout_1 = "true"
            cell_layout_2 = "false"
            cell_layout_3 = "false"
        elif (cell_layout == "2"):
            error = False
            cell_layout_1 = "false"
            cell_layout_2 = "true"
            cell_layout_3 = "false"
        elif (cell_layout == "3"):
            error = False
            cell_layout_1 = "false"
            cell_layout_2 = "false"
            cell_layout_3 = "true"
        else:
            print ("  Please select a valid option!")
    print("")
    # End-Effector:
    print("- End-effector:")
    error = True
    while (error == True):
        print("     + Option N1: No end-effector.")
        print("     + Option N2: Robotiq 2f-85 parallel gripper.")
        end_effector = input ("  Please select: ")
        if (end_effector == "1"):
            error = False
            EE_no = "true"
            EE_robotiq = "false"
        elif (end_effector == "2"):
            error = False
            EE_no = "false"
            EE_robotiq = "true"
        else:
            print ("  Please select a valid option!")
    print("")

    # ***** ROBOT DESCRIPTION ***** #
    # UR3 Description file package:
    ur3_description_path = os.path.join(
        get_package_share_directory('ros2srrc_ur3_gazebo'))
    # UR3 ROBOT urdf file path:
    xacro_file = os.path.join(ur3_description_path,
                              'urdf',
                              'ur3.urdf.xacro')
    # Generate ROBOT_DESCRIPTION for UR3:
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={
        "cell_layout_1": cell_layout_1,
        "cell_layout_2": cell_layout_2,
        "cell_layout_3": cell_layout_3,
        "EE_no": EE_no,
        "EE_robotiq": EE_robotiq,
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

    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'ur3'],
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
        arguments=["ur_controller", "-c", "/controller_manager"],
    )
    # === ROBOTIQ 2f-85 CONTROLLER === #
    robotiq_controller_spawner_LKJ = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_controller_LKJ", "-c", "/controller_manager"],
    )
    robotiq_controller_spawner_RKJ = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_controller_RKJ", "-c", "/controller_manager"],
    )
    robotiq_controller_spawner_LIKJ = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_controller_LIKJ", "-c", "/controller_manager"],
    )
    robotiq_controller_spawner_RIKJ = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_controller_RIKJ", "-c", "/controller_manager"],
    )
    robotiq_controller_spawner_LFTJ = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_controller_LFTJ", "-c", "/controller_manager"],
    )
    robotiq_controller_spawner_RFTJ = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_controller_RFTJ", "-c", "/controller_manager"],
    )

    # ========== END-EFFECTORS ========== #
    # ========== END-EFFECTORS ========== #

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        
        gazebo, 
        node_robot_state_publisher,
        spawn_entity,

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
                    robotiq_controller_spawner_LKJ,
                    robotiq_controller_spawner_RKJ,
                    robotiq_controller_spawner_LIKJ,
                    robotiq_controller_spawner_RIKJ,
                    robotiq_controller_spawner_LFTJ,
                    robotiq_controller_spawner_RFTJ,
                ]
            )
        ),

    ])