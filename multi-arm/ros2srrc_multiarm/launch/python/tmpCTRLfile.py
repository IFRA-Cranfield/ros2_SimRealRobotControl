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

# tmpCTRLfile.py:
# Utilities to generate a temporary ROS 2 control YAML file for any number of
# robots and optional end-effectors. The output file is intended to be used by
# the GZ ROS 2 control plugin, for example:
#   /tmp/multiarm_controller.yaml

# [SHORT NOTE]: This file has been generated using CODEX.

import copy
from pathlib import Path

import yaml

try:
    from ament_index_python.packages import (
        PackageNotFoundError,
        get_package_share_directory,
    )
except ImportError:
    PackageNotFoundError = Exception
    get_package_share_directory = None

LOG_PREFIX = "[TMP-ControllerFile]"

class _IndentedSafeDumper(yaml.SafeDumper):
    def increase_indent(self, flow=False, indentless=False):
        return super().increase_indent(flow, False)

def _message(text):
    return f"{LOG_PREFIX} {text}"

# Try to resolve the database folder from the current workspace so this also
# works before the package has been installed/sourced through ROS 2:
def _find_directory_in_workspace(directory_name):

    current_file = Path(__file__).resolve()
    candidates = [Path.cwd().resolve(), current_file.parent]
    candidates.extend(Path.cwd().resolve().parents)
    candidates.extend(current_file.parents)

    checked = set()
    for candidate in candidates:
        if candidate in checked:
            continue
        checked.add(candidate)

        directory_path = candidate / directory_name
        if directory_path.is_dir():
            return directory_path

    raise FileNotFoundError(
        _message(
            f"Could not locate '{directory_name}' from '{Path.cwd()}' or '{current_file}'."
        )
    )

# Prefer the installed ROS 2 package share path, but fall back to the
# local source workspace during development:
def _get_database_root(package_name, directory_name):

    if get_package_share_directory is not None:
        try:
            return Path(get_package_share_directory(package_name))
        except (PackageNotFoundError, ValueError):
            pass

    return _find_directory_in_workspace(directory_name)

def _load_yaml_file(yaml_path):
    with yaml_path.open("r", encoding="utf-8") as yaml_file:
        data = yaml.safe_load(yaml_file)

    if not isinstance(data, dict):
        raise ValueError(_message(f"Invalid YAML structure in '{yaml_path}'."))

    return data

def _get_robot_controller_yaml_path(robot_name):
    robots_root = _get_database_root("ros2srrc_robots", "ros2srrc_robots")
    controller_yaml_path = robots_root / robot_name / "config" / "controller.yaml"

    if not controller_yaml_path.exists():
        raise FileNotFoundError(
            _message(
                f"Robot controller file not found for '{robot_name}': '{controller_yaml_path}'."
            )
        )

    return controller_yaml_path

def _get_ee_controller_yaml_path(ee_name):
    ee_root = _get_database_root("ros2srrc_endeffectors", "ros2srrc_endeffectors")
    controller_yaml_path = ee_root / ee_name / "config" / "controller.yaml"

    if not controller_yaml_path.exists():
        raise FileNotFoundError(
            _message(
                f"End-effector controller file not found for '{ee_name}': '{controller_yaml_path}'."
            )
        )

    return controller_yaml_path

def _prefix_joint_fields(value, prefix):

    if isinstance(value, dict):
        prefixed_value = {}

        for key, item in value.items():

            # Controller YAMLs only need joint-related names to be rewritten:
            if key == "joint" and isinstance(item, str):
                prefixed_value[key] = prefix + item
            elif key == "joints" and isinstance(item, list):
                prefixed_value[key] = [
                    prefix + joint if isinstance(joint, str) else joint for joint in item
                ]
            else:
                prefixed_value[key] = _prefix_joint_fields(item, prefix)
        return prefixed_value

    if isinstance(value, list):
        return [_prefix_joint_fields(item, prefix) for item in value]

    return value

def _merge_prefixed_controller_yaml(output_yaml, source_yaml, prefix):

    # Add the controller entries under controller_manager, skipping the common
    # sections that must appear only once in the merged file:

    manager_parameters = source_yaml.get("controller_manager", {}).get("ros__parameters", {})
    if not isinstance(manager_parameters, dict):
        raise ValueError(_message("Invalid controller_manager.ros__parameters section."))

    output_manager_parameters = output_yaml["controller_manager"]["ros__parameters"]

    for controller_name, controller_value in manager_parameters.items():
        if controller_name == "update_rate":
            continue
        if controller_name == "joint_state_broadcaster":
            continue

        prefixed_controller_name = prefix + controller_name
        if prefixed_controller_name in output_manager_parameters:
            raise ValueError(
                _message(
                    f"Controller name collision detected for '{prefixed_controller_name}'."
                )
            )

        output_manager_parameters[prefixed_controller_name] = copy.deepcopy(controller_value)

    for section_name, section_value in source_yaml.items():
        if section_name == "controller_manager":
            continue

        # Duplicate each controller block under its new prefixed name and update
        # any joint references inside it:
        prefixed_section_name = prefix + section_name
        if prefixed_section_name in output_yaml:
            raise ValueError(
                _message(
                    f"Controller configuration collision detected for '{prefixed_section_name}'."
                )
            )

        output_yaml[prefixed_section_name] = _prefix_joint_fields(
            copy.deepcopy(section_value), prefix
        )

def _validate_inputs(robot_names, ee_names, prefixes):

    # These three arrays describe one logical robot entry per index:
    if not robot_names:
        raise ValueError(_message("At least one robot must be provided."))

    if len(robot_names) != len(ee_names) or len(robot_names) != len(prefixes):
        raise ValueError(
            _message("robot_names, ee_names and prefixes must all have the same length.")
        )

    for index, robot_name in enumerate(robot_names):
        if not isinstance(robot_name, str) or not robot_name.strip():
            raise ValueError(_message(f"Invalid robot name at index {index}."))

    for index, prefix in enumerate(prefixes):
        if not isinstance(prefix, str):
            raise ValueError(_message(f"Invalid prefix at index {index}."))

    if len(set(prefixes)) != len(prefixes):
        raise ValueError(
            _message("All prefixes must be unique to avoid controller name collisions.")
        )

def create_tmp_controller_file(
    robot_names,
    ee_names,
    prefixes,
    id=None,
    output_dir="/tmp",
    update_rate=250,
):

    # Build one merged controller YAML containing all robot arm controllers and,
    # when present, all end-effector controllers:

    _validate_inputs(robot_names, ee_names, prefixes)

    output_yaml = {
        "controller_manager": {
            "ros__parameters": {
                "update_rate": update_rate,
                "joint_state_broadcaster": {
                    "type": "joint_state_broadcaster/JointStateBroadcaster"
                },
            }
        }
    }

    for robot_name, ee_name, prefix in zip(robot_names, ee_names, prefixes):

        # Every robot contributes its joint trajectory controller block:
        robot_yaml_path = _get_robot_controller_yaml_path(robot_name)
        robot_yaml = _load_yaml_file(robot_yaml_path)
        _merge_prefixed_controller_yaml(output_yaml, robot_yaml, prefix)

        # End-effectors are optional; "none" means skip them for that robot:
        if isinstance(ee_name, str) and ee_name.lower() != "none":
            ee_yaml_path = _get_ee_controller_yaml_path(ee_name)
            ee_yaml = _load_yaml_file(ee_yaml_path)
            _merge_prefixed_controller_yaml(output_yaml, ee_yaml, prefix)

    # The launch file can point GZ Sim directly to this generated file:
    output_path = Path(output_dir) / "multiarm_controller.yaml"
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with output_path.open("w", encoding="utf-8") as output_file:
        yaml.dump(
            output_yaml,
            output_file,
            Dumper=_IndentedSafeDumper,
            sort_keys=False,
            default_flow_style=False,
        )

    print(_message(f"Generated temporary controller file: {output_path}"))

    return str(output_path)

def CreateTMPControllerFile(robot_names, ee_names, prefixes, id=None):
    return create_tmp_controller_file(robot_names, ee_names, prefixes, id)