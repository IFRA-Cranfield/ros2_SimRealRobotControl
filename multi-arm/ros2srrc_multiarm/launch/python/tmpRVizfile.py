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

# tmpRVizfile.py:
# Utilities to generate a temporary RViz configuration file from an existing
# MoveIt RViz template. The output file is intended to be used by the RViz
# Visualization Tool's node, for example:
#   /tmp/multiarm_rviz_rob1.rviz

# [SHORT NOTE]: This file has been generated using CODEX.

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

LOG_PREFIX = "[TMP-RVizFile]"


class _IndentedSafeDumper(yaml.SafeDumper):
    def increase_indent(self, flow=False, indentless=False):
        return super().increase_indent(flow, False)


def _message(text):
    return f"{LOG_PREFIX} {text}"


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


def _get_template_rviz_path(robot_name, ee_name):

    moveit_root = _get_database_root("ros2srrc_moveit", "ros2srrc_moveit")
    rviz_path = moveit_root / "config" / "ros2srrc.rviz"

    if not rviz_path.exists():
        raise FileNotFoundError(
            _message(f"RViz template file not found: '{rviz_path}'.")
        )

    return rviz_path


def _normalize_single_value(value, label):

    if isinstance(value, (list, tuple)):
        if len(value) != 1:
            raise ValueError(
                _message(
                    f"{label} must describe exactly one robot per file generation call."
                )
            )
        value = value[0]

    if not isinstance(value, str) or not value.strip():
        raise ValueError(_message(f"Invalid {label}."))

    return value.strip()


def _namespace_from_prefix(prefix):
    return prefix[:-1] if prefix.endswith("_") else prefix


def _arm_group_from_robot(robot_name, prefix):
    return prefix + robot_name + "_arm"


def _patch_motion_planning_display(display, robot_name, prefix, namespace):

    display["Move Group Namespace"] = namespace

    planned_path = display.get("Planned Path", {})
    if isinstance(planned_path, dict):
        planned_path["Trajectory Topic"] = f"/{namespace}/display_planned_path"

    planning_request = display.get("Planning Request", {})
    if isinstance(planning_request, dict):
        planning_request["Planning Group"] = _arm_group_from_robot(robot_name, prefix)

    display["Planning Scene Topic"] = f"/{namespace}/monitored_planning_scene"


def _patch_rviz_template(rviz_yaml, robot_name, prefix):

    namespace = _namespace_from_prefix(prefix)
    manager = rviz_yaml.get("Visualization Manager", {})
    displays = manager.get("Displays", [])

    if not isinstance(displays, list):
        raise ValueError(_message("Invalid RViz template: Displays must be a list."))

    for display in displays:
        if not isinstance(display, dict):
            continue
        if display.get("Class") == "moveit_rviz_plugin/MotionPlanning":
            _patch_motion_planning_display(display, robot_name, prefix, namespace)

    return rviz_yaml


def create_tmp_rviz_file(
    robot_name,
    ee_name,
    prefix,
    id=None,
    output_dir="/tmp",
):

    robot_name = _normalize_single_value(robot_name, "robot_name")
    ee_name = _normalize_single_value(ee_name, "ee_name")
    prefix = _normalize_single_value(prefix, "prefix")

    template_rviz_path = _get_template_rviz_path(robot_name, ee_name)
    output_yaml = _load_yaml_file(template_rviz_path)
    output_yaml = _patch_rviz_template(output_yaml, robot_name, prefix)

    namespace = _namespace_from_prefix(prefix)
    suffix = _normalize_single_value(id, "id") if id is not None else namespace
    output_path = Path(output_dir) / f"multiarm_rviz_{suffix}.rviz"
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with output_path.open("w", encoding="utf-8") as output_file:
        yaml.dump(
            output_yaml,
            output_file,
            Dumper=_IndentedSafeDumper,
            sort_keys=False,
            default_flow_style=False,
        )

    print(_message(f"Generated temporary RViz file: {output_path}"))

    return str(output_path)


def CreateTMPRVizFile(robot_name, ee_name, prefix, id=None):
    return create_tmp_rviz_file(robot_name, ee_name, prefix, id)
