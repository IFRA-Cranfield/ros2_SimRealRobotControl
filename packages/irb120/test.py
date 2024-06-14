import os, sys, yaml
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None
    
YAML_ROB = load_yaml("ros2srrc_robots", "irb120/config/controller_moveit2.yaml")
YAML_EE = load_yaml("ros2srrc_endeffectors", "egp64/config/controller_moveit2.yaml")

print(YAML_ROB)
print(YAML_EE)

for x in YAML_ROB["controller_names"]:

    YAML_EE["controller_names"].append(x)
    
moveit_simple_controllers_yaml = YAML_ROB | YAML_EE

print(moveit_simple_controllers_yaml)