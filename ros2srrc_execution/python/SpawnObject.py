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

# IMPORT LIBRARIES:

import argparse
import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

from ros_gz_interfaces.srv import SpawnEntity
from geometry_msgs.msg import Pose


class GzEntitySpawner(Node):

    def __init__(self, args):

        super().__init__('objectpose_GzEntitySpawner')
        self.args = args

        self.service_name = f'/world/{self.args.world}/create'
        self.get_logger().info(f'Connecting to `{self.service_name}` ...')

        self.cli = self.create_client(SpawnEntity, self.service_name)
        while not self.cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().info(f'Waiting for service {self.service_name} ...')
        self.get_logger().info('...connected!')

    def _resolve_sdf_path(self) -> str:

        if os.path.isabs(self.args.sdf):
            return self.args.sdf
        share = get_package_share_directory(self.args.package)
        for sub in ('sdf', 'models', 'objects'):
            cand = os.path.join(share, sub, self.args.sdf)
            if os.path.exists(cand):
                return cand
        raise FileNotFoundError(
            f"'{self.args.sdf}' not found under {share}/sdf. {share}/models, or {share}/objects, "
            "and it is not an absolute path."
        )

    def build_sdf_string(self) -> str:

        # Read the SDF file text:
        sdf_path = self._resolve_sdf_path()
        with open(sdf_path, 'r') as f:
            xml = f.read()
        
        # Compute absolute path to the mesh: 
        share = get_package_share_directory(self.args.package)
        sdf_base = os.path.splitext(os.path.basename(self.args.sdf))[0]
        mesh_path = os.path.join(share, 'meshes', 'objects', f'{sdf_base}.dae')

        if not os.path.exists(mesh_path):
            self.get_logger().info(f"Mesh not found: {mesh_path}. Ignore this error if the sdf file does not use a mesh.")
        
        # Convert to a correct file URI:
        mesh_uri = Path(mesh_path).as_uri()   

        # Do your existing replacements plus the mesh:
        xml = xml.replace('$(name)', self.args.name)
        xml = xml.replace('$(mesh_uri)', mesh_uri)
        return xml

    def spawn(self):

        req = SpawnEntity.Request()
        req.entity_factory.name = self.args.name
        req.entity_factory.allow_renaming = True          
        req.entity_factory.relative_to = "world"
        req.entity_factory.sdf = self.build_sdf_string()

        pose = Pose()
        pose.position.x = float(self.args.x)
        pose.position.y = float(self.args.y)
        pose.position.z = float(self.args.z)
        req.entity_factory.pose = pose

        self.get_logger().info(
            f"Spawning `{self.args.name}` at ({self.args.x}, {self.args.y}, {self.args.z})"
        )

        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res is not None:
            self.get_logger().info(f"Spawn success={res.success} msg='{getattr(res, 'status_message', '')}'")
            if not res.success:
                raise RuntimeError(f"Spawn failed: {getattr(res, 'status_message', '')}")
        else:
            raise RuntimeError(f'Exception calling service: {fut.exception()}')


def main():

    parser = argparse.ArgumentParser(description='Spawn an SDF model into a Gazebo (Gz Fortress) world.')

    parser.add_argument('--package', type=str, required=True, help='Package where the SDF file is installed.')
    parser.add_argument('--sdf', type=str, default='box.sdf', help='SDF filename (relative to the package share/sdf or absolute path).')
    parser.add_argument('--name', type=str, required=True, help='Model instance name (also used to replace $(name) in the SDF).')
    parser.add_argument('--x', type=float, default=0.0, help='Initial X [m].')
    parser.add_argument('--y', type=float, default=0.0, help='Initial Y [m].')
    parser.add_argument('--z', type=float, default=0.2, help='Initial Z [m].')
    parser.add_argument('--world', type=str, default='ros2srrc_GzWorld', help='Target world name (default: ros2srrc_GzWorld).')

    args, _ = parser.parse_known_args()

    rclpy.init()
    node = GzEntitySpawner(args)
    try:
        node.spawn()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()