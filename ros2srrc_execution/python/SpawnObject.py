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
import xacro
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# Gz (Fortress) service interface:
from ros_gz_interfaces.srv import SpawnEntity 
from geometry_msgs.msg import Pose

# EntitySpawner CLASS:
class GzEntitySpawner(Node):

    def __init__(self, args):

        super().__init__('ros2srrc_GzEntitySpawner')
        self.args = args

        self.service_name = '/world/ros2srrc_GzWorld/create'
        self.get_logger().info(f'Connecting to `{self.service_name}` ...')
        
        self.cli = self.create_client(SpawnEntity, self.service_name)

        if not self.cli.service_is_ready():
            self.cli.wait_for_service()
            self.get_logger().info('...connected!')

    def build_urdf_string(self):
        
        urdf_file_path = os.path.join(
            get_package_share_directory(self.args.package),
            'urdf', 'objects', self.args.urdf
        )
        
        x = xacro.process_file(urdf_file_path, mappings={"name": self.args.name})
        return x.toxml()

    def spawn(self):

        req = SpawnEntity.Request()

        req.entity_factory.name = self.args.name
        req.entity_factory.allow_renaming = False 
        req.entity_factory.relative_to = "world"

        req.entity_factory.sdf = self.build_urdf_string()

        pose = Pose()
        pose.position.x = float(self.args.x)
        pose.position.y = float(self.args.y)
        pose.position.z = float(self.args.z)
        # (orientation left at default 0,0,0,1)
        req.entity_factory.pose = pose

        self.get_logger().info(
            f"Spawning `{self.args.name}` into world Gz Simulation at "
            f"({self.args.x}, {self.args.y}, {self.args.z})"
        )

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Spawn success.')
            else:
                self.get_logger().error('Spawn failed (service returned false).')
        else:
            raise RuntimeError(f'Exception calling service: {future.exception()}')

def main():

    parser = argparse.ArgumentParser(description='Spawn object into a Gazebo (Gz Fortress) world.')
    parser.add_argument('--package', type=str, required=True, help='Package where URDF/XACRO file is located.')
    parser.add_argument('--urdf', type=str, required=True, help='URDF/XACRO file name under /urdf/objects.')
    parser.add_argument('--name', type=str, required=True, help='Name of the object to spawn.')
    parser.add_argument('--x', type=float, default=0.0, help='Initial X [m].')
    parser.add_argument('--y', type=float, default=0.0, help='Initial Y [m].')
    parser.add_argument('--z', type=float, default=0.0, help='Initial Z [m].')

    parser.add_argument('--namespace', type=str, default='', help='ROS namespace (handled in your URDF/SDF/plugins).')
    parser.add_argument('--ns', type=bool, default=True, help='Whether to enable namespacing (no-op for the service).')

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