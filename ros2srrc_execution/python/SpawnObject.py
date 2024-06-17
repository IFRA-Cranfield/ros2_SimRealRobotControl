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
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import rclpy

def main():

    # Get input arguments from user:
    parser = argparse.ArgumentParser(description='Spawn object into our Gazebo world.')
    parser.add_argument('--package', type=str, default='', help='Package where URDF/XACRO file is located.')
    parser.add_argument('--urdf', type=str, default='', help='URDF of the object to spawn.')
    parser.add_argument('--name', type=str, default='', help='Name of the object to spawn.')
    parser.add_argument('--namespace', type=str, default='', help='ROS namespace to apply to the tf and plugins.')
    parser.add_argument('--ns', type=bool, default=True, help='Whether to enable namespacing')
    parser.add_argument('--x', type=float, default=0.0, help='the x component of the initial position [meters].')
    parser.add_argument('--y', type=float, default=0.0, help='the y component of the initial position [meters].')
    parser.add_argument('--z', type=float, default=0.0, help='the z component of the initial position [meters].')
    
    args, unknown = parser.parse_known_args()

    # Start node:
    rclpy.init()
    node = rclpy.create_node('entity_spawner')

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, '/spawn_entity')

    node.get_logger().info('Connecting to `/spawn_entity` service...')
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info('...connected!')

    # Set data for request:
    request = SpawnEntity.Request()
    request.name = args.name

    urdf_file_path = os.path.join(get_package_share_directory(args.package), 'urdf', 'objects', args.urdf) # It is assumed that the .urdf/.xacro file is located in /urdf/objects folder!
    xacro_file = xacro.process_file(urdf_file_path, mappings={"name": args.name})
    request.xml = xacro_file.toxml()

    request.initial_pose.position.x = float(args.x)
    request.initial_pose.position.y = float(args.y)
    request.initial_pose.position.z = float(args.z)

    if args.namespace is True:
        node.get_logger().info('spawning `{}` on namespace `{}` at {}, {}, {}'.format(
            args.name, args.namespace, args.x, args.y, args.z))

        request.namespace = args.namespace
        print(args.namespace)

    else:
        node.get_logger().info('spawning `{}` at {}, {}, {}'.format(
            args.name, args.x, args.y, args.z))

    node.get_logger().info('Spawning OBJECT using service: `/spawn_entity`')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info('Done! Shutting down node.')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()