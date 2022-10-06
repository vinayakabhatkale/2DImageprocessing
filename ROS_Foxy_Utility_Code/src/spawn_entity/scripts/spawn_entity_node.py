#!/usr/bin/python3

import math
import argparse
import xml.etree.ElementTree as ET

import rclpy
from gazebo_msgs.srv import SpawnEntity, SetModelConfiguration


def main():
    # Get the arguments
    parser = argparse.ArgumentParser(description='Spawn URDF into Gazebo.')
    parser.add_argument('-description', type=str, help='URDF of the entity.')
    parser.add_argument('-name', type=str, help='Name of the entity.')
    parser.add_argument('-namespace', type=str, help='Namespace of the entity.')
    parser.add_argument('-x', type=str, help='X coordinate of the entity.')
    parser.add_argument('-y', type=str, help='Y coordinate of the entity.')
    parser.add_argument('-z', type=str, help='Z coordinate of the entity.')
    parser.add_argument('-R', type=str, help='Roll of the entity.')
    parser.add_argument('-P', type=str, help='Pitch of the entity.')
    parser.add_argument('-Y', type=str, help='Yaw of the entity.')
    parser.add_argument('-J', action='append',
                        nargs=2,
                        metavar=('joint', 'value'),
                        help='Joint name with its initial position.')

    args, _ = parser.parse_known_args()

    # Start the node
    rclpy.init()
    entity_spawner = rclpy.create_node('entity_spawner')

    # Robot spawn
    # Create client
    entity_spawner.get_logger().info("Creating service client connecting to '/spawn_entity'.")
    spawn_client = entity_spawner.create_client(SpawnEntity, '/spawn_entity')

    # Connect client
    entity_spawner.get_logger().info("Connecting to '/spawn_entity' service.")
    if not spawn_client.service_is_ready():
        entity_spawner.get_logger().info('Spawn service not ready, waiting.')
        spawn_client.wait_for_service()
    entity_spawner.get_logger().info('Spawn service connected.')

    # Compute rotation as quaternion
    q = euler2quaternion(float(args.R), float(args.P), float(args.Y))

    # Form request
    request = SpawnEntity.Request()
    request.name = args.name
    request.xml = args.description
    request.initial_pose.position.x = float(args.x)
    request.initial_pose.position.y = float(args.y)
    request.initial_pose.position.z = float(args.z)
    request.initial_pose.orientation.w = q[0]
    request.initial_pose.orientation.x = q[1]
    request.initial_pose.orientation.y = q[2]
    request.initial_pose.orientation.z = q[3]
    request.robot_namespace = args.namespace

    # Request spawn
    entity_spawner.get_logger().info('Spawning entity.')

    #entity_spawner.get_logger().info(args.description)
    """f = open(r"desc.urdf", "w")
    f.write(args.description)
    f.close()"""

    spawn_future = spawn_client.call_async(request)
    rclpy.spin_until_future_complete(entity_spawner, spawn_future)

    # Result from spawn attempt
    if spawn_future.result():
        entity_spawner.get_logger().info('{}'.format(spawn_future.result().status_message))
    else:
        entity_spawner.get_logger().error('{}'.format(spawn_future.exception()))

    # Robot initial joint position
    if args.J:
        # Create client
        entity_spawner.get_logger().info("Creating service client connecting to '/set_model_configuration'.")
        joint_client = entity_spawner.create_client(SetModelConfiguration, '/set_model_configuration')

        # Connect client
        entity_spawner.get_logger().info("Connecting to '/set_model_configuration' service.")
        if not joint_client.service_is_ready():
            entity_spawner.get_logger().info('Joint service not ready, waiting.')
            joint_client.wait_for_service()
        entity_spawner.get_logger().info('Joint service connected.')

        # Get list of valid joint value pairs
        valid_joints = get_valid_joints(args.description)
        joint_names = []
        joint_positions = []
        for joint, value in args.J:
            if joint in valid_joints:
                joint_names.append(joint)
                joint_positions.append(float(value))

        # Form request
        request = SetModelConfiguration.Request()
        request.model_name = args.name
        request.joint_names = joint_names
        request.joint_positions = joint_positions

        # Request positions
        entity_spawner.get_logger().info('Set joint position.')
        joint_future = joint_client.call_async(request)
        rclpy.spin_until_future_complete(entity_spawner, joint_future)

        # Result from spawn attempt
        if joint_future.result():
            entity_spawner.get_logger().info('{}'.format(joint_future.result()))
        else:
            entity_spawner.get_logger().error('{}'.format(joint_future.exception()))

    # Exiting
    entity_spawner.get_logger().info('Finished. Node shutting down.')
    entity_spawner.destroy_node()
    rclpy.shutdown()


def euler2quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0, 0, 0, 0]
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q


def get_valid_joints(urdf):
    joint_list = []
    root = ET.fromstring(urdf)
    for joint in root.iter('joint'):
        joint_list.append(joint.attrib['name'])
    return set(joint_list)


if __name__ == '__main__':
    main()
