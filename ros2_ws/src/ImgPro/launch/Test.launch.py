#!/usr/bin/env python3

# ROS launch system
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument,
                            OpaqueFunction)
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):

    # Initialize arguments
    frequency = LaunchConfiguration("frequency")
    device_name = LaunchConfiguration("device_name")

    # Define the start of the video publisher
    publisher_node = Node(
        package="ImgPro",
        executable="VideoPublisher2D.py",
        parameters=[
            {
                "frequency": frequency,
                "device_name": device_name
            }
        ],
        output="screen"
    )

    # Define the start of the video monitor
    monitor_node = Node(
        package="ImgPro",
        executable="VideoMonitor2D.py",
        output="screen"
    )

    nodes_to_start = [
        publisher_node,
        monitor_node
    ]
    return nodes_to_start


def generate_launch_description():

    # List of parameter arguments
    declared_arguments = []

    # Declare frequency as launch argument
    declared_arguments.append(
        DeclareLaunchArgument(
            "frequency",
            description="The publishing frequency of the camera stream",
            default_value="20.0"
        )
    )

    # Declare device_name as launch argument
    declared_arguments.append(
        DeclareLaunchArgument(
            "device_name",
            description="The device the video stream should be generated from",
            default_value="0"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
