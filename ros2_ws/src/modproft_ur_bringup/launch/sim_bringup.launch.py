import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ur_bringup = os.path.join(
        get_package_share_directory('modproft_ur_bringup'), 'launch', 'sim_control.launch.py')

    moveit_config = os.path.join(
        get_package_share_directory('modproft_ur_bringup'), 'launch', 'ur_moveit.launch.py')
    
    octomap = os.path.join(
        get_package_share_directory('octomap_server2'), 'launch', 'octomap_server_launch.py')


    ur_bringup = IncludeLaunchDescription(PythonLaunchDescriptionSource(ur_bringup),
                    launch_arguments={"ur_type": "ur5e",
                                      "robot_ip":"yyy.yyy.yyy.yyy",
                                      "use_fake_hardware":"true",
                                      "launch_rviz":"false",
                                     }.items())

    moveit_config = IncludeLaunchDescription(PythonLaunchDescriptionSource(moveit_config),
                    launch_arguments={"ur_type": "ur5e",
                                      "robot_ip":"xxx.xxx",
                                      "use_fake_hardware":"true",
                                      "launch_rviz":"true",
                                     }.items())

    
    octomap = IncludeLaunchDescription(PythonLaunchDescriptionSource(octomap))

    octomap_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="octomap_tf",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "custom_camera_link", "world"],
        parameters=[{'use_sim_time': 'True'}]
    )

    ld.add_action(ur_bringup)
    ld.add_action(moveit_config)
    ld.add_action(octomap_tf)
    ld.add_action(octomap)


    return ld