import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ur_bringup = os.path.join(
        get_package_share_directory('modproft_ur_bringup'), 'launch', 'ur_control.launch.py')

    moveit_config = os.path.join(
        get_package_share_directory('modproft_ur_bringup'), 'launch', 'ur_moveit.launch.py')

    adapter_bringup = os.path.join(
        get_package_share_directory('modproft_adapter_description'), 'launch', 'spawn_adapter.launch.py')

    pcb_bringup = os.path.join(
        get_package_share_directory('modproft_pcb_description'), 'launch', 'spawn_multiple_pcb.launch.py')

    ur_bringup = IncludeLaunchDescription(PythonLaunchDescriptionSource(ur_bringup),
                    launch_arguments={"ur_type": "ur5e",
                                      "robot_ip":"yyy.yyy.yyy.yyy",
                                      "use_fake_hardware":"true",
                                      "launch_rviz":"false",
                                      "controllers_file":"sim_controllers.yaml"
                                     }.items())

    moveit_config = IncludeLaunchDescription(PythonLaunchDescriptionSource(moveit_config),
                    launch_arguments={"ur_type": "ur5e",
                                      "robot_ip":"xxx.xxx",
                                      "use_fake_hardware":"true",
                                      "launch_rviz":"true",
                                      "use_sim_time":"true",
                                     }.items())

    adapter_bringup = IncludeLaunchDescription(PythonLaunchDescriptionSource(adapter_bringup),
                    launch_arguments={"package_name": "modproft_adapter_description",
                                      "position": "-750.0 0.0 0.0"
                                      }.items())

    pcb_bringup = IncludeLaunchDescription(PythonLaunchDescriptionSource(pcb_bringup),
                    launch_arguments={"package_name": "modproft_pcb_description",
                                      "offset": "0.0 750.0 0.0"
                                      }.items())

    ld.add_action(ur_bringup)
    ld.add_action(moveit_config)
    ld.add_action(adapter_bringup)
    ld.add_action(pcb_bringup)

    return ld
