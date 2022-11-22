import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()


    ur_bringup = os.path.join(
        get_package_share_directory('modproft_ur_bringup'), 'launch', 'real_ur_control.launch.py')

    moveit_config = os.path.join(
        get_package_share_directory('ur_bringup'), 'launch', 'ur_moveit.launch.py')


    ur_bringup = IncludeLaunchDescription(PythonLaunchDescriptionSource(ur_bringup),
                    launch_arguments={"ur_type": "ur5e",
                                      "robot_ip":"192.168.1.20",
                                      "use_fake_hardware":"false",
                                      "launch_rviz":"false",
                                     }.items())

    moveit_config = IncludeLaunchDescription(PythonLaunchDescriptionSource(moveit_config),
                    launch_arguments={"ur_type": "ur5e",
                                      "robot_ip":"192.168.1.20",
                                      "use_fake_hardware":"false",
                                      "launch_rviz":"true",
                                     }.items())

    ld.add_action(ur_bringup)
    ld.add_action(moveit_config)


    return ld