import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    package_name_lc = LaunchConfiguration("package_name")
    package_name = package_name_lc.perform(context)
    box_type = LaunchConfiguration("box_type")
    box_type = box_type.perform(context)
    adapter_type = LaunchConfiguration("adapter_type")
    adapter_type = adapter_type.perform(context)
    adapter_name = LaunchConfiguration("adapter_name")
    adapter_name = adapter_name.perform(context)
    position_lc = LaunchConfiguration("position")
    position = position_lc.perform(context)

    spawn_entity = os.path.join(get_package_share_directory('spawn_entity'), 'launch',
                                'spawn_entity.launch.py')

    x_pos, y_pos, z_pos = [float(pos) for pos in position.split(' ')]

    adapter_box = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_entity),
        launch_arguments=dict(
            package_name=str(package_name),
            entity_type=str(box_type),
            entity_name=str(adapter_name),
            urdf_xacro_name='adapterbox.urdf.xacro',
            x=str(x_pos),
            y=str(y_pos),
            z=str(z_pos),
            urdf_args=adapter_type
        ).items())
    return [adapter_box]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "package_name",
            default_value="modproft_adapter_description",
        ),
        DeclareLaunchArgument(
            "box_type",
            default_value="adapterbox_bf_tech_klein"
        ),
        DeclareLaunchArgument(
            "adapter_type",
            default_value="adapter_bmt135_bmt172"  # todo: To be removed, testing only
        ),
        DeclareLaunchArgument(
            "adapter_name",
            default_value="testbox_1"   # todo: To be removed, testing only
        ),
        DeclareLaunchArgument(
            "position",
            default_value="0.0 0.0 0.0"
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
