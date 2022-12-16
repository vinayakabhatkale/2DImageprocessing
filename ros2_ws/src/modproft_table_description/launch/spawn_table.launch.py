import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    package_name_lc = LaunchConfiguration("package_name")
    package_name = package_name_lc.perform(context)
    table_type = LaunchConfiguration("table_type")
    table_type = table_type.perform(context)
    table_name = LaunchConfiguration("table_name")
    table_name = table_name.perform(context)
    position_lc = LaunchConfiguration("position")
    position = position_lc.perform(context)
    attachment_lc = LaunchConfiguration("attachment_link")
    attachment = attachment_lc.perform(context)

    spawn_entity = os.path.join(get_package_share_directory('spawn_entity'), 'launch',
                                'spawn_entity.launch.py')

    x_pos, y_pos, z_pos = [float(pos) for pos in position.split(' ')]

    urdf_arguments = {"prefix": "", "attachment_link": str(attachment)}

    file_name = "{}_temp.yaml".format(str(table_name))
    file_path = os.path.join(
        get_package_share_directory(package_name),
        'temp'
    )
    full_path = os.path.join(
        get_package_share_directory(package_name),
        'temp',
        file_name
    )

    if not os.path.exists(file_path):
        os.makedirs(file_path)

    with open(full_path, 'w') as file:
        yaml.dump(urdf_arguments, file)

    table = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_entity),
        launch_arguments=dict(
            package_name=str(package_name),
            entity_type=str(table_type),
            entity_name=str(table_name),
            urdf_xacro_name='_table.urdf.xacro',
            x=str(x_pos),
            y=str(y_pos),
            z=str(z_pos),
            urdf_args=full_path
        ).items())
    return [table]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "package_name",
            default_value="modproft_table_description",
        ),
        DeclareLaunchArgument(
            "table_type",
            default_value="box"
        ),
        DeclareLaunchArgument(
            "table_name"
        ),
        DeclareLaunchArgument(
            "position",
            default_value="0.0 0.0 0.0"
        ),
        DeclareLaunchArgument(
            "attachment_link",
            default_value="world"
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
