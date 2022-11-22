import os

import numpy as np
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    package_name_lc = LaunchConfiguration("package_name")
    package_name = package_name_lc.perform(context)
    pcb_type_lc = LaunchConfiguration("pcb_type")
    pcb_type = pcb_type_lc.perform(context)
    number_lc = LaunchConfiguration("number")
    number = number_lc.perform(context)
    offset_lc = LaunchConfiguration("offset")
    offset = offset_lc.perform(context)

    spawn_entity = os.path.join(get_package_share_directory('spawn_entity'), 'launch',
                                'spawn_entity.launch.py')

    # Coordinates
    tablar_size = (548, 348)  # in mm
    tablar_border = (10, 10)  # in mm
    start_point = (10, 0, 0)  # in mm
    distance = (170, 160, 0)  # in mm
    x_positions = np.arange(
        tablar_border[0] + start_point[0],
        tablar_size[0] - tablar_border[0],
        distance[0]
    ).tolist()
    y_positions = np.arange(
        tablar_border[1] + start_point[1],
        tablar_size[1] - tablar_border[1],
        distance[1]
    ).tolist()
    raster = []

    # Fill coordinates to raster
    assert isinstance(x_positions, list)
    assert isinstance(y_positions, list)
    for y in y_positions:
        for x in x_positions:
            raster.append((x, y, start_point[2]))

    spawned_pcbs = []
    for index in range(int(number)):
        if index < len(raster):
            x_pos, y_pos, z_pos = raster[index]
            x_off, y_off, z_off = [float(off) for off in offset.split(' ')]
            spawned_pcbs.append(
                IncludeLaunchDescription(PythonLaunchDescriptionSource(spawn_entity),
                                         launch_arguments=dict(
                                             package_name=str(package_name),
                                             entity_type=str(pcb_type),
                                             entity_name='{}_{}'.format(pcb_type, index),
                                             urdf_xacro_name='pcb.urdf.xacro',
                                             x=str(x_pos + x_off),
                                             y=str(y_pos + y_off),
                                             z=str(z_pos + z_off),
                                             urdf_args="None"
                                         ).items())
            )
    return spawned_pcbs


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "package_name",
            default_value="modproft_pcb_description",
        ),
        DeclareLaunchArgument(
            "pcb_type",
            default_value="bmt135"
        ),
        DeclareLaunchArgument(
            "number",
            default_value="5"
        ),
        DeclareLaunchArgument(
            "offset",
            default_value="0.0 0.0 0.0"
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
