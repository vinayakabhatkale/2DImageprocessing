import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit


def launch_setup(context, *args, **kwargs):
    package_name = LaunchConfiguration('package_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    entity_type = LaunchConfiguration('entity_type').perform(context)
    entity_name = LaunchConfiguration('entity_name').perform(context)
    urdf_xacro_name = LaunchConfiguration('urdf_xacro_name')
    x_pos = float(LaunchConfiguration('x', default='0').perform(context))
    y_pos = float(LaunchConfiguration('y', default='0').perform(context))
    z_pos = float(LaunchConfiguration('z', default='0').perform(context))
    r_rot = LaunchConfiguration('R', default='0').perform(context)
    p_rot = LaunchConfiguration('P', default='0').perform(context)
    y_rot = LaunchConfiguration('Y', default='0').perform(context)

    # Load controller configurations
    entity_controllers = PathJoinSubstitution(
        [
            FindPackageShare(package_name), "config", "{}_controller.yaml".format(entity_type)
        ]
    )

    # Load entity description
    entity_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(package_name), 'urdf', urdf_xacro_name])
            ,
            " ",
            "package_name:=",
            package_name,
            " ",
            "entity_type:=",
            entity_type,
            " ",
            "entity_name:=",
            entity_name
        ]
    )
    # Necessary nodes and delays
    spawn_within_gazebo = Node(
        package="spawn_entity",
        executable="spawn_entity_node.py",
        output="both",
        arguments=['-description', entity_description,
                   '-name', entity_name,
                   '-namespace', entity_name,
                   '-x', str(x_pos / 1e3),
                   '-y', str(y_pos / 1e3),
                   '-z', str(z_pos / 1e3),
                   '-R', r_rot,
                   '-P', p_rot,
                   '-Y', y_rot]
    )
    spawn_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': entity_description},
                    entity_controllers],
        arguments=['-namespace', entity_name],
        output="both"
    )
    delay_spawn_controller_manager = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_within_gazebo,
            on_exit=[spawn_controller_manager]
        )
    )

    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=['joint_state_broadcaster',
                   '--controller-manager',
                   '{}/controller_manager'.format(entity_name)]
    )
    spawn_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': entity_description}],
        # 'tf_prefix': "{}_{}".format(pcb_type, i)}],
        # arguments=['-namespace', "{}_{}".format(pcb_type, i)]
    )

    return [spawn_within_gazebo]#, delay_spawn_controller_manager]#, spawn_jsb]#, spawn_rsp]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "package_name",
            description="Name of the package from whom the entity is to be spawned."
        ), DeclareLaunchArgument(
            "use_sim_time",
            default_value='true',
            description="Use sim time of Gazebo simulation environment.",
        ), DeclareLaunchArgument(
            "entity_type",
            description="The type of the entity."
                        "This has to correspond to file names without the respective endings!",
        ), DeclareLaunchArgument(
            "entity_name",
            description="The name and namespace of the entity.",
        ), DeclareLaunchArgument(
            "urdf_xacro_name",
            description="The full file name of the xacro which creates the URDF.",
        ), DeclareLaunchArgument(
            "x",
            description="X coordinate of the entity.",
            default_value='0.0'
        ), DeclareLaunchArgument(
            "y",
            description="Y coordinate of the entity.",
            default_value='0.0'
        ), DeclareLaunchArgument(
            "z",
            description="Z coordinate of the entity.",
            default_value='0.0'
        ), DeclareLaunchArgument(
            "R",
            description="The roll of the entity.",
            default_value='0.0'
        ), DeclareLaunchArgument(
            "P",
            description="The pitch of the entity.",
            default_value='0.0'
        ), DeclareLaunchArgument(
            "Y",
            description="The yaw of the entity.",
            default_value='0.0'
        )
    ]

    #print()
    #print('ruc')
    #for arg in declared_arguments:
    #    print(arg)
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
