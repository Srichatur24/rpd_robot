from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
import os
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():

    rpd_robot_description_pkg = get_package_share_directory('rpd_robot_description')
    rpd_robot_bringup_pkg = get_package_share_directory('rpd_robot_bringup')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    gazebo_bridge_config_file = os.path.join(rpd_robot_bringup_pkg, 'config', 'gazebo_bridge_config.yaml')
    xacro_file = os.path.join(rpd_robot_description_pkg, 'urdf', 'rpd_robot.urdf.xacro')

    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})
    robot_description = doc.toprettyxml(indent='  ')

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}],
        output="screen"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(ros_gz_sim_pkg, 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[('gz_args', ['empty.sdf', ' -r'])],
    )

    gazebo_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description'],
        output='screen'
    )

    gazebo_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': gazebo_bridge_config_file}],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['joint_state_broadcaster'],
        output="screen"
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['arm_controller'],
        output="screen"
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['gripper_controller'],
        output="screen"
    )


    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gazebo_spawn_node,
                on_exit=[joint_state_broadcaster_spawner],
        )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner, gripper_controller_spawner],
            )
        ),
        robot_state_publisher_node,
        gazebo_bridge_node,
        gazebo,
        gazebo_spawn_node,
    ])