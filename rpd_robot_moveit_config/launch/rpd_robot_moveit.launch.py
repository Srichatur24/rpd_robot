from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    rpd_robot_moveit_config_package = get_package_share_directory('rpd_robot_moveit_config')
    config_folder = os.path.join(rpd_robot_moveit_config_package, 'config')
    rviz_config_file = os.path.join(config_folder, 'moveit.rviz')

    declared_arguments = [
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('use_sim', default_value='false'),
        DeclareLaunchArgument('warehouse_sqlite_path', default_value=os.path.expanduser('~/.ros/warehouse_ros.sqlite')),
        DeclareLaunchArgument('publish_robot_description_semantic', default_value='true'),
    ]

    start_rviz = LaunchConfiguration('start_rviz')
    use_sim = LaunchConfiguration('use_sim')
    warehouse_sqlite_path = LaunchConfiguration('warehouse_sqlite_path')
    publish_robot_description_semantic = LaunchConfiguration('publish_robot_description_semantic')

    moveit_config = (
        MoveItConfigsBuilder(robot_name='rpd_robot', package_name='rpd_robot_moveit_config')
            .robot_description_semantic(os.path.join(config_folder, 'rpd_robot.srdf'))
            .joint_limits(os.path.join(config_folder, 'joint_limits.yaml'))
            .trajectory_execution(os.path.join(config_folder, 'moveit_controllers.yaml'))
            .robot_description_kinematics(os.path.join(config_folder, 'kinematics.yaml'))
            .to_moveit_configs()
    )

    warehouse_ros_config = {
        'warehouse_plugin': 'warehouse_ros_sqlite::DatabaseConnection',
        'warehouse_host': warehouse_sqlite_path,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            {
                'use_sim_time': use_sim,
                'publish_robot_description_semantic': publish_robot_description_semantic,
            },
        ],
    )

    rviz_node = Node(
        package='rviz2',
        condition=IfCondition(start_rviz),
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {
                'use_sim_time': use_sim,
            },
        ],
    )

    return LaunchDescription([*declared_arguments, move_group_node, rviz_node])