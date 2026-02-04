from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    rpd_robot_description_pkg = get_package_share_path('rpd_robot_description')
    rpd_robot_bringup_pkg = get_package_share_path('rpd_robot_bringup')

    urdf_path = os.path.join(rpd_robot_description_pkg, 'urdf', 'rpd_robot.urdf.xacro')
    rviz_config_path = os.path.join(rpd_robot_description_pkg, 'rviz', 'urdf_config.rviz')
    robot_controllers = os.path.join(rpd_robot_bringup_pkg, 'config', 'rpd_robot_controllers.yaml')

    robot_description = ParameterValue(Command(['xacro', ' ', urdf_path]), value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['joint_state_broadcaster']
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['arm_controller']
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        rviz2
    ])