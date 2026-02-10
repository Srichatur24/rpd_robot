from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
import os
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():

    rpd_robot_description_pkg = get_package_share_directory('rpd_robot_description')
    rpd_robot_bringup_pkg = get_package_share_directory('rpd_robot_bringup')

    DeclareLaunchArgument('port', default_value='/dev/ttyACM0')
    port = LaunchConfiguration('port')

    robot_controllers = os.path.join(rpd_robot_bringup_pkg, 'config', 'rpd_robot_controllers.yaml')
    xacro_file = os.path.join(rpd_robot_description_pkg, 'urdf', 'rpd_robot.urdf.xacro')

    doc = xacro.process_file(xacro_file, mappings={'port': str(port)})
    robot_description = doc.toprettyxml(indent='  ')


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}],
        output="screen"
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="screen"
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
                target_action=control_node,
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
        control_node
    ])