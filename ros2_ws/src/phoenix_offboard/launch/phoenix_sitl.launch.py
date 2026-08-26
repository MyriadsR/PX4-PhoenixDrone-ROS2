from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


PX4_ROOT = Path('/home/zr/PX4-PhoenixDrone-ROS2')
AGENT = Path('/home/zr/px4_ros_uxrce_dds_ws/src/install/microxrcedds_agent/bin/MicroXRCEAgent')
AGENT_LIB = Path('/home/zr/px4_ros_uxrce_dds_ws/src/install/microxrcedds_agent/lib')


def generate_launch_description() -> LaunchDescription:
    demo = LaunchConfiguration('demo')
    headless = LaunchConfiguration('headless')
    return LaunchDescription([
        DeclareLaunchArgument('demo', default_value='position'),
        DeclareLaunchArgument('headless', default_value='true'),
        ExecuteProcess(
            cmd=[str(AGENT), 'udp4', '-p', '8888'],
            additional_env={'LD_LIBRARY_PATH': f'{AGENT_LIB}:/opt/ros/jazzy/lib'},
            output='screen',
        ),
        ExecuteProcess(
            cmd=['make', 'px4_sitl_default', 'gz_phoenixdrone'],
            cwd=str(PX4_ROOT),
            additional_env={
                'HEADLESS': PythonExpression([
                    "'1' if '", headless, "'.lower() == 'true' else ''"
                ])
            },
            output='screen',
        ),
        Node(
            package='phoenix_offboard',
            executable='phoenix_controller',
            output='screen',
        ),
        Node(
            package='phoenix_offboard',
            executable='phoenix_position_controller',
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", demo, "' == 'position' or '", demo, "' == 'rectangle'"
            ])),
        ),
        Node(
            package='phoenix_offboard',
            executable='position_demo',
            output='screen',
            condition=IfCondition(PythonExpression(["'", demo, "' == 'position'"])),
        ),
        Node(
            package='phoenix_offboard',
            executable='rectangle_demo',
            output='screen',
            condition=IfCondition(PythonExpression(["'", demo, "' == 'rectangle'"])),
        ),
        Node(
            package='phoenix_offboard',
            executable='attitude_demo',
            output='screen',
            condition=IfCondition(PythonExpression(["'", demo, "' == 'attitude'"])),
        ),
    ])
